// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <chrono>
#include <map>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

//interp1d for rpm->winding current
#include "JustInterp/JustInterp.hpp"

// pbsrv commands
#include "buoy_msgs/srv/pc_pack_rate_command.hpp"
#include "buoy_msgs/srv/pc_wind_curr_command.hpp"
#include "buoy_msgs/srv/pc_retract_command.hpp"
#include "buoy_msgs/srv/pc_scale_command.hpp"

// power converter feedback data
#include "buoy_msgs/msg/pc_record.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;


std::map<int8_t, std::string> pbsrv_enum2str = {{0, "OK"},
                                                {-1, "BAD_SOCK"},
                                                {-2, "BAD_OPTS"},
                                                {-3, "BAD_INPUT"}};


struct PBTorqueControlPolicy
{
  float Torque_constant;  // N-m/Amps
  std::vector<float> N_Spec;  // RPM
  std::vector<float> Torque_Spec;  // N-m
  std::vector<float> I_Spec;  // Amps
  JustInterp::LinearInterpolator<float> interp1d;

  PBTorqueControlPolicy()
   : Torque_constant(0.438F),
     N_Spec{0.0F, 300.0F, 600.0F, 1000.0F, 1700.0F, 4400.0F, 6790.0F},
     Torque_Spec{0.0F, 0.0F, 0.8F, 2.9F, 5.6F, 9.8F, 16.6F},  // Matches old boost converter targets that have been deployed.
     I_Spec(Torque_Spec.size(), 0.0F)
  {
    update_params();
  }

  void update_params()
  {
    std::transform(Torque_Spec.cbegin(), Torque_Spec.cend(),
                   I_Spec.begin(),
                   [tc=Torque_constant](const float &ts){ return ts/tc; });

    interp1d.SetData(N_Spec, I_Spec);
  }

  float WindingCurrentTarget(const float &rpm, const float &scale_factor, const float &retract_factor)
  {
    float N = fabs(rpm);
    float I = 0.0F;

    if (N>=N_Spec.back())
    {
      I = I_Spec.back();
    }
    else
    {
      I = interp1d(N);
    }

    I *= scale_factor;
    if (rpm>0.0F)
    {
      I *= -retract_factor;
    }

    return I;
  }
};

std::ostream& operator<<(std::ostream& os, const PBTorqueControlPolicy &policy)
{
  os << "PBTorqueControlPolicy:" << std::endl;

  os << "\tTorque_constant: " << policy.Torque_constant << std::endl;

  os << "\tN_Spec: " << std::flush;
  std::copy(policy.N_Spec.cbegin(), policy.N_Spec.cend(), std::ostream_iterator<float>(os, ","));
  os << "\b \b" << std::endl;

  os << "\tTorque_Spec: " << std::flush;
  std::copy(policy.Torque_Spec.cbegin(), policy.Torque_Spec.cend(), std::ostream_iterator<float>(os, ","));
  os << "\b \b" << std::endl;

  os << "\tI_Spec: " << std::flush;
  std::copy(policy.I_Spec.cbegin(), policy.I_Spec.cend(), std::ostream_iterator<float>(os, ","));
  os << "\b \b" << std::endl;

  return os;
}

class PBTorqueController : public rclcpp::Node
{
public:
  PBTorqueController()
    : Node("pb_torque_controller"),
      sub_count_(0U),
      cmd_count_(0U)
  {
    set_params();

    pack_rate_client_ = this->create_client<buoy_msgs::srv::PCPackRateCommand>("/pc_pack_rate_command");
    wind_curr_client_ = this->create_client<buoy_msgs::srv::PCWindCurrCommand>("/pc_wind_curr_command");

    bool found = wait_for_service(pack_rate_client_, "/pc_pack_rate_command");
    found &= wait_for_service(wind_curr_client_, "/pc_wind_curr_command");

    if (!found)
    {
      RCLCPP_ERROR(rclcpp::get_logger("pb_torque_controller"), "Did not find required services");
      return;
    }
    set_pc_pack_rate();

    power_data_sub_ = this->create_subscription<buoy_msgs::msg::PCRecord>("/power_data", 1,
        std::bind(&PBTorqueController::rpm_callback, this, _1));

  }

private:

  template <class T>
  bool wait_for_service(T &client, const std::string &service)
  {
    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("pb_torque_controller"),
                     "Interrupted while waiting for %s. Exiting.",
                     service.c_str());
        return false;
      }
      RCLCPP_INFO(rclcpp::get_logger("pb_torque_controller"),
                  "%s not available, still waiting...",
                  service.c_str());
    }
    return true;
  }

  void set_params()
  {
    this->declare_parameter("Torque_constant", policy_.Torque_constant);
    policy_.Torque_constant = this->get_parameter("Torque_constant").as_double();

    this->declare_parameter("N_Spec", std::vector<double>(policy_.N_Spec.begin(), policy_.N_Spec.end()));
    std::vector<double> temp_double_arr = this->get_parameter("N_Spec").as_double_array();
    policy_.N_Spec.assign(temp_double_arr.begin(), temp_double_arr.end());

    this->declare_parameter("Torque_Spec", std::vector<double>(policy_.Torque_Spec.begin(), policy_.Torque_Spec.end()));
    temp_double_arr = this->get_parameter("Torque_Spec").as_double_array();
    policy_.Torque_Spec.assign(temp_double_arr.begin(), temp_double_arr.end());

    policy_.update_params();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pb_torque_controller"), policy_);
  }

  void set_pc_pack_rate()
  {
    auto request = std::make_shared<buoy_msgs::srv::PCPackRateCommand::Request>();
    request->rate_hz = 50;

    using RateServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::PCPackRateCommand>::SharedFuture;
    auto pack_rate_response_callback = [this](RateServiceResponseFuture future)
    {
      if (future.get()->result.value==future.get()->result.OK)
      {
        RCLCPP_INFO(rclcpp::get_logger("pb_torque_controller"),
                    "Successfully set /pc_pack_rate_command to 50Hz");
        this->cmd_start_ = this->sub_start_ = this->now();
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("pb_torque_controller"),
                     "Failed to set /pc_pack_rate_command to 50Hz: received error code [[ %s ]]",
                     pbsrv_enum2str[future.get()->result.value].c_str());
        //TODO: should we shutdown?
      }
    };

    auto response = pack_rate_client_->async_send_request(request, pack_rate_response_callback);

  }

  void rpm_callback(const buoy_msgs::msg::PCRecord &power)
  {
    this->hz(std::string("feedback_rate"), sub_count_, sub_start_);

    const float I = policy_.WindingCurrentTarget(power.rpm, power.scale, power.retract);

    //RCLCPP_INFO_STREAM(rclcpp::get_logger("pb_torque_controller"), rosidl_generator_traits::to_yaml(power));
    //RCLCPP_INFO_STREAM(rclcpp::get_logger("pb_torque_controller"), "power_data -- status: " << power.status);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pb_torque_controller"), "power_data -- rpm: " << power.rpm);
    //RCLCPP_INFO_STREAM(rclcpp::get_logger("pb_torque_controller"), "power_data -- bias_current: " << power.bias_current);
    //RCLCPP_INFO_STREAM(rclcpp::get_logger("pb_torque_controller"), "power_data -- scale_factor: " << power.scale);
    //RCLCPP_INFO_STREAM(rclcpp::get_logger("pb_torque_controller"), "power_data -- retract_factor: " << power.retract);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pb_torque_controller"), "power_data -- I: " << power.target_a);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pb_torque_controller"), "torque_controller -- I: " << I);

    auto request = std::make_shared<buoy_msgs::srv::PCWindCurrCommand::Request>();
    request->wind_curr = I;

    using WindCurrServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::PCWindCurrCommand>::SharedFuture;
    auto wind_curr_response_callback = [this](WindCurrServiceResponseFuture future)
    {
      if (future.get()->result.value==future.get()->result.OK)
      {
        RCLCPP_INFO(rclcpp::get_logger("pb_torque_controller"),
                    "Successfully set /pc_wind_curr_command");
        this->hz(std::string("command_rate"), cmd_count_, cmd_start_);
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("pb_torque_controller"),
                    "Failed to set /pc_wind_curr_command: received error code [[ %s ]]",
                    pbsrv_enum2str[future.get()->result.value].c_str());
        //TODO: should we shutdown?
      }
    };

    auto response = wind_curr_client_->async_send_request(request, wind_curr_response_callback);

  }

  void hz(const std::string &type, uint16_t &count, rclcpp::Time &start)
  {
    count += 1U;
    if(count % 500U == 0U)
    {
      count = 0U;
      start = this->now();
    }
    else
    {
      rclcpp::Time now = this->now();
      float elapsed_sec = ((now-start).nanoseconds()/1e9);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("pb_torque_controller"),
                         type << " : " << count/elapsed_sec);
    }

  }

  PBTorqueControlPolicy policy_;

  uint16_t sub_count_, cmd_count_;
  rclcpp::Time sub_start_, cmd_start_;
  rclcpp::Subscription<buoy_msgs::msg::PCRecord>::SharedPtr power_data_sub_;
  rclcpp::Client<buoy_msgs::srv::PCPackRateCommand>::SharedPtr pack_rate_client_;
  rclcpp::Client<buoy_msgs::srv::PCWindCurrCommand>::SharedPtr wind_curr_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PBTorqueController>());
  rclcpp::shutdown();

  return 0;
}
