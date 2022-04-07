#include "buoy_examples/torque_controller.hpp"
#include "buoy_examples/torque_control_policy.hpp"

#include <chrono>
#include <map>

using std::placeholders::_1;
using namespace std::chrono_literals;


std::map<int8_t, std::string> pbsrv_enum2str = {{0, "OK"},
                                                {-1, "BAD_SOCK"},
                                                {-2, "BAD_OPTS"},
                                                {-3, "BAD_INPUT"}};

PBTorqueController::PBTorqueController()
  : Node("pb_torque_controller")
{
  policy_.reset(new PBTorqueControlPolicy());

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

template <class T>
bool PBTorqueController::wait_for_service(T &client, const std::string &service)
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

void PBTorqueController::set_pc_pack_rate()
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

void PBTorqueController::rpm_callback(const buoy_msgs::msg::PCRecord &power)
{
  //TODO: make this generic...
  const float I = policy_->WindingCurrentTarget(power.rpm, power.scale, power.retract);

  auto request = std::make_shared<buoy_msgs::srv::PCWindCurrCommand::Request>();
  request->wind_curr = I;

  using WindCurrServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::PCWindCurrCommand>::SharedFuture;
  auto wind_curr_response_callback = [this](WindCurrServiceResponseFuture future)
  {
    if (future.get()->result.value==future.get()->result.OK)
    {
      RCLCPP_INFO(rclcpp::get_logger("pb_torque_controller"),
                  "Successfully set /pc_wind_curr_command");
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PBTorqueController>());
  rclcpp::shutdown();

  return 0;
}
