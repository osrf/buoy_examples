#ifndef PB_CONTROLLER_HPP_
#define PB_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

// pbsrv commands
// power microcontroller
#include "buoy_msgs/srv/pc_batt_switch_command.hpp"
#include "buoy_msgs/srv/pc_bias_curr_command.hpp"
#include "buoy_msgs/srv/pc_charge_curr_lim_command.hpp"
#include "buoy_msgs/srv/pc_draw_curr_lim_command.hpp"
#include "buoy_msgs/srv/pc_pack_rate_command.hpp"
#include "buoy_msgs/srv/pc_retract_command.hpp"
#include "buoy_msgs/srv/pc_scale_command.hpp"
#include "buoy_msgs/srv/pc_std_dev_targ_command.hpp"
#include "buoy_msgs/srv/pcv_targ_max_command.hpp"
#include "buoy_msgs/srv/pc_wind_curr_command.hpp"
#include "buoy_msgs/srv/gain_command.hpp"

// battery microcontroller
#include "buoy_msgs/srv/bc_reset_command.hpp"

// spring microcontroller
#include "buoy_msgs/srv/sc_pack_rate_command.hpp"
#include "buoy_msgs/srv/sc_reset_command.hpp"
#include "buoy_msgs/srv/valve_command.hpp"
#include "buoy_msgs/srv/pump_command.hpp"
#include "buoy_msgs/srv/bender_command.hpp"
#include "buoy_msgs/srv/tether_command.hpp"

// trefoil microcontroller
#include "buoy_msgs/srv/tf_reset_command.hpp"
#include "buoy_msgs/srv/tf_set_actual_pos_command.hpp"
#include "buoy_msgs/srv/tf_set_charge_mode_command.hpp"
#include "buoy_msgs/srv/tf_set_curr_lim_command.hpp"
#include "buoy_msgs/srv/tf_set_mode_command.hpp"
#include "buoy_msgs/srv/tf_set_pos_command.hpp"
#include "buoy_msgs/srv/tf_set_state_machine_command.hpp"
#include "buoy_msgs/srv/tf_watch_dog_command.hpp"


// pb telemetry
#include "buoy_msgs/msg/xb_record.hpp"  // ahrs
#include "buoy_msgs/msg/bc_record.hpp"  // battery
#include "buoy_msgs/msg/sc_record.hpp"  // spring
#include "buoy_msgs/msg/pc_record.hpp"  // power
#include "buoy_msgs/msg/tf_record.hpp"  // trefoil
#include "buoy_msgs/msg/pb_record.hpp"  // consolidated

#include <map>
#include <chrono>


namespace PBInterface
{

using std::placeholders::_1;
using namespace std::chrono_literals;


std::map<int8_t, std::string> pbsrv_enum2str = {{0, "OK"},
                                                {-1, "BAD_SOCK"},
                                                {-2, "BAD_OPTS"},
                                                {-3, "BAD_INPUT"}};


template<class ControllerImplCRTP>
class PBController : public rclcpp::Node
{
public:
  using CRTP = PBController;
  PBController(const std::string &node_name)
    : Node(node_name),
      with_ahrs(false),
      with_battery(false),
      with_spring(false),
      with_power(false),
      with_trefoil(false),
      with_all(false)
  {
    pc_pack_rate_client_ = this->create_client<buoy_msgs::srv::PCPackRateCommand>("/pc_pack_rate_command");
    pc_wind_curr_client_ = this->create_client<buoy_msgs::srv::PCWindCurrCommand>("/pc_wind_curr_command");
    bender_client_  = this->create_client<buoy_msgs::srv::BenderCommand>("/bender_command");
    bc_reset_client_  = this->create_client<buoy_msgs::srv::BCResetCommand>("/bc_reset_command");
    pump_client_  = this->create_client<buoy_msgs::srv::PumpCommand>("/pump_command");
    valve_client_  = this->create_client<buoy_msgs::srv::ValveCommand>("/valve_command");
    tether_client_  = this->create_client<buoy_msgs::srv::TetherCommand>("/tether_command");
    sc_reset_client_  = this->create_client<buoy_msgs::srv::SCResetCommand>("/sc_reset_command");
    sc_pack_rate_client_  = this->create_client<buoy_msgs::srv::SCPackRateCommand>("/sc_pack_rate_command");
    pc_scale_client_  = this->create_client<buoy_msgs::srv::PCScaleCommand>("/pc_scale_command");
    pc_retract_client_  = this->create_client<buoy_msgs::srv::PCRetractCommand>("/pc_retract_command");
    pc_v_targ_max_client_  = this->create_client<buoy_msgs::srv::PCVTargMaxCommand>("/pc_v_targ_max_command");
    pc_charge_curr_lim_client_  = this->create_client<buoy_msgs::srv::PCChargeCurrLimCommand>("/pc_charge_curr_lim_command");
    pc_batt_switch_client_  = this->create_client<buoy_msgs::srv::PCBattSwitchCommand>("/pc_batt_switch_command");
    gain_client_  = this->create_client<buoy_msgs::srv::GainCommand>("/gain_command");
    pc_std_dev_targ_client_  = this->create_client<buoy_msgs::srv::PCStdDevTargCommand>("/pc_std_dev_targ_command");
    pc_draw_curr_lim_client_  = this->create_client<buoy_msgs::srv::PCDrawCurrLimCommand>("/pc_draw_curr_lim_command");
    pc_bias_curr_client_  = this->create_client<buoy_msgs::srv::PCBiasCurrCommand>("/pc_bias_curr_command");
    tf_set_pos_client_  = this->create_client<buoy_msgs::srv::TFSetPosCommand>("/tf_set_pos_command");
    tf_set_actual_pos_client_  = this->create_client<buoy_msgs::srv::TFSetActualPosCommand>("/tf_set_actual_pos_command");
    tf_set_mode_client_  = this->create_client<buoy_msgs::srv::TFSetModeCommand>("/tf_set_mode_command");
    tf_set_charge_mode_client_  = this->create_client<buoy_msgs::srv::TFSetChargeModeCommand>("/tf_set_charge_mode_command");
    tf_set_curr_lim_client_  = this->create_client<buoy_msgs::srv::TFSetCurrLimCommand>("/tf_set_curr_lim_command");
    tf_set_state_machine_client_  = this->create_client<buoy_msgs::srv::TFSetStateMachineCommand>("/tf_set_state_machine_command");
    tf_watch_dog_client_  = this->create_client<buoy_msgs::srv::TFWatchDogCommand>("/tf_watch_dog_command");
    tf_reset_client_  = this->create_client<buoy_msgs::srv::TFResetCommand>("/tf_reset_command");

    bool found = wait_for_service(pc_pack_rate_client_, "/pc_pack_rate_command");
    found &= wait_for_service(pc_wind_curr_client_, "/pc_wind_curr_command");
    found &= wait_for_service(bender_client_, "/bender_command");
    found &= wait_for_service(bc_reset_client_, "/bc_reset_command");
    found &= wait_for_service(pump_client_, "/pump_command");
    found &= wait_for_service(valve_client_, "/valve_command");
    found &= wait_for_service(tether_client_, "/tether_command");
    found &= wait_for_service(sc_reset_client_, "/sc_reset_command");
    found &= wait_for_service(sc_pack_rate_client_, "/sc_pack_rate_command");
    found &= wait_for_service(pc_scale_client_, "/pc_scale_command");
    found &= wait_for_service(pc_retract_client_, "/pc_retract_command");
    found &= wait_for_service(pc_v_targ_max_client_, "/pc_v_targ_max_command");
    found &= wait_for_service(pc_charge_curr_lim_client_, "/pc_charge_curr_lim_command");
    found &= wait_for_service(pc_batt_switch_client_, "/pc_batt_switch_command");
    found &= wait_for_service(gain_client_, "/gain_command");
    found &= wait_for_service(pc_std_dev_targ_client_, "/pc_std_dev_targ_command");
    found &= wait_for_service(pc_draw_curr_lim_client_, "/pc_draw_curr_lim_command");
    found &= wait_for_service(pc_bias_curr_client_, "/pc_bias_curr_command");
    found &= wait_for_service(tf_set_pos_client_, "/tf_set_pos_command");
    found &= wait_for_service(tf_set_actual_pos_client_, "/tf_set_actual_pos_command");
    found &= wait_for_service(tf_set_mode_client_, "/tf_set_mode_command");
    found &= wait_for_service(tf_set_charge_mode_client_, "/tf_set_charge_mode_command");
    found &= wait_for_service(tf_set_curr_lim_client_, "/tf_set_curr_lim_command");
    found &= wait_for_service(tf_set_state_machine_client_, "/tf_set_state_machine_command");
    found &= wait_for_service(tf_watch_dog_client_, "/tf_watch_dog_command");
    found &= wait_for_service(tf_reset_client_, "/tf_reset_command");

    if (!found)
    {
      RCLCPP_ERROR(rclcpp::get_logger(node_name), "Did not find required services");
      return;
    }
  }
  
  void setup_subscribers()
  {
    if (this->with_ahrs)
    {
      ahrs_data_sub_ = this->create_subscription<buoy_msgs::msg::XBRecord>("/ahrs_data", 1,
          std::bind(&ControllerImplCRTP::ahrs_callback, static_cast<ControllerImplCRTP*>(this), _1));
    }
    if (this->with_battery)
    {
      battery_data_sub_ = this->create_subscription<buoy_msgs::msg::BCRecord>("/battery_data", 1,
            std::bind(&ControllerImplCRTP::battery_callback, static_cast<ControllerImplCRTP*>(this), _1));
    }
    if (this->with_spring)
    {
      spring_data_sub_ = this->create_subscription<buoy_msgs::msg::SCRecord>("/spring_data", 1,
          std::bind(&ControllerImplCRTP::spring_callback, static_cast<ControllerImplCRTP*>(this), _1));
    }
    if (this->with_power)
    {
      power_data_sub_ = this->create_subscription<buoy_msgs::msg::PCRecord>("/power_data", 1,
          std::bind(&ControllerImplCRTP::power_callback, static_cast<ControllerImplCRTP*>(this), _1));
    }
    if (this->with_trefoil)
    {
      trefoil_data_sub_ = this->create_subscription<buoy_msgs::msg::TFRecord>("/trefoil_data", 1,
          std::bind(&ControllerImplCRTP::trefoil_callback, static_cast<ControllerImplCRTP*>(this), _1));
    }
    if (this->with_all)
    {
      powerbuoy_data_sub_ = this->create_subscription<buoy_msgs::msg::PBRecord>("/powerbuoy_data", 1,
          std::bind(&ControllerImplCRTP::powerbuoy_callback, static_cast<ControllerImplCRTP*>(this), _1));
    }
  }

  void set_pc_pack_rate()
  {
    auto request = std::make_shared<buoy_msgs::srv::PCPackRateCommand::Request>();
    request->rate_hz = 50;

    auto pack_rate_response_callback = [this](PCPackRateServiceResponseFuture future)
    {
      if (future.get()->result.value==future.get()->result.OK)
      {
        RCLCPP_INFO(rclcpp::get_logger(this->get_name()),
                    "Successfully set /pc_pack_rate_command to 50Hz");
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger(this->get_name()),
                     "Failed to set /pc_pack_rate_command to 50Hz: received error code [[ %s ]]",
                     pbsrv_enum2str[future.get()->result.value].c_str());
        //TODO: should we shutdown?
      }
    };

    auto response = pc_pack_rate_client_->async_send_request(request, pack_rate_response_callback);

  }

  void set_sc_pack_rate()
  {
    auto request = std::make_shared<buoy_msgs::srv::SCPackRateCommand::Request>();
    request->rate_hz = 50;

    auto pack_rate_response_callback = [this](SCPackRateServiceResponseFuture future)
    {
      if (future.get()->result.value==future.get()->result.OK)
      {
        RCLCPP_INFO(rclcpp::get_logger(this->get_name()),
                    "Successfully set /sc_pack_rate_command to 50Hz");
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger(this->get_name()),
                     "Failed to set /sc_pack_rate_command to 50Hz: received error code [[ %s ]]",
                     pbsrv_enum2str[future.get()->result.value].c_str());
        //TODO: should we shutdown?
      }
    };

    auto response = sc_pack_rate_client_->async_send_request(request, pack_rate_response_callback);

  }

protected:
  virtual ~PBController() = default;
  
  virtual void set_params() {};  // defined by user

  virtual void ahrs_callback(const buoy_msgs::msg::XBRecord &) {};
  virtual void battery_callback(const buoy_msgs::msg::BCRecord &) {};
  virtual void spring_callback(const buoy_msgs::msg::SCRecord &) {};
  virtual void power_callback(const buoy_msgs::msg::PCRecord &) {};
  virtual void trefoil_callback(const buoy_msgs::msg::TFRecord &) {};
  virtual void powerbuoy_callback(const buoy_msgs::msg::PBRecord &) {};

  using BenderServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::BenderCommand>::SharedFuture;
  using BCResetServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::BCResetCommand>::SharedFuture;
  using PumpServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::PumpCommand>::SharedFuture;
  using ValveServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::ValveCommand>::SharedFuture;
  using TetherServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::TetherCommand>::SharedFuture;
  using SCResetServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::SCResetCommand>::SharedFuture;
  using SCPackRateServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::SCPackRateCommand>::SharedFuture;
  using PCScaleServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::PCScaleCommand>::SharedFuture;
  using PCRetractServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::PCRetractCommand>::SharedFuture;
  using PCVTargMaxServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::PCVTargMaxCommand>::SharedFuture;
  using PCChargeCurrLimServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::PCChargeCurrLimCommand>::SharedFuture;
  using PCBattSwitchServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::PCBattSwitchCommand>::SharedFuture;
  using GainServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::GainCommand>::SharedFuture;
  using PCStdDevTargServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::PCStdDevTargCommand>::SharedFuture;
  using PCDrawCurrLimServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::PCDrawCurrLimCommand>::SharedFuture;
  using PCWindCurrServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::PCWindCurrCommand>::SharedFuture;
  using PCBiasCurrServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::PCBiasCurrCommand>::SharedFuture;
  using PCPackRateServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::PCPackRateCommand>::SharedFuture;
  using TFSetPosServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::TFSetPosCommand>::SharedFuture;
  using TFSetActualPosServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::TFSetActualPosCommand>::SharedFuture;
  using TFSetModeServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::TFSetModeCommand>::SharedFuture;
  using TFSetChargeModeServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::TFSetChargeModeCommand>::SharedFuture;
  using TFSetCurrLimServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::TFSetCurrLimCommand>::SharedFuture;
  using TFSetStateMachineServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::TFSetStateMachineCommand>::SharedFuture;
  using TFWatchDogServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::TFWatchDogCommand>::SharedFuture;
  using TFResetServiceResponseFuture = rclcpp::Client<buoy_msgs::srv::TFResetCommand>::SharedFuture;

  bool with_ahrs, with_battery, with_spring, with_power, with_trefoil, with_all;

  rclcpp::Client<buoy_msgs::srv::BenderCommand>::SharedPtr bender_client_;
  rclcpp::Client<buoy_msgs::srv::BCResetCommand>::SharedPtr bc_reset_client_;
  rclcpp::Client<buoy_msgs::srv::PumpCommand>::SharedPtr pump_client_;
  rclcpp::Client<buoy_msgs::srv::ValveCommand>::SharedPtr valve_client_;
  rclcpp::Client<buoy_msgs::srv::TetherCommand>::SharedPtr tether_client_;
  rclcpp::Client<buoy_msgs::srv::SCResetCommand>::SharedPtr sc_reset_client_;
  rclcpp::Client<buoy_msgs::srv::SCPackRateCommand>::SharedPtr sc_pack_rate_client_;
  rclcpp::Client<buoy_msgs::srv::PCScaleCommand>::SharedPtr pc_scale_client_;
  rclcpp::Client<buoy_msgs::srv::PCRetractCommand>::SharedPtr pc_retract_client_;
  rclcpp::Client<buoy_msgs::srv::PCVTargMaxCommand>::SharedPtr pc_v_targ_max_client_;
  rclcpp::Client<buoy_msgs::srv::PCChargeCurrLimCommand>::SharedPtr pc_charge_curr_lim_client_;
  rclcpp::Client<buoy_msgs::srv::PCBattSwitchCommand>::SharedPtr pc_batt_switch_client_;
  rclcpp::Client<buoy_msgs::srv::GainCommand>::SharedPtr gain_client_;
  rclcpp::Client<buoy_msgs::srv::PCStdDevTargCommand>::SharedPtr pc_std_dev_targ_client_;
  rclcpp::Client<buoy_msgs::srv::PCDrawCurrLimCommand>::SharedPtr pc_draw_curr_lim_client_;
  rclcpp::Client<buoy_msgs::srv::PCWindCurrCommand>::SharedPtr pc_wind_curr_client_;
  rclcpp::Client<buoy_msgs::srv::PCBiasCurrCommand>::SharedPtr pc_bias_curr_client_;
  rclcpp::Client<buoy_msgs::srv::PCPackRateCommand>::SharedPtr pc_pack_rate_client_;
  rclcpp::Client<buoy_msgs::srv::TFSetPosCommand>::SharedPtr tf_set_pos_client_;
  rclcpp::Client<buoy_msgs::srv::TFSetActualPosCommand>::SharedPtr tf_set_actual_pos_client_;
  rclcpp::Client<buoy_msgs::srv::TFSetModeCommand>::SharedPtr tf_set_mode_client_;
  rclcpp::Client<buoy_msgs::srv::TFSetChargeModeCommand>::SharedPtr tf_set_charge_mode_client_;
  rclcpp::Client<buoy_msgs::srv::TFSetCurrLimCommand>::SharedPtr tf_set_curr_lim_client_;
  rclcpp::Client<buoy_msgs::srv::TFSetStateMachineCommand>::SharedPtr tf_set_state_machine_client_;
  rclcpp::Client<buoy_msgs::srv::TFWatchDogCommand>::SharedPtr tf_watch_dog_client_;
  rclcpp::Client<buoy_msgs::srv::TFResetCommand>::SharedPtr tf_reset_client_;

private:

  template <class T>
  bool wait_for_service(T &client, const std::string &service)
  {
    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger(this->get_name()),
                     "Interrupted while waiting for %s. Exiting.",
                     service.c_str());
        return false;
      }
      RCLCPP_INFO(rclcpp::get_logger(this->get_name()),
                  "%s not available, still waiting...",
                  service.c_str());
    }
    return true;
  }

  rclcpp::Subscription<buoy_msgs::msg::XBRecord>::SharedPtr ahrs_data_sub_;
  rclcpp::Subscription<buoy_msgs::msg::BCRecord>::SharedPtr battery_data_sub_;
  rclcpp::Subscription<buoy_msgs::msg::SCRecord>::SharedPtr spring_data_sub_;
  rclcpp::Subscription<buoy_msgs::msg::PCRecord>::SharedPtr power_data_sub_;
  rclcpp::Subscription<buoy_msgs::msg::TFRecord>::SharedPtr trefoil_data_sub_;
  rclcpp::Subscription<buoy_msgs::msg::PBRecord>::SharedPtr powerbuoy_data_sub_;

};

} //PBInterface

#endif //PB_CONTROLLER_HPP_
