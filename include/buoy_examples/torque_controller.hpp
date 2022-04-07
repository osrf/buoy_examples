#ifndef TORQUE_CONTROLLER_HPP_
#define TORQUE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

// pbsrv commands
#include "buoy_msgs/srv/pc_pack_rate_command.hpp"
#include "buoy_msgs/srv/pc_wind_curr_command.hpp"

// power converter feedback data
#include "buoy_msgs/msg/pc_record.hpp"

// forward declare
struct PBTorqueControlPolicy;  //defined by user in torque_control_policy.hpp

class PBTorqueController : public rclcpp::Node
{
public:
  PBTorqueController();

private:

  template <class T>
  bool wait_for_service(T &client, const std::string &service);

  void set_params();  //defined by user in torque_control_policy.hpp

  //TODO: maybe add functions for all microcontrollers' pack rates
  void set_pc_pack_rate();

  //TODO: make this generic...
  void rpm_callback(const buoy_msgs::msg::PCRecord &power);

  std::unique_ptr<PBTorqueControlPolicy> policy_;

  //TODO make generic...
  rclcpp::Subscription<buoy_msgs::msg::PCRecord>::SharedPtr power_data_sub_;

  //TODO add clients for all commands a user is allowed to send
  rclcpp::Client<buoy_msgs::srv::PCPackRateCommand>::SharedPtr pack_rate_client_;
  rclcpp::Client<buoy_msgs::srv::PCWindCurrCommand>::SharedPtr wind_curr_client_;

};

#endif //TORQUE_CONTROLLER_HPP_
