
#include "buoy_examples/torque_control_policy.hpp"


PBTorqueController::PBTorqueController(const std::string &node_name)
  : PBInterface::PBController<PBTorqueController>(node_name)
{
  policy_.reset(new PBTorqueControlPolicy());
  set_params();

  set_pc_pack_rate();
}

void PBTorqueController::power_callback(const buoy_msgs::msg::PCRecord &data)
{
  auto request = std::make_shared<buoy_msgs::srv::PCWindCurrCommand::Request>();
  request->wind_curr = policy_->WindingCurrentTarget(data.rpm, data.scale, data.retract);;

  auto response = pc_wind_curr_client_->async_send_request(request, pc_wind_curr_callback);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PBTorqueController>("pb_torque_controller"));
  rclcpp::shutdown();

  return 0;
}
