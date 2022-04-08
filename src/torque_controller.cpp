
#include "buoy_examples/torque_controller.hpp"
#include "buoy_examples/torque_control_policy.hpp"


PBTorqueController::PBTorqueController(const std::string &node_name)
  : PBInterface::PBController<PBTorqueController>(node_name)
{
  policy_.reset(new PBTorqueControlPolicy());

  set_params();
  setup_subscribers();
}

void PBTorqueController::power_callback(const buoy_msgs::msg::PCRecord &data)
{
  //TODO: make this generic...
  const float I = policy_->WindingCurrentTarget(data.rpm, data.scale, data.retract);

  auto request = std::make_shared<buoy_msgs::srv::PCWindCurrCommand::Request>();
  request->wind_curr = I;

  auto wind_curr_response_callback = [this](PCWindCurrServiceResponseFuture future)
  {
    if (future.get()->result.value==future.get()->result.OK)
    {
      RCLCPP_INFO(rclcpp::get_logger(this->get_name()),
                  "Successfully set /pc_wind_curr_command");
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger(this->get_name()),
                  "Failed to set /pc_wind_curr_command: received error code [[ %s ]]",
                  PBInterface::pbsrv_enum2str[future.get()->result.value].c_str());
      //TODO: should we shutdown?
    }
  };

  auto response = pc_wind_curr_client_->async_send_request(request, wind_curr_response_callback);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PBTorqueController>("pb_torque_controller"));
  rclcpp::shutdown();

  return 0;
}
