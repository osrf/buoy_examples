#ifndef TORQUE_CONTROLLER_HPP_
#define TORQUE_CONTROLLER_HPP_

#include "buoy_examples/controller.hpp"

// forward declare
struct PBTorqueControlPolicy;  //defined by user in torque_control_policy.hpp

class PBTorqueController : public PBInterface::PBController<PBTorqueController>
{
public:
  PBTorqueController(const std::string &node_name);
  virtual ~PBTorqueController() = default;

private:
  friend CRTP;  //syntactic sugar (see https://stackoverflow.com/a/58435857/9686600)

  virtual void set_params() override final;  //defined by user in torque_control_policy.hpp
  virtual void power_callback(const buoy_msgs::msg::PCRecord &data) override final;

  std::unique_ptr<PBTorqueControlPolicy> policy_;

};

#endif //TORQUE_CONTROLLER_HPP_
