#ifndef TORQUE_CONTROLLER_HPP_
#define TORQUE_CONTROLLER_HPP_

#include "buoy_examples/controller.hpp"

// forward declare
struct PBTorqueControlPolicy;  //defined by user in torque_control_policy.hpp

class PBTorqueController : public PBInterface::PBController<PBTorqueController>
{
public:
  //using PBInterface::PBController<PBTorqueController>::PBController;
  PBTorqueController(const std::string &node_name);
  virtual ~PBTorqueController() = default;

protected:
  virtual void power_callback(const buoy_msgs::msg::PCRecord &data) override final;

private:
  friend CRTP;
  virtual void set_params() override final;  //defined by user in torque_control_policy.hpp

  std::unique_ptr<PBTorqueControlPolicy> policy_;

};

#endif //TORQUE_CONTROLLER_HPP_
