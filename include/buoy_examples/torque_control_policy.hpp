#ifndef TORQUE_CONTROL_POLICY_HPP_
#define TORQUE_CONTROL_POLICY_HPP_

/********************************************************
/ User-space to define control policy and param loading /
********************************************************/

#include <vector>
#include <cmath>

//interp1d for rpm->winding current
#include "JustInterp/JustInterp.hpp"

#include "buoy_examples/torque_controller.hpp"


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

  //TODO: find a way to make this generic...
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


void PBTorqueController::set_params()
{
  this->declare_parameter("torque_constant", policy_->Torque_constant);
  policy_->Torque_constant = this->get_parameter("torque_constant").as_double();

  this->declare_parameter("n_spec", std::vector<double>(policy_->N_Spec.begin(), policy_->N_Spec.end()));
  std::vector<double> temp_double_arr = this->get_parameter("n_spec").as_double_array();
  policy_->N_Spec.assign(temp_double_arr.begin(), temp_double_arr.end());

  this->declare_parameter("torque_spec", std::vector<double>(policy_->Torque_Spec.begin(), policy_->Torque_Spec.end()));
  temp_double_arr = this->get_parameter("torque_spec").as_double_array();
  policy_->Torque_Spec.assign(temp_double_arr.begin(), temp_double_arr.end());

  policy_->update_params();
  RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), *policy_);
}

#endif //TORQUE_CONTROL_POLICY_HPP_
