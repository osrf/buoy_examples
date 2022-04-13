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

#ifndef BUOY_EXAMPLES__TORQUE_CONTROLLER_HPP_
#define BUOY_EXAMPLES__TORQUE_CONTROLLER_HPP_


#include <string>
#include <memory>

#include "buoy_examples/controller.hpp"


// forward declare
struct PBTorqueControlPolicy;  // defined by user in torque_control_policy.hpp

class PBTorqueController final: public PBInterface::PBController<PBTorqueController>
{
public:
  explicit PBTorqueController(const std::string &node_name);
  ~PBTorqueController() = default;

private:
  friend CRTP;  // syntactic sugar (see https://stackoverflow.com/a/58435857/9686600)

  void set_params() final;  // defined by user in torque_control_policy.hpp
  void power_callback(const buoy_msgs::msg::PCRecord &data);

  std::unique_ptr<PBTorqueControlPolicy> policy_;
};

#endif  // BUOY_EXAMPLES__TORQUE_CONTROLLER_HPP_
