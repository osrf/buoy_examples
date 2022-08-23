#!/usr/bin/python3

# Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from buoy_api import Interface
from buoy_interfaces.srv import PCBiasCurrCommand
import rclpy
from scipy import interpolate


class NLBiasDampingPolicy(object):

    def __init__(self):
        self.breaks = [0.0, 2.03]  # TODO(andermi) find suitable defaults (or leave this)
        self.bias = [0.0, 0.0]  # TODO(andermi) find suitable defaults (or leave this)
        self.deadzone = [0.75, 1.25]
        self.bias_interp1d = None
        self.update_params()

    def update_params(self):
        if len(self.breaks) > 0 and len(self.bias) > 0:
            self.bias_interp1d = interpolate.interp1d(self.breaks, self.bias,
                                                      fill_value=(0.0, self.bias[-1]),
                                                      bounds_error=False)
        else:
            self.bias_interp1d = None

    def bias_current_target(self, piston_pos):
        if self.deadzone[0] < piston_pos < self.deadzone[1]:
            return None
        if self.bias_interp1d is not None:
            return self.bias_interp1d(piston_pos)
        else:
            return None


class NonLinearBiasDamping(Interface):

    def __init__(self):
        super().__init__('pb_nl_bias_damping')
        self.policy = NLBiasDampingPolicy()
        self.set_params()
        self.set_sc_pack_rate_param(50.0)

    def set_params(self):
        self.declare_parameter('bias_damping.position_breaks', self.policy.breaks)
        self.declare_parameter('bias_damping.bias', self.policy.bias)
        self.declare_parameter('bias_damping.position_deadzone', self.policy.deadzone)
        position_break_params = self.get_parameters_by_prefix('bias_damping')

        self.policy.breaks = \
            position_break_params['position_breaks'].get_parameter_value().double_array_value
        self.policy.bias = \
            position_break_params['bias'].get_parameter_value().double_array_value

        self.policy.update_params()

    def spring_callback(self, data):
        bct = self.policy.bias_current_target(data.range_finder)
        self.get_logger().info(f'Bias Damping: f({data.range_finder}) = {bct}')
        if bct is None:
            return

        request = PCBiasCurrCommand.Request()
        request.bias_curr = float(bct)

        self.pc_bias_curr_future_ = self.pc_bias_curr_client_.call_async(request)
        self.pc_bias_curr_future_.add_done_callback(self.default_service_response_callback)


if __name__ == '__main__':
    rclpy.init()

    controller = NonLinearBiasDamping()

    rclpy.spin(controller)

    rclpy.shutdown()
