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

from buoy_msgs.interface import Interface
from buoy_msgs.srv import PCWindCurrCommand
import numpy as np
import rclpy
from scipy import interpolate


class PBTorqueControlPolicy(object):

    def __init__(self):
        self.Torque_constant = 0.438  # N-m/Amps
        self.N_Spec = np.array([0.0, 300.0, 600.0, 1000.0, 1700.0, 4400.0, 6790.0])  # RPM
        # Matches old boost converter targets that have been deployed.
        self.Torque_Spec = np.array([0.0, 0.0, 0.8, 2.9, 5.6, 9.8, 16.6])  # N-m

        self.update_params()

    def update_params(self):
        self.I_Spec = self.Torque_Spec/self.Torque_constant  # Amps
        self.windcurr_interp1d = interpolate.interp1d(self.N_Spec, self.I_Spec)

    def winding_current_target(self, rpm, scale_factor, retract_factor):
        N = abs(rpm)
        if N >= self.N_Spec[-1]:
            I = self.I_Spec[-1]  # noqa: E741
        else:
            I = self.windcurr_interp1d(N)  # noqa: E741

        I *= scale_factor  # noqa: E741
        if rpm > 0.0:
            I *= -retract_factor  # noqa: E741

        return float(I)

    def __str__(self):
        return """PBTorqueControlPolicy:
\tTorque_constant: {tc}
\tN_Spec: {nspec}
\tTorque_Spec: {tspec}
\tI_Spec: {ispec}""".format(tc=self.Torque_constant,
                            nspec=self.N_Spec,
                            tspec=self.Torque_Spec,
                            ispec=self.I_Spec)


class PBTorqueController(Interface):

    def __init__(self):
        super().__init__('pb_torque_control')

        self.policy = PBTorqueControlPolicy()
        self.set_params()

        self.set_pc_pack_rate()

    def power_callback(self, data):
        request = PCWindCurrCommand.Request()
        request.wind_curr = self.policy.winding_current_target(data.rpm, data.scale, data.retract)

        self.get_logger().info(f'WindingCurrent: f({data.rpm}, {data.scale}, {data.retract}) = ' +
                               f'{request.wind_curr}')

        self.pc_wind_curr_future_ = self.pc_wind_curr_client_.call_async(request)
        self.pc_wind_curr_future_.add_done_callback(self.service_response_callback)

    def set_params(self):
        self.declare_parameter('torque_constant', self.policy.Torque_constant)
        self.policy.Torque_constant = \
            self.get_parameter('torque_constant').get_parameter_value().double_value

        self.declare_parameter('n_spec', self.policy.N_Spec.tolist())
        self.policy.N_Spec = \
            np.array(self.get_parameter('n_spec').get_parameter_value().double_array_value)

        self.declare_parameter('torque_spec', self.policy.Torque_Spec.tolist())
        self.policy.Torque_Spec = \
            np.array(self.get_parameter('torque_spec').get_parameter_value().double_array_value)

        self.policy.update_params()
        self.get_logger().info(str(self.policy))


if __name__ == '__main__':
    rclpy.init()

    controller = PBTorqueController()

    rclpy.spin(controller)

    rclpy.shutdown()
