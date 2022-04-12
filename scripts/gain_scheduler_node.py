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

#!/usr/bin/python

import rclpy
from rclpy.node import Node

from threading import Lock

from buoy_msgs.srv import PCScaleCommand

from buoy_examples.damping_selector import get_gain, generate_schedule


class GainScheduler(Node):
    def __init__(self):
        super().__init__('pb_gain_scheduler')

        self.scale_client = self.create_client(PCScaleCommand, '/pc_scale_command')
        while not self.scale_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/pc_scale_command service not available... still waiting')
        self.future = None

        self.lock = Lock()

        self.gain = 1.0
        self.declare_parameter('gain', self.gain)
        gain_p = self.get_parameter('gain')
        self.gain = gain_p.value
        self.declare_paramter('set_gain_timer_mins', 5)  # default 5 mins
        set_gain_timer_sec_p = self.get_parameter('get_gain_timer_mins')
        self.set_gain_t = self.create_timer(60.0*set_gain_timer_sec_p.value, self.set_gain)

        self.declare_paramter('get_gain_timer_mins', 30)  # default 30 mins
        get_gain_timer_sec_p = self.get_parameter('get_gain_timer_mins')
        self.get_gain_t = self.create_timer(60.0*get_gain_timer_sec_p.value, self.get_gain)

    def get_gain(self):
        with self.lock:
            self.gain = get_gain()

    def set_gain(self):
        req = PCScaleCommand.Request()
        with self.lock:
            req.scale = self.gain
        self.future = self.scale_client.call_async(req)


if __name__=='__main__':
    rclpy.init()

    import os
    from pathlib import Path
    script_path = os.path.dirname(os.path.realpath(__file__))
    damping_schedule_nc = Path(os.path.join(script_path, "damping_schedule.nc"))
    if not my_file.exists():
        generate_schedule()

    gs = GainScheduler()

    err2str = {-1:'BAD_SOCK', -2:'BAD_OPTS', -3:'BAD_INPUT'}

    while rclpy.ok():
        rclpy.spin_once(gs)
        if gs.future is not None and gs.future.done():
            try:
                resp = gs.future.result()
            except Exception as err:
                gs.get_logger().error('Service call failed -- Unable to set PC scale: {err}'.format(err=err))
            else:
                if resp.result.value==resp.result.OK:
                    gs.get_logger().info('Successfully set PC scale')
                else:
                    gs.get_logger().error('Unable to set PC scale: {code}'.format(code=err2str(resp.result.value))

    gs.destroy_node()
    rclpy.shutdown()
