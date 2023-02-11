# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

#import maestro as m

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        #self.servo = m.Controller()
        #self.servo.setAccel(0,0)
        #self.servo.setAccel(1,0)
        #self.servo.setSpeed(0,0)
        #self.servo.setSpeed(1,0)
        self.base = 6000
        self.max = 1000

    def listener_callback(self, msg):
        bools = msg.data.split(" ")
    	
        
        speed_left = 0
        speed_right = 0
    	
        if (bools[0] == 'True'):
            speed_left += .50
            speed_right += .50
        if (bools[1] == 'True'):
            speed_left -= .50
            speed_right -= .50
        if (bools[2] == 'True'):
            speed_left -= .40
            speed_right += .40
        if (bools[3] == 'True'):
            speed_left += .40
            speed_right -= .40            
        
        pwm_left = (speed_left * self.max) + self.base
        pwm_right = (speed_right * self.max) + self.base
        
        pwms = str(pwm_left) + "  |  " + str(pwm_right)
        
        #self.servo.setTarget(0,pwm_left)
        #self.servo.setTarget(0,pwm_left)
    	
        self.get_logger().info(pwms)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    
    

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
