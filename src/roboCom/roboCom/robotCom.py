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


import serial
import time

"""
Publisher Data Format:

BoomPos -> int (10,90,135) (degrees)
Auger Spin -> bool (True/False)
AugerHeight -> int (0,100)

"""



class RobotPubSub(Node):

    def __init__(self):
        super().__init__('robot_pubsub')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
            
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
            
        self.subscription  # prevent unused variable warning
        self.base = 6000
        self.max = 1000
        
        self.boomPos = 10
        self.augerSpin = False
        self.augerHeight = 100
        self.doorLatch = True
        self.wheelRight = False
        self.wheelLeft = False
        
        self.dataRecieved = ""
        self.dataSent = ""
        
        #self.arduino = serial.Serial('COM3', 9600)
        #time.sleep(2)
                
    def timer_callback(self):
        msgType = "tfData*"
        dataMessage = str(self.boomPos) + " " + str(self.augerSpin) + " " + str(self.augerHeight) + " " + str(self.doorLatch) + " " + str(self.wheelLeft) + " " + str(self.wheelLeft)
        com = msgType + dataMessage
        msg = String()
        msg.data = com
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def listener_callback(self, msg):
        keys = msg.data
        keys_split = keys.split('*')
        if (keys_split[0] == 'keyData'):
            """
            self.dataRecieved = self.arduino.readLine()
            
            dataRecieved_split = self.dataRecieved.split(' ')
            
            self.boomPos = dataRecieved_split[0]
            self.augerSpin = dataRecieved_split[1]
            self.augerHeight = dataRecieved_split[2]
            self.doorLatch = dataRecieved_split[3]
            self.wheelRight = dataRecieved_split[4]
            self.wheelLeft = dataRecieved_split[5]
            
            self.arduino.write(keys_split[1])
    	
            self.get_logger().info(self.dataRecieved)
            """    
            self.get_logger().info(keys_split[1])
        

def main(args=None):
    rclpy.init(args=args)

    robot_pubsub = RobotPubSub()
    
    

    rclpy.spin(robot_pubsub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
