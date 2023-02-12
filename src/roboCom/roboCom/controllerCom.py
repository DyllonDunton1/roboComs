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

import pygame



arrowWidth = 50
width = 1500
height = 900


"""
CONTROLS:

Forward -> up arrow
Backward -> down arrow
Left -> Left arrow
Right -> Right arrow

Boom Down -> Q
Boom Up -> W
Boom Dump -> E
Auger Spin -> A
Auger Lower -> Z
Auger Raise -> C
"""


class Pane(pygame.sprite.Sprite):
    
    def __init__(self, text, xPos, yPos, typeDisp):
        super(Pane, self).__init__()
        self.text = text
        self.font = pygame.font.SysFont('Arial', 16)
        self.textsurf = self.font.render(self.text, True, (0,0,0))
        self.height = arrowWidth
        self.width = arrowWidth
        self.red = 255
        self.green = 0
        
        if (typeDisp == "Action"):
            self.width = arrowWidth*2
        self.image = pygame.Surface((self.width, self.height))
        self.image.fill((self.red, self.green, 0))
        self.xpos = xPos
        self.ypos = yPos
        
        self.textx = (self.image.get_width() - self.textsurf.get_width())/2
        self.texty = (self.image.get_height() - self.textsurf.get_height())/2
        
        self.image.blit(self.textsurf, (self.textx,self.texty))
        screen.blit(self.image, (self.xpos,self.ypos))
        
    def setFill(self, active):
        if (active):
            self.red = 0
            self.green = 255
        else:
            self.red = 255
            self.green = 0
    
        self.image = pygame.Surface((self.width, self.height))
        self.image.fill((self.red,self.green,0))    #red   
        
        self.textx = (self.image.get_width() - self.textsurf.get_width())/2
        self.texty = (self.image.get_height() - self.textsurf.get_height())/2
        
        self.image.blit(self.textsurf, (self.textx,self.texty))
        screen.blit(self.image, (self.xpos,self.ypos))      
            
    def setFillRG(self, r,g): 
        self.red = r
        self.green = g
        self.image = pygame.Surface((self.width, self.height))
        self.image.fill((self.red,self.green,0))  #new color
        
        self.textx = (self.image.get_width() - self.textsurf.get_width())/2
        self.texty = (self.image.get_height() - self.textsurf.get_height())/2
        
        self.image.blit(self.textsurf, (self.textx,self.texty))
        screen.blit(self.image, (self.xpos,self.ypos))
        
    def setText(self, newText):
        self.text = newText
        self.textsurf = self.font.render(newText, True, (0,0,0))
        self.image = pygame.Surface((self.width, self.height))
        self.image.fill((self.red,self.green,0))
        
        self.textx = (self.image.get_width() - self.textsurf.get_width())/2
        self.texty = (self.image.get_height() - self.textsurf.get_height())/2
        
        self.image.blit(self.textsurf, (self.textx,self.texty))
        screen.blit(self.image, (self.xpos,self.ypos))


class ControllerPubSub(Node):

    def __init__(self):
        super().__init__('controller_pubsub')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.subscription = self.create_subscription(String,'topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        
        self.i = 0
        self.up = False;
        self.down = False;
        self.left = False;
        self.right = False; 
        self.BoomDown = False;
        self.BoomUp = False;
        self.BoomDump = False;
        self.AugerSpin = False;
        self.AugerLower = False;
        self.AugerRaise = False;
        
        self.roboBoomPos = 10
        self.roboAugerSpin = False
        self.roboAaugerHeight = 100
        self.roboDoorLatch = True
        self.roboWheelsLeft = False
        self.roboWheelsRight = False
        
        self.augerTarget = "up"
        self.boomTarget = 10
        self.boomStart = 10
        
        self.boomMessage = "Boom Down"
        self.augerMessage = "Auger Up"
        
        
    def listener_callback(self, msg):
        tf = msg.data
        tf_split = tf.split('*')
        if (tf_split[0] == 'tfData'):
        
            roboData_split = tf_split[1].split(' ')
            self.roboBoomPos = roboData_split[0]
            self.roboAugerSpin = roboData_split[1]
            self.roboAugerHeight = roboData_split[2]
            self.roboDoorLatch = roboData_split[3]
            self.roboWheelsLeft = roboData_split[4]
            self.roboWheelsRight = roboData_split[5]
            
            #set UI based on Data
            
            #augerSpin
            if (self.roboAugerSpin == 'True'):
                augerSpin.setFill(True)
            else:
                augerSpin.setFill(False)
            
            #augerHeight
            max_height = 100
            height = int(self.roboAugerHeight)
            green = int((height*255)/max_height)
            red = 255 - green
            if (self.augerTarget == "down"):
                red = green
                green = 255 - green
            augerHeight.setFillRG(red,green)
            
            
            #augerBoom
            distance_away = self.boomTarget - int(self.roboBoomPos)
            distance_toTravel = self.boomTarget - self.boomStart
            if (distance_toTravel == 0):
            	distance_toTravel = 1
            red = int((distance_away/distance_toTravel)*255)
            green = 255 - red
            boomPos.setFillRG(red, green)
            

            #doorLatch
            if (self.roboDoorLatch == 'True'):
                doorLatch.setFill(True)
            else:
                doorLatch.setFill(False)
            
            
            self.get_logger().info(tf_split[1])  
            
            #wheels
            if (self.roboWheelsLeft == 'True'):
                wheelsLeft.setFill(True)
            else:
                wheelsLeft.setFill(False)
                
            if (self.roboWheelsRight == 'True'):
                wheelsRight.setFill(True)
            else:
                wheelsRight.setFill(False)    
         

    def timer_callback(self):
        temp_width = 0
        temp_heigth = 0 
        for event in pygame.event.get(): #check for quit out
            if event.type == pygame.QUIT:
                running = False
                quit()
            if event.type == pygame.KEYDOWN: #if a key has been pressed, turn that key on
                keys = pygame.key.get_pressed()
                if keys[pygame.K_LEFT]:
                    self.left = True;
                    leftKey.setFill(True)
                if keys[pygame.K_RIGHT]:
                    self.right = True;
                    rightKey.setFill(True)
                if keys[pygame.K_UP]:
                    self.up = True;
                    upKey.setFill(True)
                if keys[pygame.K_DOWN]:
                    self.down = True;
                    downKey.setFill(True)
                if keys[pygame.K_q]:
                    self.BoomDown = True; 
                    self.boomStart = int(self.roboBoomPos)
                    self.boomTarget = 10
                    self.boomMessage = "Boom Down"
                    boomPos.setText(self.boomMessage)
                if keys[pygame.K_w]:
                    self.BoomUp = True;
                    self.boomStart = int(self.roboBoomPos)
                    self.boomTarget = 90
                    self.boomMessage = "Boom Up"
                    boomPos.setText(self.boomMessage)
                if keys[pygame.K_e]:
                    self.BoomDump = True;
                    self.boomStart = int(self.roboBoomPos)
                    self.boomTarget = 135
                    self.boomMessage = "Boom Dump"
                    boomPos.setText(self.boomMessage)
                if keys[pygame.K_a]:
                    self.AugerSpin = True;
                if keys[pygame.K_z]:
                    self.AugerLower = True;
                    self.augerTarget = "down"
                    self.augerMessage = "Auger Down"
                    augerHeight.setText(self.augerMessage)
                if keys[pygame.K_c]:
                    self.AugerRaise = True;   
                    self.augerTarget = "up"  
                    self.augerMessage = "Auger Up"    
                    augerHeight.setText(self.augerMessage)              
            if event.type == pygame.KEYUP: #if a key has been UNpressed, turn that key off
                keys = pygame.key.get_pressed()
                if not keys[pygame.K_LEFT]:
                    self.left = False;
                    leftKey.setFill(False)
                if not keys[pygame.K_RIGHT]:
                    self.right = False;
                    rightKey.setFill(False)
                if not keys[pygame.K_UP]:
                    self.up = False;
                    upKey.setFill(False)
                if not keys[pygame.K_DOWN]:
                    self.down = False;     
                    downKey.setFill(False)  
                if not keys[pygame.K_q]:
                    self.BoomDown = False; 
                if not keys[pygame.K_w]:
                    self.BoomUp = False;
                if not keys[pygame.K_e]:
                    self.BoomDump = False;
                if not keys[pygame.K_a]:
                    self.AugerSpin = False;
                if not keys[pygame.K_z]:
                    self.AugerLower = False;
                if not keys[pygame.K_c]:
                    self.AugerRaise = False;      
    	
        updateSprites()
        pygame.display.flip()
    	
    	#Build com string based on keyboard press data
        msgType = "keyData*"
        arrowCom = str(self.up) + " " + str(self.down) + " " + str(self.left) + " " + str(self.right)
        actionCom = str(self.BoomDown) + " " + str(self.BoomUp) + " " + str(self.BoomDump) + " " + str(self.AugerSpin) + " " + str(self.AugerLower) + " " + str(self.AugerRaise)
        com = msgType + arrowCom + "," + actionCom
        msg = String()
        msg.data = com
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
            
    
def updateSprites():
    #Keys
    rightKey.update()
    leftKey.update()
    upKey.update()
    downKey.update()
    
    #Remote side actions
    augerSpin.update()
    augerHeight.update()
    boomPos.update()
    wheelsLeft.update()
    wheelsRight.update()
    doorLatch.update()

def initUIObjects():
    
    global rightKey
    global leftKey
    global upKey
    global downKey
    
    global augerSpin
    global augerHeight
    global boomPos
    global wheelsLeft
    global wheelsRight
    global doorLatch
    
    arrowSeperation = arrowWidth/3
    spacing = arrowWidth + arrowSeperation
    centerx = width - 2*(spacing)
    centery = height - (spacing)
    
    rightx = centerx + spacing
    righty = centery
    leftx = centerx - spacing
    lefty = centery
    upx = centerx
    upy = centery - spacing
    downx = centerx
    downy = centery
    
    augerSpinx = arrowSeperation
    augerSpiny = height - 1*(spacing)
    augerHeightx = arrowSeperation
    augerHeighty = height - 2*(spacing)
    boomPosx = arrowSeperation
    boomPosy = height - 3*(spacing)
    wheelsLeftx = arrowSeperation
    wheelsLefty = height - 4*(spacing)
    wheelsRightx = arrowSeperation
    wheelsRighty = height - 5*(spacing)
    doorLatchx = arrowSeperation
    doorLatchy = height - 6*(spacing)
    
    #arrows
    rightKey = Pane('R',rightx,righty,'Arrow')
    leftKey = Pane('L',leftx,lefty,'Arrow')
    upKey = Pane('FW',upx,upy,'Arrow')
    downKey = Pane('BW',downx,downy,'Arrow')
    
    #Remote side actions
    augerSpin = Pane('Auger Spin', augerSpinx, augerSpiny, 'Action')
    augerHeight = Pane('Auger Up', augerHeightx, augerHeighty, 'Action')
    boomPos = Pane('Boom Down', boomPosx, boomPosy, 'Action')
    wheelsLeft = Pane('Left Wheels', wheelsLeftx, wheelsLefty, 'Action')
    wheelsRight = Pane('Right Wheels', wheelsRightx, wheelsRighty, 'Action')
    doorLatch = Pane('Door Latch', doorLatchx, doorLatchy, 'Action')
    
    
    
    
def main(args=None):
    rclpy.init(args=args)

    pygame.init()
    global screen
    screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
    screen.fill((0,0,0))
    
    initUIObjects()
    
    controller_pubsub = ControllerPubSub()
    try:
        rclpy.spin(controller_pubsub)
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        controller_pubsub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
       
    
