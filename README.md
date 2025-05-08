# roboComs
2023 Lunabotics communications system for the University of Maine Black Bear Robotics robot. This code includes the controller-robot communications using ROS2 foxy, in addition to the code that talks to arduino controller on the robot which drives the motors. This also includes the camera capture on both the front and back of the robot, in addition to diagnostic information, and displaying it on the controller interface for the user. At the time of coding, the robot frame was not finished by the Mechy team. This is just the Software. 

See a controller demo below, in addition to usage instructions and some more genral information.

# Controller Interface Demo
![robocom-demo](https://github.com/user-attachments/assets/9a207cae-b316-4d0c-af99-07674e16845c)






# ROS2 Foxy Node Setup

publishers:
remote->keys
bot->camera1
bot->camera2
bot->camera3
bot->TFData


subscribers:
bot->keys
remote->cameras
remote->TFData

# 2023 Lunabotics communications Usage 

To run this program, make sure that librealsense is installed on your brain system. The Github repo for Linux can be found here: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages  

Note: Both the robot brain and controller MUST be on the same network to operate.  

To run the controller:  

Source ROS distro:  
````source /opt/ros/foxy/setup.bash````  

Note: Pygame must be installed on controller host system  
```` sudo apt-get install python3-pygame````  

Build package:  
````cd /home/${USER}/Lunabotics2023 && colcon build````  

To launch the controller, run the bash file:
````~/Lunabotics2023/ControllerLaunch.sh```` 

If successful, a GUI will appear  

To run the robot brain:  

Source ROS distro:  
````source /opt/ros/foxy/setup.bash````    

Build package:  
````cd /home/${USER}/Lunabotics2023 && colcon build````  

Finally, run the bash script:  
````~/Lunabotics2023/RobotLaunch.sh````  

If successful, the controller GUI will be populated with camera feed  



# General Controls 

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

