# roboComs
2023 Lunabotics communications


cd /home/dunto/roboComs && . install/setup.bash && ros2 run roboCom controller

cd /home/dunto/roboComs && . install/setup.bash && ros2 run roboCom robot





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



# roboComs  
2023 Lunabotics communications  

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

General Controls:  

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

