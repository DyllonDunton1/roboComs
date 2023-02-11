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
