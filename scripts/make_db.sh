#!/bin/sh


cd ~/srvss_ws/src/Image_Generation_VisOdo/Data_Base/tree4
rosrun gazebo_ros spawn_model -file ~/srvss_ws/src/Image_Generation_VisOdo/models/FLEA3/model.sdf -sdf -model flea3
rosrun Image_Generation_VisOdo Image_Generation_VisOdo
#cd ~/Desktop/Sagi
#rosrun imagesub imagesub



