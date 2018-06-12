# ADLINK ROS 2.0 Training Material  
  
## Developers  
* HaoChih, LIN (haochih.lin@adlinktech.com)  

## License  
Apache 2.0 (Copyright 2017 ADLINK Technology, Inc.)  
  
## Compile (examples)   
$ cd ~/ros2_ws  
$ ament build  
OR (build only)  
$ ament build --only-packages adlink_ros2  

For isolated build  
$ ament build --isolated --build-tests --symlink-install --only adlink_ros2_training
  
## Run (examples) 
* Method-1 (dynamic_bridge)  
$ roscore  
$ ros2 run ros1_bridge dynamic_bridge  
$ ros2 run adlink_ros2_training pose_array_pub  
$ rostopic list   
(now you can echo "/neonj_poses", like: $ rostopic echo /neonj_poses)  
   
* Method-2 (parameter_bridge)  
$ roscore  
$ rosparam set /topics "[{'topic':neonj_poses, 'type':geometry_msgs/PoseArray}]"   
$ ros2 run ros1_bridge parameter_bridge  
$ ros2 run adlink_ros2_training pose_array_pub  
$ rostopic list   
(now you can echo "/neonj_poses", like: $ rostopic echo /neonj_poses)  


