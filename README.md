# RMoveIt Testing
ROS2 Humble workspace for testing different ways of interacting with the Trossen Robotics PX100. I took three different approaches detailed below in the "Files" section

## Files
##### pos_control_moveit_interface.cpp: 
As far as I understand, I should be able to launch the "interbotic_xsarm_moveit" launch file and then in this node, create and use the "MoveGroupInterface" object in a similar way to this tutorial "https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html"

##### pos_control_moveit_api.cpp: 
Attempt to use the API to control the MoveGroup node indirectly. Last time I was pointed to this example of the interbotix_moveit_api being used "https://github.com/Interbotix/interbotix_ros_toolboxes/blob/humble/interbotix_common_toolbox/interbotix_moveit_interface/src/moveit_interface.cpp" but I can't get this code to build in my package.

##### pos_control_joint_cmd.cpp: 
I attempt to use "interbotix_xs_msgs::msg::JointGroupCommand" to send joint commands. 

## Build
Compile with: 
```bash
colcon build
```

## Run

### 1) To (attempt to) use interbotix moveit interface:
```bash
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=wx250 hardware_type:=fake
ros2 run move_arm_test pos_control_moveit_interface
```
The error with this is that my node gives the error after running:
```bash
Could not find parameter robot_description and did not receive robot_description via std_msgs::msg::String subscription within 10.000000 seconds.
```

### 2) To (attempt to) use interbotix moveit api:
```bash
ros2 launch interbotix_xsarm_moveit_interface xsarm_moveit_interface.launch.py robot_model:=px100
ros2 run move_arm_test pos_control_moveit_api
```
The first error with this is that my node gives the compilaion error:
```bash
Starting >>> move_arm_test
--- stderr: move_arm_test                               
/home/calvinjs/ws_moveit_test/src/move_arm_test/src/pos_control_moveit_api.cpp:29:10: fatal error: interbotix_moveit_interface/moveit_interface_obj.hpp: No such file or directory
   29 | #include "interbotix_moveit_interface/moveit_interface_obj.hpp"
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/pos_control_moveit_api.dir/build.make:76: CMakeFiles/pos_control_moveit_api.dir/src/pos_control_moveit_api.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:167: CMakeFiles/pos_control_moveit_api.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< move_arm_test [10.1s, exited with code 2]
```
The second error, when giving a direct path the the interface api header, is an undefined reference:
```bash
Starting >>> move_arm_test
--- stderr: move_arm_test                                
/usr/bin/ld: CMakeFiles/pos_control_moveit_api.dir/src/pos_control_moveit_api.cpp.o: in function `void __gnu_cxx::new_allocator<interbotix::InterbotixMoveItInterface>::construct<interbotix::InterbotixMoveItInterface, std::shared_ptr<rclcpp::Node>&>(interbotix::InterbotixMoveItInterface*, std::shared_ptr<rclcpp::Node>&)':
pos_control_moveit_api.cpp:(.text._ZN9__gnu_cxx13new_allocatorIN10interbotix25InterbotixMoveItInterfaceEE9constructIS2_JRSt10shared_ptrIN6rclcpp4NodeEEEEEvPT_DpOT0_[_ZN9__gnu_cxx13new_allocatorIN10interbotix25InterbotixMoveItInterfaceEE9constructIS2_JRSt10shared_ptrIN6rclcpp4NodeEEEEEvPT_DpOT0_]+0x47): undefined reference to `interbotix::InterbotixMoveItInterface::InterbotixMoveItInterface(std::shared_ptr<rclcpp::Node>&)'
/usr/bin/ld: CMakeFiles/pos_control_moveit_api.dir/src/pos_control_moveit_api.cpp.o: in function `void __gnu_cxx::new_allocator<interbotix::InterbotixMoveItInterface>::destroy<interbotix::InterbotixMoveItInterface>(interbotix::InterbotixMoveItInterface*)':
pos_control_moveit_api.cpp:(.text._ZN9__gnu_cxx13new_allocatorIN10interbotix25InterbotixMoveItInterfaceEE7destroyIS2_EEvPT_[_ZN9__gnu_cxx13new_allocatorIN10interbotix25InterbotixMoveItInterfaceEE7destroyIS2_EEvPT_]+0x1c): undefined reference to `interbotix::InterbotixMoveItInterface::~InterbotixMoveItInterface()'
collect2: error: ld returned 1 exit status
gmake[2]: *** [CMakeFiles/pos_control_moveit_api.dir/build.make:429: pos_control_moveit_api] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:167: CMakeFiles/pos_control_moveit_api.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< move_arm_test [26.2s, exited with code 2]
```

### 3) To (attempt to) use command the px100 via interbotix_xs_sdk node
```bash
ros2 launch interbotix_xsarm_ros_control xsarm_ros_control.launch.py robot_model:=px100
ros2 run move_arm_test pos_control_joint_cmd
```
The problem with this is that my node does not successfuly send a messaage. An error is never called, however, a JointGroupCommand msg never appears on the topic "/px100/commands/joint_group"
