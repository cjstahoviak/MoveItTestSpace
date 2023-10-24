cmake_minimum_required(VERSION 3.8)
project(move_arm_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interbotix_moveit_interface REQUIRED)
find_package(interbotix_moveit_interface_msgs REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(pos_control_moveit_interface src/pos_control_moveit_interface.cpp)
target_include_directories(pos_control_moveit_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pos_control_moveit_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pos_control_moveit_interface
  "moveit_ros_planning_interface"
  "rclcpp"
)
install(TARGETS pos_control_moveit_interface
  DESTINATION lib/${PROJECT_NAME}
) 
 

### Build pos_control_moveit_api.cpp
add_executable(pos_control_moveit_api src/pos_control_moveit_api.cpp)
target_include_directories(pos_control_moveit_api PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  #$<BUILD_INTERFACE:/home/calvinjs/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/include>
)
ament_target_dependencies( pos_control_moveit_api  
  moveit_ros_planning_interface
  rclcpp
  interbotix_moveit_interface
  interbotix_moveit_interface_msgs
  moveit_visual_tools
)
target_link_libraries( pos_control_moveit_api
  ${ament_index_cpp_LIBRARIES}
  #${interbotix_moveit_interface_LIBRARIES}
)
# target_link_libraries(pos_control_moveit_api interbotix_moveit_interface::interbotix_moveit_interface)
install(TARGETS pos_control_moveit_api
  DESTINATION lib/${PROJECT_NAME}
) 


### Build pos_control_joint_cmd.cpp
add_executable(pos_control_joint_cmd src/pos_control_joint_cmd.cpp)
target_include_directories(pos_control_joint_cmd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pos_control_joint_cmd PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies( pos_control_joint_cmd
  "moveit_ros_planning_interface"
  "rclcpp"
  "interbotix_xs_msgs"
)
install(
  TARGETS pos_control_joint_cmd
  DESTINATION lib/${PROJECT_NAME}
) 



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
