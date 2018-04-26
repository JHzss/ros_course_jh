cmake_minimum_required(VERSION 2.8.3)
project(rbx1_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    std_msgs
    tf
    nav_msgs
    move_base_msgs
    actionlib
    cv_bridge
)
set(OpenCV_DIR /home/jh/opencv-3.2.0/build)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs nav_msgs move_base_msgs actionlib)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf geometry_msgs nav_msgs move_base_msgs actionlib cv_bridge
)


# the node:  move forward and back
add_executable(timed_out_and_back src/timed_out_and_back.cpp)
target_link_libraries(timed_out_and_back ${catkin_LIBRARIES})
add_dependencies(timed_out_and_back ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# the node: move forward and back with odom
add_executable(odom_out_and_back src/odom_out_and_back.cpp )
target_link_libraries(odom_out_and_back ${catkin_LIBRARIES})
add_dependencies(odom_out_and_back ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# the node: move following a square
add_executable(Move_square src/Move_suqare.cpp)
target_link_libraries(Move_square ${catkin_LIBRARIES})
add_dependencies(Move_square ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# the node: move following a square in a SET speed
add_executable(speed_control src/speed_control.cpp)
target_link_libraries(speed_control ${catkin_LIBRARIES})
add_dependencies(speed_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# the node: move following a PID speed CONCTOL
add_executable(pid_control src/pid_control.cpp)
target_link_libraries(pid_control ${catkin_LIBRARIES})
add_dependencies(pid_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# the node: move following a PID speed CONCTOL
add_executable(target src/target.cpp)
target_link_libraries(target ${catkin_LIBRARIES})
add_dependencies(target ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# the node: move forward and back with odom
add_executable(Keyboard_Control src/Keyboard_Control.cpp)
target_link_libraries(Keyboard_Control ${catkin_LIBRARIES})
add_dependencies(Keyboard_Control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})
add_dependencies(simple_navigation_goals ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sweep src/sweep.cpp)
target_link_libraries(sweep ${catkin_LIBRARIES})
add_dependencies(sweep ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_slam_node src/ros_slam_node.cpp)
target_link_libraries(ros_slam_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(ros_slam_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
