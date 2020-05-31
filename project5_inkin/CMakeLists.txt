cmake_minimum_required(VERSION 3.0.2)
project(project5_inkin)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  visualization_msgs
)

add_service_files(
   FILES
   jint_control.srv
   oint_control.srv
   jint_clear.srv
   oint_clear.srv
   ocmd_control.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
