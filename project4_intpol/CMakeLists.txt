cmake_minimum_required(VERSION 3.0.2)
project(project4_intpol)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
   FILES
   jint_control.srv
   oint_control.srv
   jint_clear.srv
   oint_clear.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)




catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
