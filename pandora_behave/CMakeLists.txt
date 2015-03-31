cmake_minimum_required(VERSION 2.8.3)
project(pandora_behave)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  std_srvs
  )

catkin_python_setup()

add_message_files(
  FILES
    MockCmd.msg
    MockResponse.msg
  )

add_service_files(
  FILES
    MockTask.srv
  )

generate_messages(
  DEPENDENCIES
    std_msgs
    std_srvs
  )

catkin_package(
  DEPENDS
  CATKIN_DEPENDS
    rospy
    std_msgs
  LIBRARIES
  INCLUDE_DIRS
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )
