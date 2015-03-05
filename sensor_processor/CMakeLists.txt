cmake_minimum_required(VERSION 2.8.3)
project(sensor_processor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  state_manager
  )

catkin_package(
  CATKIN_DEPENDS
    roscpp
    state_manager
  INCLUDE_DIRS
    include
  )

include_directories(include ${catkin_INCLUDE_DIRS})

# add libraries, dependencies and links to them

# add executables, dependencies and links to them

# add roslint

# add test cmake
