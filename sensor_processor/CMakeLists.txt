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

add_library(abstract_processor src/abstract_processor.cpp)
target_link_libraries(abstract_processor
  ${catkin_LIBRARIES}
  )
  
add_library(vision_preprocessor src/vision_preprocessor.cpp)
target_link_libraries(vision_preprocessor
  ${catkin_LIBRARIES}
  )
  
add_library(vision_postprocessor src/vision_postprocessor.cpp)
target_link_libraries(vision_postprocessor
  ${catkin_LIBRARIES}
  )

# add executables, dependencies and links to them

# add roslint

# add test cmake
