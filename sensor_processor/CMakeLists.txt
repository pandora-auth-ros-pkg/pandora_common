cmake_minimum_required(VERSION 2.8.3)
project(sensor_processor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  cv_bridge
  urdf
  sensor_msgs
  state_manager
  state_manager_msgs
  pandora_common_msgs
  pandora_vision_msgs
  roslint
  )

add_message_files(
  FILES ProcessorLogInfo.msg
  )

generate_messages(
  DEPENDENCIES std_msgs
  )

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    cv_bridge
    urdf
    sensor_msgs
    state_manager
    state_manager_msgs
    pandora_common_msgs
    pandora_vision_msgs
  INCLUDE_DIRS
    include
  )

include_directories(include ${catkin_INCLUDE_DIRS})

# add libraries, dependencies and links to them
  
# add_library(vision_preprocessor src/vision_preprocessor.cpp)
# target_link_libraries(vision_preprocessor
  # ${catkin_LIBRARIES}
  # )
  
# add_library(qr_postprocessor src/qr_postprocessor.cpp)
# target_link_libraries(qr_postprocessor
  # ${catkin_LIBRARIES}
  # )
  
# add_library(vision_handler src/vision_handler.cpp)
# target_link_libraries(vision_handler

# add_library(vision_preprocessor src/vision_preprocessor.cpp)
# target_link_libraries(vision_preprocessor
  # ${catkin_LIBRARIES}
  # )

#~ add_library(vision_postprocessor src/vision_postprocessor.cpp)
#~ target_link_libraries(vision_postprocessor
  #~ ${catkin_LIBRARIES}
  #~ )

# add_library(vision_handler src/vision_handler.cpp)
# target_link_libraries(vision_handler
  # ${catkin_LIBRARIES}
  # vision_preprocessor
  # # vision_postprocessor
  # )

add_library(dummy_preprocessor src/dummy_preprocessor.cpp)
target_link_libraries(dummy_preprocessor
  ${catkin_LIBRARIES}
  )
add_library(dummy_processor src/dummy_processor.cpp)
target_link_libraries(dummy_processor
  ${catkin_LIBRARIES}
  )
add_library(dummy_postprocessor src/dummy_postprocessor.cpp)
target_link_libraries(dummy_postprocessor
>>>>>>> Fixes sensor_processor. Adds processor_log.
  ${catkin_LIBRARIES}
  )
add_library(dummy_handler src/dummy_handler.cpp)
target_link_libraries(dummy_handler
  ${catkin_LIBRARIES}
  dummy_preprocessor
  dummy_processor
  dummy_postprocessor
  )

# add executables, dependencies and links to them
add_executable(yolor src/yolor.cpp)
target_link_libraries(yolor
  ${catkin_LIBRARIES}
  dummy_handler
  )

# add roslint

# add test cmake
