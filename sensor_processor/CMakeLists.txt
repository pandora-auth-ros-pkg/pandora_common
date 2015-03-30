cmake_minimum_required(VERSION 2.8.3)
project(sensor_processor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  state_manager
  state_manager_msgs
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
    state_manager
    state_manager_msgs
  INCLUDE_DIRS
    include
  )

include_directories(include ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  find_package(pandora_testing_tools REQUIRED)

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
    ${catkin_LIBRARIES}
    )
  add_library(dummy_handler src/dummy_handler.cpp)
  add_dependencies(dummy_handler
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
  target_link_libraries(dummy_handler
    ${catkin_LIBRARIES}
    dummy_preprocessor
    dummy_processor
    dummy_postprocessor
    )

  add_executable(dummy_processor_node src/dummy_processor_node.cpp)
  target_link_libraries(dummy_processor_node
    ${catkin_LIBRARIES}
    dummy_handler
    )

  find_package(rostest REQUIRED)
  find_package(pandora_testing_tools REQUIRED)

  add_rostest(launch/sensor_processor_node.test)

endif()

