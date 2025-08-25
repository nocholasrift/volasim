# ROS
find_package(catkin COMPONENTS
  roscpp
  nav_msgs
  std_msgs
  trajectory_msgs
)

include_directories(
  include/
  ${CMAKE_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${cppzmq_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(lee_controller
  src/matrix_utils.cpp
  src/lee_controller.cpp
)
target_link_libraries(lee_controller
  PRIVATE 
  ${EIGEN3_LIBRARIES}
)

if(${catkin_FOUND})
  message(STATUS "Found ROS install - Building ROS nodes")

  add_executable(receive_data
    src/receive_data.cpp
  )
  target_link_libraries(receive_data
    PRIVATE 
    ${catkin_LIBRARIES}
    cppzmq
    myproto
  )

  add_executable(lee_control_node
    src/lee_control_ros.cpp
  )

  target_link_libraries(lee_control_node
    PRIVATE
    ${catkin_LIBRARIES}
    lee_controller
  )
endif()
