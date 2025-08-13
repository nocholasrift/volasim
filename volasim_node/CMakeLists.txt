# ROS
find_package(catkin COMPONENTS
  roscpp
  nav_msgs
  std_msgs
)

include_directories(
  include/
  ${EIGEN3_INCLUDE_DIRS}
  ${cppzmq_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

if(${catkin_FOUND})
  message(STATUS "Found ROS install - Building ROS nodes")

  add_executable(receive_data
    volasim_node/src/receive_data.cpp
  )
  target_link_libraries(receive_data
    PRIVATE 
    ${catkin_LIBRARIES}
    cppzmq
    myproto
  )
endif()

add_library(lee_controller
  src/matrix_utils.cpp
  src/lee_controller.cpp
)
target_link_libraries(lee_controller
  PRIVATE 
  ${EIGEN3_LIBRARIES}
)
