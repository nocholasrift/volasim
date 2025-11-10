# determine ros version
if(NOT DEFINED ROS_VERSION)
  find_package(catkin QUIET)
  if(${catkin_FOUND})
    set(ROS_VERSION 1)
  else()
    find_package(rclcpp QUIET)
    if(${rclcpp_FOUND})
      set(ROS_VERSION 2)
    endif()
  endif()
endif()

include_directories(
  include/
  ${CMAKE_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${cppzmq_INCLUDE_DIRS}
)

add_library(lee_controller
  src/matrix_utils.cpp
  src/lee_controller.cpp
)
target_link_libraries(lee_controller
  PRIVATE 
  ${EIGEN3_LIBRARIES}
)

if(ROS_VERSION EQUAL 1)
  message(STATUS "ROS1 installation detected")
  find_package(catkin COMPONENTS
    roscpp
    nav_msgs
    std_msgs
    geometry_msgs
    trajectory_msgs
  )

  include_directories(${catkin_INCLUDE_DIRS})

  add_executable(vola_ros_bridge
    src/vola_ros1_bridge.cpp
  )
  target_link_libraries(vola_ros_bridge
    PRIVATE 
    ${catkin_LIBRARIES}
    cppzmq
    myproto
  )

  add_executable(lee_control_node
    src/lee_control_ros1.cpp
  )

  target_link_libraries(lee_control_node
    PRIVATE
    ${catkin_LIBRARIES}
    lee_controller
  )
  
elseif(ROS_VERSION EQUAL 2)
  message(STATUS "ROS2 installation detected")

  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  find_package(trajectory_msgs REQUIRED)

  include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${rcl_interfaces_INCLUDE_DIRS}
    ${trajectory_msgs_INCLUDE_DIRS}
  )

  add_executable(vola_ros_bridge
    src/vola_ros2_bridge.cpp
  )
  ament_target_dependencies(vola_ros_bridge 
    rclcpp
    std_msgs
    std_srvs
    nav_msgs
    trajectory_msgs
  )
  target_link_libraries(vola_ros_bridge
    cppzmq
    myproto
  )
  install(TARGETS vola_ros_bridge
    DESTINATION ${PROJECT_SOURCE_DIR}/build
  )

  add_executable(lee_control_node
    src/lee_control_ros2.cpp
  )
  ament_target_dependencies(lee_control_node
    rclcpp
    std_msgs
    nav_msgs
    geometry_msgs
    trajectory_msgs
  )
  target_link_libraries(lee_control_node
    lee_controller
  )
  install(TARGETS lee_control_node
    DESTINATION ${PROJECT_SOURCE_DIR}/build
  )

else()
  message(WARNING "No ROS installation found, only building simulator exectuables")
  message(WARNING "If ROS is installed on this machine, check that ROS_VERSION is defined")
endif()

