# Declared here, not in the top-level CMakeLists, because the tracker is moving out of volasim.
include(FetchContent)
if(EXISTS ${CMAKE_SOURCE_DIR}/../blackbox/CMakeLists.txt)
  FetchContent_Declare(blackbox SOURCE_DIR ${CMAKE_SOURCE_DIR}/../blackbox)
else()
  FetchContent_Declare(
    blackbox
    GIT_REPOSITORY https://github.com/nocholasrift/blackbox.git
    GIT_TAG        fe337059cdcd47381718e663041383e59f0c77a2
  )
endif()
FetchContent_MakeAvailable(blackbox)

add_executable(lee_control_zmq
  src/lee_control_zmq.cpp
)
target_include_directories(lee_control_zmq PRIVATE
  include
  ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(lee_control_zmq
  PRIVATE
  vola_controller
  cppzmq-static
  volasim_msgs
  blackbox::blackbox
)
target_compile_features(lee_control_zmq PRIVATE cxx_std_17)

add_executable(send_position
  src/send_position.cpp
)
target_include_directories(send_position PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(send_position
  PRIVATE
  cppzmq-static
  volasim_msgs
)
target_compile_features(send_position PRIVATE cxx_std_17)
