# blackbox: records the tracker's logs, inputs and tracking error to MCAP for
# Foxglove (run blackbox-record or volasim-record alongside). Declared here
# rather than in the top-level CMakeLists because the tracker is meant to move
# out of volasim. Uses a sibling checkout (../blackbox) when present, else the
# git repo; override with -DFETCHCONTENT_SOURCE_DIR_BLACKBOX=<path>.
include(FetchContent)
if(EXISTS ${CMAKE_SOURCE_DIR}/../blackbox/CMakeLists.txt)
  FetchContent_Declare(blackbox SOURCE_DIR ${CMAKE_SOURCE_DIR}/../blackbox)
else()
  FetchContent_Declare(
    blackbox
    GIT_REPOSITORY git@github.com:nocholasrift/blackbox.git
    GIT_TAG        main
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
