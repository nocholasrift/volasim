# Root shim so the container reuses volasim_node/CMakeLists.txt unchanged,
# supplying the targets it expects from a parent (myproto, cppzmq, Eigen)
# without pulling in the simulator's deps (SDL/Jolt/assimp/OpenGL).
cmake_minimum_required(VERSION 3.16)
project(volasim_ros2 CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})  # run_ros2_stack.sh path

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})  # node links Eigen PRIVATE; expose it
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED IMPORTED_TARGET libzmq)

# Header-only cppzmq over the system libzmq.
add_library(cppzmq INTERFACE)
target_link_libraries(cppzmq INTERFACE PkgConfig::ZMQ)
target_include_directories(cppzmq INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cppzmq)
set(cppzmq_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cppzmq)

include_directories(include/)

# myproto is defined here, not via the repo's msgs CMakeLists, because that file
# needs CONFIG-mode protobuf which apt does not ship. Generating into the -I root
# keeps the <volasim/comms/msgs/Foo.pb.h> include path intact.
find_package(Protobuf REQUIRED)

# proto_path is the msgs dir so bare imports (import "Imu.proto") resolve;
# include/ on the compile path then resolves <volasim/comms/msgs/Foo.pb.h>.
set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/volasim/comms/msgs)
file(GLOB PROTO_FILES ${PROTO_DIR}/*.proto)

set(PROTO_SRCS "")
foreach(proto ${PROTO_FILES})
  get_filename_component(proto_name ${proto} NAME_WE)
  set(gen_cc ${PROTO_DIR}/${proto_name}.pb.cc)
  add_custom_command(
    OUTPUT ${gen_cc} ${PROTO_DIR}/${proto_name}.pb.h
    COMMAND protobuf::protoc
    ARGS --proto_path=${PROTO_DIR} --cpp_out=${PROTO_DIR} ${proto}
    DEPENDS ${proto} protobuf::protoc
    VERBATIM
  )
  list(APPEND PROTO_SRCS ${gen_cc})
endforeach()

add_library(myproto STATIC ${PROTO_SRCS})
target_link_libraries(myproto PUBLIC protobuf::libprotobuf)
set_target_properties(myproto PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

add_subdirectory(volasim_node)
