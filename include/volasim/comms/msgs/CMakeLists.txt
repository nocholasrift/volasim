find_package(protobuf CONFIG REQUIRED)


set(PROTO_FILES
  Thrust.proto
  Imu.proto
  Odometry.proto
  DroneState.proto
)

add_library(myproto STATIC ${PROTO_FILES})
target_include_directories(myproto INTERFACE ${CMAKE_CURRENT_BINARY_DIR})
set_target_properties(myproto PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

target_link_libraries(myproto
  PUBLIC protobuf::libprotobuf
)

protobuf_generate(TARGET myproto LANGUAGE cpp PROTOC_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

