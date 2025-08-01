cmake_minimum_required(VERSION 3.16)
project(volasim C CXX)

add_compile_options(-std=c++17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")

add_subdirectory(third_party/SDL EXCLUDE_FROM_ALL)
add_subdirectory(third_party/JoltPhysics/Build EXCLUDE_FROM_ALL)
add_subdirectory(third_party/pugixml EXCLUDE_FROM_ALL)

find_package(OpenGL REQUIRED)
find_package(glm REQUIRED)
find_package(GLUT REQUIRED)
find_package(Eigen3 REQUIRED system)

include_directories(
  include/
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(volasim 
  src/main.cpp
  src/glad/glad.c
  src/simulation/simulation.cpp
  src/simulation/xml_parser.cpp
  src/simulation/display_object.cpp
  src/simulation/display_object_container.cpp
  src/simulation/shape_renderable.cpp
  src/simulation/physics_interface.cpp
  src/vehicles/drone.cpp
  src/event/event_dispatcher.cpp
)
target_link_libraries(volasim
  PRIVATE 
  SDL3::SDL3
  Jolt::Jolt
  pugixml
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${GLM_LIBRARIES}
)
