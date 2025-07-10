cmake_minimum_required(VERSION 3.16)
project(volasim C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")

add_subdirectory(third_party/SDL EXCLUDE_FROM_ALL)

find_package(OpenGL REQUIRED)
find_package(glm REQUIRED)
find_package(GLUT REQUIRED)

include_directories(include/)

add_executable(volasim 
  src/main.cpp
  src/simulation/simulation.cpp
  src/simulation/display_object.cpp
  src/simulation/display_object_container.cpp
  src/simulation/shape_renderable.cpp
  src/vehicles/drone.cpp
)
target_link_libraries(volasim
  PRIVATE 
  SDL3::SDL3
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${GLM_LIBRARIES}
)
