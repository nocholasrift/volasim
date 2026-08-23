# Unit tests for the parts that need no window, no physics engine and no clock.
# Kept off SDL/Jolt/assimp deliberately: this target builds in seconds, so it is
# cheap enough to run on every change.
add_executable(volasim_tests
  main.cpp
  test_args.cpp
  test_transform.cpp
  test_world_buffer.cpp
  test_loop_pacer.cpp
  test_topics.cpp
  test_rate_gate.cpp
  ${CMAKE_SOURCE_DIR}/src/simulation/world_buffer.cpp
)

target_include_directories(volasim_tests PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/third_party/doctest
)

if(TARGET glm::glm)
  target_link_libraries(volasim_tests PRIVATE glm::glm)
endif()

set_target_properties(volasim_tests PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_compile_definitions(volasim_tests PRIVATE GLM_ENABLE_EXPERIMENTAL)

add_test(NAME volasim_tests COMMAND volasim_tests)
