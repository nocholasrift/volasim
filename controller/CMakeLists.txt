add_library(vola_controller
  src/lee_controller.cpp
  src/matrix_utils.cpp
  src/minjerk_generator.cpp
)
target_include_directories(vola_controller PUBLIC include)
target_link_libraries(vola_controller PUBLIC Eigen3::Eigen)
target_compile_features(vola_controller PUBLIC cxx_std_17)
