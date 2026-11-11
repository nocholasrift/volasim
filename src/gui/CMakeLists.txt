# The GUI is its own library so ImGui never leaks into the sim: only gui.cpp
# includes ImGui, and the sim talks to it through include/volasim/gui/gui.h.
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/third_party/imgui)

add_library(volasim_gui STATIC
  gui.cpp
  style.cpp
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_sdl3.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(volasim_gui
  PUBLIC  ${CMAKE_SOURCE_DIR}/include
  PRIVATE ${IMGUI_DIR} ${IMGUI_DIR}/backends
)
target_link_libraries(volasim_gui
  PUBLIC  SDL3::SDL3
  PRIVATE ${OPENGL_LIBRARIES}
)
set_target_properties(volasim_gui PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
