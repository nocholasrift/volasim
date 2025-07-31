#ifndef SIMULATION_H
#define SIMULATION_H

#include <volasim/sensors/depth_sensor.h>
#include <volasim/simulation/shader.h>

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_opengl.h>

#include <volasim/event/event_dispatcher.h>
#include <volasim/simulation/camera.h>
#include <volasim/simulation/display_object_container.h>
#include <volasim/simulation/input_manager.h>
#include <volasim/simulation/physics_interface.h>

static const std::string mesh_vertex_shader =
    "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"
    "uniform mat4 mvp;\n"
    "void main() {\n"
    "gl_Position = mvp * vec4(aPos, 1.0f);\n"
    "}\n";

static const std::string mesh_fragment_shader =
    "#version 330 core\n"
    "uniform vec3 color;\n"
    "out vec4 FragColor;\n"
    "void main(){\n"
    "FragColor = vec4(color, 1.0);\n"
    "}\n";

class Simulation {

 public:
  // singleton patternsimulation.h
  static Simulation& getInstance(int win_width, int win_height, int fps) {
    static Simulation instance(win_width, win_height, fps);
    return instance;
  }

  Simulation(int win_width, int win_height, int fps);

  SDL_AppResult update(void* appstate);
  SDL_AppResult SDLEvent(void* appstate, SDL_Event* event);

  SDL_AppResult initSDL(void** appstate, int argc, char* argv[]);
  void quitSDL(void* appstate, SDL_AppResult result);

  EventDispatcher& getHandler() { return event_handler_; }
  PhysicsInterface& getPhysicsInterface() { return physics_interface_; }

 private:
  int window_width_;
  int window_height_;
  int frames_per_sec_;

  double time_;

  Uint64 ms_per_frame_;
  Uint64 frame_start_;
  Uint64 last_step_;

  SDL_Window* window_;
  SDL_GLContext gl_ctx_;

  // std::unique_ptr<DisplayObjectContainer> world_;
  DisplayObjectContainer* world_;

  EventDispatcher& event_handler_;
  PhysicsInterface& physics_interface_;

  Camera camera_;

  InputManager input_manager_;

  std::unique_ptr<GPUSensor> depth_sensor_;

  Shader shape_shader_;
};

#endif
