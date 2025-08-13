#ifndef SIMULATION_H
#define SIMULATION_H

#include <volasim/event/event_dispatcher.h>
#include <volasim/sensors/depth_sensor.h>
#include <volasim/simulation/camera.h>
#include <volasim/simulation/display_object_container.h>
#include <volasim/simulation/input_manager.h>
#include <volasim/simulation/physics_interface.h>
#include <volasim/simulation/shader.h>
#include <volasim/types.h>

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_opengl.h>

#include <condition_variable>
#include <string_view>
#include <thread>

static const std::string mesh_vertex_shader =
    "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"
    "layout (location = 1) in vec3 aNormal;\n"
    "uniform mat4 model;\n"
    "uniform mat4 mvp;\n"
    "out vec3 Normal;\n"
    "out vec3 FragPos;\n"
    "void main() {\n"
    "gl_Position = mvp * vec4(aPos, 1.0f);\n"
    "FragPos = vec3(model * vec4(aPos, 1.0f));\n"
    "Normal = aNormal;\n"
    "}\n";

static const std::string mesh_fragment_shader =
    "#version 330 core\n"
    "in vec3 Normal;\n"
    "in vec3 FragPos;\n"
    "uniform vec3 lightColor;\n"
    "uniform vec3 lightPos;\n"
    "uniform vec3 color;\n"
    "out vec4 FragColor;\n"
    "void main(){\n"
    "vec3 norm = Normal;\n"
    "vec3 lightDir = normalize(lightPos - FragPos);\n"
    "float diff = max(dot(norm, lightDir), 0.0);\n"
    "vec3 diffuse = diff * lightColor;\n"
    "vec3 result = diffuse * color;\n"
    "FragColor = vec4(result, 1.0);\n"
    "}\n";

class Simulation {

 public:
  // singleton patternsimulation.h
  static Simulation& getInstance(int win_width, int win_height, int fps) {
    static Simulation instance(win_width, win_height, fps);
    return instance;
  }

  ~Simulation();

  SDL_AppResult update(void* appstate);
  SDL_AppResult SDLEvent(void* appstate, SDL_Event* event);

  SDL_AppResult initSDL(void** appstate, int argc, char* argv[]);
  void quitSDL(void* appstate, SDL_AppResult result);

  bool isRunning() { return is_running_.load(); }

  void setSimState();
  void setInputs(const std::string& buffer);

  const std::string getSimState();
  EventDispatcher& getHandler() { return event_handler_; }
  PhysicsInterface& getPhysicsInterface() { return physics_interface_; }

 private:
  Simulation(int win_width, int win_height, int fps);

  static constexpr uint8_t kMouseRightClick = 1;
  static constexpr uint8_t kMouseMiddleClick = 2;
  static constexpr uint8_t kMouseLeftClick = 3;

  int window_width_;
  int window_height_;
  int frames_per_sec_;

  double time_;

  std::atomic<bool> is_running_ = false;
  std::condition_variable running_cv_;
  std::mutex running_mtx_;

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

  SimState sim_state_;
};

#endif
