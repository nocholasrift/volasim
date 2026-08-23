#ifndef SIMULATION_H
#define SIMULATION_H

#include <volasim/args.h>
#include <volasim/event/event_dispatcher.h>
#include <volasim/sensors/depth_sensor.h>
#include <volasim/simulation/camera.h>
#include <volasim/simulation/entity.h>
#include <volasim/simulation/input_manager.h>
#include <volasim/sensors/depth_frame.h>
#include <volasim/simulation/loop_pacer.h>
#include <volasim/simulation/physics_interface.h>
#include <volasim/simulation/rate_counter.h>
#include <volasim/simulation/shader.h>
#include <volasim/simulation/world_buffer.h>
#include <volasim/simulation/xml_parser.h>
#include <volasim/types.h>

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_opengl.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <list>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>

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
    "float ambient = 0.1;\n"
    "vec3 result = (diffuse + ambient) * color;\n"
    "FragColor = vec4(result, 1.0);\n"
    "}\n";

class Simulation {

 public:
  // singleton pattern
  static Simulation& getInstance() {
    static Simulation instance;
    return instance;
  }

  // ~Simulation();

  SDL_AppResult update(void* appstate);
  SDL_AppResult SDLEvent(void* appstate, SDL_Event* event);

  SDL_AppResult initSDL(void** appstate, int argc, char* argv[],
                        const Args& args);
  void          quitSDL(void* appstate, SDL_AppResult result);

  bool isRunning() { return is_running_.load(); }

  void setSimState();
  void setInputs(const std::string& buffer);

  const Camera& camera() const { return cameras_[active_camera]; }
  Camera&       camera() { return cameras_[active_camera]; }

  // Blocks until the world is loaded and the sim is running, or until it is torn
  // down before ever starting. The comms thread calls this once before its loop.
  void waitUntilRunning();

  // Snapshot of the latest serialized simulation state.
  std::unordered_map<uint32_t, std::string> getSimState();

  // Newest depth frame per sensor, moved out for the comms thread to publish.
  std::vector<DepthFrame> drainCloudFrames();

  EventDispatcher&  getHandler() { return event_handler_; }
  PhysicsInterface& getPhysicsInterface() { return physics_interface_; }

 private:
  Simulation();

  // Steps physics at a fixed rate on its own thread until the sim stops.
  void physicsLoop();

  // Hands the newest command from the comms thread to the dynamics. Physics
  // thread only — nothing else may touch a DynamicObject while it is stepping.
  void applyPendingInput();

  static constexpr uint8_t kMouseRightClick  = 1;
  static constexpr uint8_t kMouseMiddleClick = 2;
  static constexpr uint8_t kMouseLeftClick   = 3;

  int window_width_;
  int window_height_;

  unsigned int frames_per_sec_;

  double time_;

  std::atomic<bool>       is_running_       = false;
  std::atomic<bool>       is_shutting_down_ = false;
  std::condition_variable running_cv_;
  std::mutex              running_mtx_;

  std::thread physics_thread_;

  // set from the command line; the render loop's rate comes from the world XML
  double physics_step_seconds_{1. / 1000.};

  // newest depth frame per sensor, handed from the render thread to comms. Each
  // sensor paces its own captures at the rate_hz in its XML definition.
  CloudHandoff cloud_handoff_;

  bool        report_rates_{false};
  RateCounter render_rate_{"render", "fps"};

  Uint64 ms_per_frame_;
  Uint64 frame_start_;

  SDL_Window*   window_{nullptr};
  SDL_GLContext gl_ctx_{nullptr};

  std::unique_ptr<Entity> world_;

  // physics publishes poses here; the render thread reads whole steps at a time
  WorldBuffer world_buffer_;

  // render thread only
  PoseFrames    render_frames_;
  WorldSnapshot render_poses_;  // the blend of those frames, when interpolating

  bool interpolate_{false};

  std::mutex  input_mtx_;
  std::string pending_input_;
  bool        has_pending_input_{false};

  EventDispatcher&  event_handler_;
  PhysicsInterface& physics_interface_;

  std::vector<Camera> cameras_;
  uint8_t             active_camera{0};

  InputManager input_manager_;

  std::list<GPUSensor> gpu_sensors_;

  Shader shape_shader_;

  SimState sim_state_;
};

#endif
