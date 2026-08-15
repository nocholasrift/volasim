#include <glad/glad.h>

#include <volasim/simulation/simulation.h>
#include <volasim/vehicles/drone.h>
#include "SDL3/SDL_video.h"

#ifdef USE_APPLE_OPENGL_HEADERS
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#include <GL/glut.h>
#endif
#include <chrono>
#include <cmath>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <stdexcept>

Simulation::Simulation()
    : event_handler_(EventDispatcher::getInstance()),
      physics_interface_(PhysicsInterface::getInstance()) {

  event_handler_.addEventListener(&PhysicsInterface::getInstance(), "OBJ_ADD");
  event_handler_.addEventListener(&PhysicsInterface::getInstance(), "OBJ_RM");
}

// Simulation::~Simulation() {
//   delete world_;
// }

SDL_AppResult Simulation::initSDL(void** appstate, int argc, char* argv[],
                                  const Args& args) {
  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  SDL_SetAppMetadata("Vola Simulator", "1.0",
                     "github.com/nocholasrift/volasim");

  glutInit(&argc, argv);

  XMLParser xml_parser(args.world_path);
  cameras_.push_back(xml_parser.loadCamera());
  cameras_.back().setID(cameras_.size() - 1);

  Camera::Dimensions camera_dims = camera().getDimensions();

  window_width_   = camera_dims.width;
  window_height_  = camera_dims.height;
  frames_per_sec_ = camera().getFPS();

  window_ = SDL_CreateWindow("Volasim", window_width_, window_height_,
                             SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);

  // camera_ = Camera(cam_settings);

  // SDL_SetWindowRelativeMouseMode(window_, true);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

  gl_ctx_ = SDL_GL_CreateContext(window_);
  SDL_GL_MakeCurrent(window_, gl_ctx_);
  SDL_GL_SetSwapInterval(1);  // enable vsync

  if (!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress)) {
    std::cerr << "Failed to initialize GLAD\n";
    return SDL_APP_FAILURE;
  }

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);  // Additional light
  glEnable(GL_PROGRAM_POINT_SIZE);

  // Brighter directional light
  GLfloat light0_pos[]     = {1.0F, 2.0F, 3.0F, 0.0F};  // directional light
  GLfloat light0_diffuse[] = {0.8F, 0.8F, 0.8F, 1.0F};
  glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);

  // Ambient fill light
  GLfloat light1_pos[]     = {-1.0F, -1.0F, 0.5F, 0.0F};  // another directional
  GLfloat light1_diffuse[] = {0.3F, 0.3F, 0.3F, 1.0F};
  glLightfv(GL_LIGHT1, GL_POSITION, light1_pos);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);

  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

  glShadeModel(GL_SMOOTH);  // for better lighting transitions

  glClearColor(0.25F, 0.25F, 0.25F, 1.0F);  // lighter background

  int major = 0;
  int minor = 0;

  glGetIntegerv(GL_MAJOR_VERSION, &major);
  glGetIntegerv(GL_MINOR_VERSION, &minor);

  std::cout << "OpenGL version: " << major << "." << minor << '\n';

  time_        = 0.;
  frame_start_ = -1000000;

  ms_per_frame_ = 1000 / frames_per_sec_;

  shape_shader_ = Shader(mesh_vertex_shader, mesh_fragment_shader);

  // parser populates the scene graph under world_ and returns any depth sensors.
  // world_ lives for the whole program; if it is ever reset/reloaded, dispatch
  // OBJ_RM for the old subtree (children-first) before destroying it, otherwise
  // physics bodies outlive their entities. removeChild() already does this per node.
  auto t_start = std::chrono::steady_clock::now();
  world_       = xml_parser.loadWorldFromXML(gpu_sensors_);
  auto elapsed = std::chrono::steady_clock::now() - t_start;
  std::cout
      << "World load time: "
      << std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count() /
             1e9
      << "\n";

  // sensors are constructed during parsing; init() needs the live GL context
  for (GPUSensor& sensor : gpu_sensors_) {
    sensor.init();
  }

  const std::vector<SimBody>& sim_bodies = physics_interface_.dynamicBodies();

  // default camera target to the first dynamic object; else focus the origin
  if (!sim_bodies.empty()) {
    camera().setTarget(sim_bodies.front().entity);
  }

  setSimState();

  {
    std::unique_lock<std::mutex> lock(running_mtx_);
    is_running_ = true;
  }
  running_cv_.notify_all();

  // The scene graph is complete at this point: physics may now step it
  // concurrently with rendering, which is only safe while no bodies are
  // added or removed.
  physics_thread_ = std::thread([this] { physicsLoop(); });

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

SDL_AppResult Simulation::SDLEvent(void* appstate, SDL_Event* event) {
  switch (event->type) {
    case SDL_EVENT_QUIT:
      return SDL_APP_SUCCESS;
    case SDL_EVENT_MOUSE_MOTION:
      input_manager_.mouseX = event->motion.xrel;
      input_manager_.mouseY = event->motion.yrel;
      camera().processMouseMovement(input_manager_.mouseX,
                                    input_manager_.mouseY);
      break;
    case SDL_EVENT_MOUSE_WHEEL:
      camera().processMouseScroll(event->wheel.y);
      break;
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
      if (event->button.button == kMouseRightClick)
        camera().enableOrbitAndPan();
      break;
    case SDL_EVENT_MOUSE_BUTTON_UP:
      camera().disableOrbitAndPan();
      break;
    case SDL_EVENT_KEY_DOWN:
      if (event->key.key == SDLK_Q)
        return SDL_APP_SUCCESS;
    default:
      return SDL_APP_CONTINUE;
  }

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

// Render only — the world is stepped on physics_thread_ and reaches us through
// world_buffer_, so a frame always shows one whole physics step.
SDL_AppResult Simulation::update(void* appstate) {
  Uint64 duration = SDL_GetTicks() - frame_start_;
  if (duration <= ms_per_frame_) {
    // nothing to do until the next frame is due; don't spin on a core
    SDL_Delay(1);
    return SDL_APP_CONTINUE;
  }

  frame_start_ = SDL_GetTicks();

  world_buffer_.read(render_poses_);

  glm::mat4 view_mat = camera().getViewMatrix(render_poses_);

  glm::mat4 proj_mat = glm::perspective(
      glm::radians(camera().getFov()),                     // fov
      static_cast<float>(window_width_) / window_height_,  // aspect ratio
      0.1F, 100.0f);                                       // near & far plane

  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  for (GPUSensor& sensor : gpu_sensors_) {
    sensor.update(*world_, render_poses_, shape_shader_);
  }

  glUseProgram(shape_shader_.getID());

  shape_shader_.setUniformVec3("lightColor", glm::vec3(.8F, .8F, .8F));
  shape_shader_.setUniformVec3("lightPos", glm::vec3(0, 0, 5));

  world_->draw(render_poses_, glm::mat4(1.F), view_mat, proj_mat,
               shape_shader_);
  for (GPUSensor& sensor : gpu_sensors_) {
    sensor.draw(view_mat, proj_mat, shape_shader_);
  }

  SDL_GL_SwapWindow(window_);

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

void Simulation::physicsLoop() {
  using clock = std::chrono::steady_clock;

  const clock::duration step = std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(kPhysicsStepSeconds));

  clock::time_point next_step = clock::now() + step;

  while (is_running_.load()) {
    applyPendingInput();
    physics_interface_.update(kPhysicsStepSeconds, world_buffer_);
    setSimState();

    next_step += step;

    // Running behind: drop the missed steps instead of trying to catch up,
    // which would only push us further behind.
    const clock::time_point now = clock::now();
    if (next_step < now) {
      next_step = now + step;
    }

    std::this_thread::sleep_until(next_step);
  }
}

void Simulation::quitSDL(void* appstate, SDL_AppResult result) {
  {
    std::unique_lock<std::mutex> lock(running_mtx_);
    is_running_       = false;
    is_shutting_down_ = true;
  }
  running_cv_.notify_all();

  if (physics_thread_.joinable()) {
    physics_thread_.join();
  }

  SDL_GL_DestroyContext(gl_ctx_);
  SDL_Quit();
}

void Simulation::setInputs(const std::string& buffer) {
  std::lock_guard<std::mutex> lock(input_mtx_);

  // Only the newest command matters; the physics thread applies it on its next
  // step rather than reaching into the dynamics from the comms thread.
  pending_input_     = buffer;
  has_pending_input_ = true;
}

void Simulation::applyPendingInput() {
  std::string buffer;

  {
    std::lock_guard<std::mutex> lock(input_mtx_);
    if (!has_pending_input_) {
      return;
    }
    buffer.swap(pending_input_);
    has_pending_input_ = false;
  }

  const std::vector<SimBody>& sim_bodies = physics_interface_.dynamicBodies();
  if (sim_bodies.empty()) {
    return;
  }

  sim_bodies.front().entity->getDynamics().setInput(buffer);
}

void Simulation::setSimState() {
  const std::vector<SimBody>& sim_bodies = physics_interface_.dynamicBodies();

  if (sim_bodies.empty()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(sim_state_.mutex);
    for (const SimBody& sb : sim_bodies) {
      if (!sb.entity->getDynamics().getSimState().SerializeToString(
              &sim_state_.state)) {
        std::cerr << "[Simulation] Failed to serialize state\n";
      }
      break;
    }
  }
}

const std::string Simulation::getSimState() {
  // comms blocks here until the world is loaded, or gives up if the sim is
  // torn down before it ever starts
  {
    std::unique_lock<std::mutex> lock(running_mtx_);
    running_cv_.wait(lock, [this] {
      return is_running_.load() || is_shutting_down_.load();
    });
  }

  std::lock_guard<std::mutex> lock(sim_state_.mutex);
  std::string                 state = sim_state_.state;

  return state;
}
