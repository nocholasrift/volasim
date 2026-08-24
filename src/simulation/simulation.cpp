#include <glad/glad.h>

#include <volasim/comms/frames.h>
#include <volasim/comms/msgs/Transform.pb.h>
#include <volasim/comms/topics.h>
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
#include <algorithm>
#include <chrono>
#include <cmath>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <stdexcept>
#include <unordered_set>

namespace {

int64_t nowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

void fillTransform(volasim_msgs::TransformStamped* tf, int64_t stamp_ns,
                   uint32_t drone_id, const std::string& parent_frame,
                   const std::string& child_frame, const glm::vec3& t,
                   const glm::quat& q) {
  volasim_msgs::Header* header = tf->mutable_header();
  header->set_stamp_ns(stamp_ns);
  header->set_drone_id(drone_id);
  header->set_frame_id(parent_frame);  // parent frame, per the tf convention
  tf->set_child_frame_id(child_frame);

  volasim_msgs::Odometry_Position* trans = tf->mutable_translation();
  trans->set_x(t.x);
  trans->set_y(t.y);
  trans->set_z(t.z);

  volasim_msgs::Odometry_Orientation* rot = tf->mutable_rotation();
  rot->set_x(q.x);
  rot->set_y(q.y);
  rot->set_z(q.z);
  rot->set_w(q.w);
}

}  // namespace

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

  physics_step_seconds_ = 1. / args.physics_hz;
  report_rates_         = args.report_rates;
  interpolate_          = args.interpolate;

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

  // ids are assigned by the parser; init() needs the live GL context
  for (GPUSensor& sensor : gpu_sensors_) {
    sensor.init();
  }

  // Sensor mount poses are fixed and the scene graph is complete here, before
  // physics can mutate anything, so compose the static tf tree once.
  buildStaticTransforms();

  const std::vector<SimBody>& sim_bodies = physics_interface_.dynamicBodies();

  // default camera target to the first dynamic object; else focus the origin
  if (!sim_bodies.empty()) {
    camera().setTarget(sim_bodies.front().entity);
  }

  setSimState();
  setTransforms();

  {
    std::unique_lock<std::mutex> lock(running_mtx_);
    is_running_ = true;
  }
  running_cv_.notify_all();

  // world loading is done, so the first report covers running frames only
  render_rate_.reset();

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

  if (report_rates_) {
    render_rate_.tick();
  }

  world_buffer_.read(render_frames_);

  // Without interpolation the newest step is drawn as it stands, which holds a
  // pose across frames whenever physics steps slower than the display.
  const WorldSnapshot* poses = &render_frames_.curr;
  if (interpolate_ && render_frames_.has_prev) {
    render_poses_.blendFrom(
        render_frames_.prev, render_frames_.curr,
        interpolationAlpha(std::chrono::steady_clock::now(),
                           render_frames_.curr_time, physics_step_seconds_));
    poses = &render_poses_;
  }

  glm::mat4 view_mat = camera().getViewMatrix(*poses);

  glm::mat4 proj_mat = glm::perspective(
      glm::radians(camera().getFov()),                     // fov
      static_cast<float>(window_width_) / window_height_,  // aspect ratio
      0.1F, 100.0f);                                       // near & far plane

  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // sadly, depth sensor updates and handoff to comms must happen in the render thread
  // due to the coupling with openGL.
  const auto now = std::chrono::steady_clock::now();
  for (GPUSensor& sensor : gpu_sensors_) {
    if (sensor.captureDue(now)) {
      sensor.update(*world_, *poses, shape_shader_);
      sensor.captureDepth();
    }

    SensorFrame frame;
    if (sensor.tryReadback(frame)) {
      sensor_handoff_.publish(sensor.sensorKey(), std::move(frame));
    }
  }

  glUseProgram(shape_shader_.getID());

  shape_shader_.setUniformVec3("lightColor", glm::vec3(.8F, .8F, .8F));
  shape_shader_.setUniformVec3("lightPos", glm::vec3(0, 0, 5));

  world_->draw(*poses, glm::mat4(1.F), view_mat, proj_mat, shape_shader_);
  for (GPUSensor& sensor : gpu_sensors_) {
    sensor.draw(view_mat, proj_mat, shape_shader_);
  }

  SDL_GL_SwapWindow(window_);

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

void Simulation::physicsLoop() {
  using clock = std::chrono::steady_clock;

  LoopPacer pacer(physics_step_seconds_, clock::now());

  RateCounter rate("physics");

  while (is_running_.load()) {
    applyPendingInput();
    physics_interface_.update(physics_step_seconds_, world_buffer_);
    setSimState();
    setTransforms();

    if (report_rates_) {
      rate.tick();
    }

    const clock::time_point deadline = pacer.nextDeadline(clock::now());

    // A wait rather than a sleep, so quitSDL's notify releases this thread at
    // once instead of it having to run out the step period. The deadline is
    // the only thing tying this loop to real time.
    {
      std::unique_lock<std::mutex> lock(running_mtx_);
      running_cv_.wait_until(lock, deadline,
                             [this] { return !is_running_.load(); });
    }
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

  // reached with nothing initialized when startup failed
  if (gl_ctx_ != nullptr) {
    SDL_GL_DestroyContext(gl_ctx_);
  }

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

  std::lock_guard<std::mutex> lock(sim_state_.mutex);
  for (const SimBody& sb : sim_bodies) {
    // Serialize into a temporary so a failed encode leaves the drone's last
    // good state in place rather than clobbering it with a partial write.
    std::string bytes;
    if (!sb.entity->getDynamics().getSimState().SerializeToString(&bytes)) {
      std::cerr << "[Simulation] Failed to serialize state for drone "
                << sb.vehicle_id << "\n";
      continue;
    }
    sim_state_.states[sb.vehicle_id] = std::move(bytes);
  }
}

void Simulation::waitUntilRunning() {
  std::unique_lock<std::mutex> lock(running_mtx_);
  running_cv_.wait(
      lock, [this] { return is_running_.load() || is_shutting_down_.load(); });
}

std::unordered_map<uint32_t, std::string> Simulation::getSimState() {
  std::lock_guard<std::mutex> lock(sim_state_.mutex);
  return sim_state_.states;
}

std::vector<SensorFrame> Simulation::drainSensorFrames() {
  return sensor_handoff_.drain();
}

void Simulation::setTransforms() {
  const std::vector<SimBody>& sim_bodies = physics_interface_.dynamicBodies();
  if (sim_bodies.empty()) {
    return;
  }

  const int64_t stamp_ns = nowNs();

  std::lock_guard<std::mutex> lock(tf_state_.mutex);
  for (const SimBody& sb : sim_bodies) {
    DynamicObject& dynamics = sb.entity->getDynamics();

    volasim_msgs::TFMessage msg;
    fillTransform(msg.add_transforms(), stamp_ns, sb.vehicle_id,
                  volasim::frames::odom(sb.vehicle_id),
                  volasim::frames::baseLink(sb.vehicle_id),
                  dynamics.getTranslation(), dynamics.getRotation());

    // Serialize into a temporary so a failed encode leaves the drone's last
    // good tf in place rather than clobbering it with a partial write.
    std::string bytes;
    if (!msg.SerializeToString(&bytes)) {
      std::cerr << "[Simulation] Failed to serialize tf for drone "
                << sb.vehicle_id << "\n";
      continue;
    }
    tf_state_.states[sb.vehicle_id] = std::move(bytes);
  }
}

void Simulation::buildStaticTransforms() {
  const int64_t stamp_ns = nowNs();

  // Fixed rotation from a sensor's link frame (REP-103: x-forward, y-left,
  // z-up) to its optical frame (REP-145: x-right, y-down, z-forward). glm quat
  // order is (w, x, y, z).
  const glm::quat kLinkToOptical(0.5F, -0.5F, 0.5F, -0.5F);

  // Which drone each vehicle body belongs to, so a sensor's owning drone can be
  // found from the entity it is mounted on.
  std::unordered_map<const Entity*, uint32_t> drone_of_vehicle;
  for (const SimBody& sb : physics_interface_.dynamicBodies()) {
    drone_of_vehicle[sb.entity] = sb.vehicle_id;
  }

  // Group each drone's sensor edges into one TFMessage so a subscriber receives
  // the whole static subtree atomically.
  std::unordered_map<uint32_t, volasim_msgs::TFMessage> per_drone;

  // Sensor names must be unique within a drone; a repeat gets a numeric suffix
  // until it no longer clashes with a name already taken on that drone.
  std::unordered_map<uint32_t, std::unordered_set<std::string>> taken;

  for (GPUSensor& sensor : gpu_sensors_) {
    const Entity& sensor_entity = sensor.entity();
    const Entity* vehicle       = sensor_entity.getParent();

    const auto     it       = drone_of_vehicle.find(vehicle);
    const uint32_t drone_id = it != drone_of_vehicle.end() ? it->second : 0;

    const std::string base = sensor_entity.getName();
    std::string       name = base;
    for (uint32_t n = 1; !taken[drone_id].insert(name).second; ++n) {
      name = base + "_" + std::to_string(n);
    }

    const std::string link    = volasim::frames::sensor(drone_id, name);
    const std::string optical = volasim::frames::sensorOptical(drone_id, name);

    // Hand the sensor its opaque publish labels; it stamps clouds in the optical
    // frame and publishes on the drone's depth topic.
    sensor.setFrameId(optical);
    sensor.setTopic(volasim::topics::depth(drone_id));

    // base_link -> sensor link frame (the physical mount pose).
    const Transform& mount = sensor_entity.getLocalTransform();
    fillTransform(per_drone[drone_id].add_transforms(), stamp_ns, drone_id,
                  volasim::frames::baseLink(drone_id), link, mount.position,
                  mount.rotation);

    // sensor link -> optical frame. Depth clouds are published in the optical
    // frame, so without this edge a z-forward cloud would be drawn along the
    // link frame's z (up) axis.
    fillTransform(per_drone[drone_id].add_transforms(), stamp_ns, drone_id, link,
                  optical, glm::vec3(0.F), kLinkToOptical);
  }

  static_tf_.clear();
  for (const auto& [drone_id, msg] : per_drone) {
    std::string bytes;
    if (!msg.SerializeToString(&bytes)) {
      std::cerr << "[Simulation] Failed to serialize static tf for drone "
                << drone_id << "\n";
      continue;
    }
    static_tf_[drone_id] = std::move(bytes);
  }
}

std::unordered_map<uint32_t, std::string> Simulation::getTransforms() {
  std::lock_guard<std::mutex> lock(tf_state_.mutex);
  return tf_state_.states;
}

std::unordered_map<uint32_t, std::string> Simulation::getStaticTransforms() {
  return static_tf_;
}
