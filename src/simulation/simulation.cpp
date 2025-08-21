#include <glad/glad.h>

#include <volasim/simulation/simulation.h>
#include <volasim/vehicles/drone.h>

#include <GL/glu.h>
#include <GL/glut.h>  // for glutSolidSphere
#include <math.h>
#include <chrono>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <stdexcept>

// EventDispatcher& event_handler_ = EventDispatcher::getInstance();
// PhysicsInterface& physics_interface_ = PhysicsInterface::getInstance();

Simulation::Simulation()
    : event_handler_(EventDispatcher::getInstance()),
      physics_interface_(PhysicsInterface::getInstance()) {

  // event_handler_ = EventDispatcher::getInstance();
  // physics_interface_ = PhysicsInterface::getInstance();

  event_handler_.addEventListener(&PhysicsInterface::getInstance(), "OBJ_ADD");
  event_handler_.addEventListener(&PhysicsInterface::getInstance(), "OBJ_RM");

  world_ = new DisplayObjectContainer("world");

  DepthSensorSettings props;
  props.width = 640;
  props.height = 480;
  props.fx = 550.0f;
  props.fy = 550.0f;
  props.cx = props.width / 2;
  props.cy = props.height / 2;
  props.z_near = 0.25f;
  props.z_far = 10.0f;

  glm::vec3 cam_pos = glm::vec3(3, 3, 7);

  glm::mat4 cam_world_pos =
      glm::lookAt(cam_pos, glm::vec3(0, 0, 0), glm::vec3(0, 0, 1));
  cam_world_pos =
      glm::rotate(cam_world_pos, glm::pi<float>(), glm::vec3(1, 0, 0));

  glm::vec3 translation = glm::vec3(cam_world_pos[3]);
  glm::mat3 rotation = glm::mat3(cam_world_pos);

  // xml_parser_ =
  //     std::make_unique<XMLParser>("./definitions/worlds/world_250_world.xml");

  // DisplayObject* depth_sensor = new DisplayObject("depth_sensor");
  // depth_sensor->setTranslation(translation);
  // depth_sensor->setRotation(glm::eulerAngles(glm::quat_cast(rotation)));
  // world_->addChild(depth_sensor);

  // depth_sensor_ = std::make_unique<GPUSensor>(props, depth_sensor);
}

Simulation::~Simulation() {
  delete world_;
}

SDL_AppResult Simulation::initSDL(void** appstate, int argc, char* argv[]) {
  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  SDL_SetAppMetadata("Vola Simulator", "1.0",
                     "github.com/nocholasrift/volasim");

  glutInit(&argc, argv);

  XMLParser xml_parser("./definitions/worlds/world_250_world.xml");
  CameraSettings cam_settings = xml_parser.getCameraSettings();

  window_width_ = cam_settings.window_sz[0];
  window_height_ = cam_settings.window_sz[1];
  frames_per_sec_ = cam_settings.fps;

  window_ = SDL_CreateWindow("Floating Sphere", window_width_, window_height_,
                             SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);

  camera_ = Camera(cam_settings);

  // SDL_SetWindowRelativeMouseMode(window_, true);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  gl_ctx_ = SDL_GL_CreateContext(window_);
  SDL_GL_MakeCurrent(window_, gl_ctx_);
  SDL_GL_SetSwapInterval(1);  // enable vsync

  if (!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress)) {
    std::cerr << "Failed to initialize GLAD\n";
    return SDL_APP_FAILURE;
  }

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);  // Additional light

  // Brighter directional light
  GLfloat light0_pos[] = {1.0f, 2.0f, 3.0f, 0.0f};  // directional light
  GLfloat light0_diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
  glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);

  // Ambient fill light
  GLfloat light1_pos[] = {-1.0f, -1.0f, 0.5f, 0.0f};  // another directional
  GLfloat light1_diffuse[] = {0.3f, 0.3f, 0.3f, 1.0f};
  glLightfv(GL_LIGHT1, GL_POSITION, light1_pos);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);

  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

  glShadeModel(GL_SMOOTH);  // for better lighting transitions

  glClearColor(0.25f, 0.25f, 0.25f, 1.0f);  // lighter background

  time_ = 0.;
  frame_start_ = -1000000;
  last_step_ = 0;

  ms_per_frame_ = 1000 / frames_per_sec_;

  shape_shader_ = Shader(mesh_vertex_shader, mesh_fragment_shader);

  // XMLParser parser("./definitions/worlds/demo_world.xml");
  xml_parser.loadWorldFromXML(world_);

  std::vector<DynamicObject*> dyna_objs =
      physics_interface_.getDynamicObjects();

  // default to first dynamic object registered as target
  // if no dynamic object, default to origin for focus
  if (dyna_objs.size() > 0)
    camera_.setTarget(dyna_objs[0]);

  setSimState();

  {
    std::unique_lock<std::mutex> lock(running_mtx_);
    is_running_ = true;
    running_cv_.notify_one();
  }

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

SDL_AppResult Simulation::SDLEvent(void* appstate, SDL_Event* event) {
  switch (event->type) {
    case SDL_EVENT_QUIT:
      return SDL_APP_SUCCESS;
    case SDL_EVENT_MOUSE_MOTION:
      input_manager_.mouseX = event->motion.xrel;
      input_manager_.mouseY = event->motion.yrel;
      camera_.processMouseMovement(input_manager_.mouseX,
                                   input_manager_.mouseY);
      break;
    case SDL_EVENT_MOUSE_WHEEL:
      camera_.processMouseScroll(event->wheel.y);
      break;
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
      if (event->button.button == kMouseRightClick)
        camera_.enableOrbitAndPan();
      break;
    case SDL_EVENT_MOUSE_BUTTON_UP:
      camera_.disableOrbitAndPan();
      break;
    case SDL_EVENT_KEY_DOWN:
      if (event->key.key == SDLK_Q)
        return SDL_APP_SUCCESS;
    default:
      return SDL_APP_CONTINUE;
  }

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

SDL_AppResult Simulation::update(void* appstate) {
  static std::chrono::system_clock::time_point last_time;

  Uint64 duration = SDL_GetTicks() - frame_start_;
  if (duration > ms_per_frame_) {
    // physics_interface_.update(ms_per_frame_ / 1000.);

    glm::mat4 view_mat = camera_.getViewMatrix();
    glm::mat4 proj_mat = glm::perspective(
        glm::radians(camera_.getFov()),                      // fov
        static_cast<float>(window_width_) / window_height_,  // aspect ratio
        0.1f, 100.0f);                                       // near & far plane

    // glUseProgram(shape_shader_.getID());
    // depth_sensor_->update(world_, shape_shader_);

    // depth_sensor_->draw(view_mat, proj_mat, shape_shader_);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(shape_shader_.getID());

    shape_shader_.setUniformVec3("lightColor", glm::vec3(.8f, .8f, .8f));
    shape_shader_.setUniformVec3("lightPos", glm::vec3(0, 0, 5));

    world_->draw(view_mat, proj_mat, shape_shader_);

    SDL_GL_SwapWindow(window_);

    frame_start_ = SDL_GetTicks();
  }

  if (last_step_ == 0) {
    last_time = std::chrono::high_resolution_clock::now();
    precise_time_ = std::chrono::high_resolution_clock::now();
    last_step_ = SDL_GetTicks();
    return SDL_APP_CONTINUE;
  }

  // DynamicDisplayWrapper<13, 4>* drone =
  //   (DynamicDisplayWrapper<13, 4>*) world_->getChild("drone");

  auto t_elapsed = std::chrono::high_resolution_clock::now() - precise_time_;

  // std::cout
  //     << std::chrono::duration_cast<std::chrono::nanoseconds>(t_elapsed).count()
  //     << std::endl;
  auto one_s_elapsed = std::chrono::high_resolution_clock::now() - last_time;
  double os_dt =
      std::chrono::duration_cast<std::chrono::milliseconds>(one_s_elapsed)
          .count();

  // std::cout << os_dt << "\n";

  // if (os_dt > 1e3) {
  //   std::vector<DynamicObject*> dyna_objs =
  //       physics_interface_.getDynamicObjects();
  //
  //   std::cout << "drone vel: " << dyna_objs[0]->getVelocity()[2] << "\n";
  //   last_time = std::chrono::high_resolution_clock::now();
  // }

  // double dt = (SDL_GetTicks() - last_step_) / 1000.;
  double dt =
      std::chrono::duration_cast<std::chrono::nanoseconds>(t_elapsed).count() /
      1e9;

  if (dt > 5e-3) {
    // std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(t_elapsed)
    //                  .count()
    //           << "\n";
    // std::cout << "dt is: " << dt << std::endl;
    physics_interface_.update(dt);
    setSimState();
    precise_time_ = std::chrono::high_resolution_clock::now();
  }

  last_step_ = SDL_GetTicks();

  // SDL_GL_SwapWindow(window_);

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

void Simulation::quitSDL(void* appstate, SDL_AppResult result) {
  {
    std::unique_lock<std::mutex> lock(running_mtx_);
    is_running_ = false;
  }

  SDL_GL_DestroyContext(gl_ctx_);
  SDL_Quit();
}

void Simulation::setInputs(const std::string& buffer) {
  std::vector<DynamicObject*> dyna_objs =
      physics_interface_.getDynamicObjects();

  if (dyna_objs.empty())
    return;

  {
    std::lock_guard<std::mutex> lock(sim_state_.mutex);
    for (DynamicObject* dyna_obj : dyna_objs) {
      if (!dyna_obj) {
        std::cerr << "[Simulation] null dynamic object detected in interface\n";
      }
      dyna_obj->setInput(buffer);
      break;
    }
  }
}

void Simulation::setSimState() {
  /*std::cout << "trying to get dynamic objects\n";*/
  std::vector<DynamicObject*> dyna_objs =
      physics_interface_.getDynamicObjects();
  /*std::cout << "checking if list empty\n";*/

  if (dyna_objs.empty())
    return;

  /*std::string buffer;*/

  {
    std::lock_guard<std::mutex> lock(sim_state_.mutex);
    for (DynamicObject* dyna_obj : dyna_objs) {
      if (!dyna_obj) {
        std::cerr << "[Simulation] null dynamic object detected in interface\n";
      }
      dyna_obj->getSimState().SerializeToString(&sim_state_.state);
      break;
    }
  }

  /*sim_state_.state = buffer;*/
}

const std::string Simulation::getSimState() {
  {
    std::unique_lock<std::mutex> lock(running_mtx_);
    running_cv_.wait(lock, [this] { return is_running_.load(); });
  }

  std::lock_guard<std::mutex> lock(sim_state_.mutex);
  std::string state = sim_state_.state;

  /*volasim_msgs::Odometry odom;*/
  /*odom.ParseFromArray(std::string(state).c_str(), state.length());*/
  /*std::cout << odom.DebugString() << "\n";*/

  return state;
}
