#include <GL/glu.h>
#include <GL/glut.h>  // for glutSolidSphere
#include <math.h>
#include <glm/gtc/type_ptr.hpp>

#include <volasim/simulation/simulation.h>
#include <volasim/simulation/xml_parser.h>
#include <volasim/vehicles/drone.h>
#include <stdexcept>

// EventDispatcher& event_handler_ = EventDispatcher::getInstance();
// PhysicsInterface& physics_interface_ = PhysicsInterface::getInstance();

Simulation::Simulation(int win_width, int win_height, int fps)
    : event_handler_(EventDispatcher::getInstance()),
      physics_interface_(PhysicsInterface::getInstance()) {

  // event_handler_ = EventDispatcher::getInstance();
  // physics_interface_ = PhysicsInterface::getInstance();

  window_width_ = win_width;
  window_height_ = win_height;

  frames_per_sec_ = fps;

  event_handler_.addEventListener(&PhysicsInterface::getInstance(), "OBJ_ADD");
  event_handler_.addEventListener(&PhysicsInterface::getInstance(), "OBJ_RM");

  world_ = std::make_unique<DisplayObjectContainer>("world");
  // world_->makeInvisible();

  Eigen::Matrix3d J;
  J << 0.0820, 0., 0., 0., 0.0845, 0., 0., 0., .1377;
  double mass = 4.34;
  double length = 0.315;
  double c_torque = 8.004e-4;

  DynamicObject* drone =
      new Drone(J, c_torque, length, mass, 1. / static_cast<double>(fps));
  drone->setTranslation(glm::vec3(0., 0., 2.));

  DisplayObject* display_drone = new DisplayObject("drone");
  ShapeMetadata sphere_data;
  sphere_data.radius = 0.3;
  display_drone->setRenderable(ShapeType::kCube, sphere_data);
  display_drone->setTranslation(glm::vec3(0., 0., 2.));

  physics_interface_.preRegister(display_drone, drone);

  world_->addChild(display_drone);

  // DisplayObject* plane = new DisplayObject("ground plane");
  // plane->setRenderable(ShapeType::kPlane, ShapeMetadata());
  // world_->addChild(plane);

  camera_ = Camera(glm::ivec2(win_width, win_height), glm::vec3(3., 3., 7.),
                   glm::vec3(0, 0, 1), -147.0f, -41.0f);

  XMLParser parser("./definitions/worlds/world_250_world.xml");
  std::vector<ShapeMetadata> renderables = parser.getRenderables();

  int i = 0;
  for (ShapeMetadata& settings : renderables) {
    DisplayObject* display_obj = new DisplayObject(settings.name);
    display_obj->setRenderable(settings.type, settings);
    display_obj->setTranslation(settings.pos);
    world_->addChild(display_obj);
  }
}

SDL_AppResult Simulation::initSDL(void** appstate, int argc, char* argv[]) {
  SDL_SetAppMetadata("Vola Simulator", "1.0",
                     "github.com/nocholasrift/volasim");

  glutInit(&argc, argv);

  window_ = SDL_CreateWindow("Floating Sphere", window_width_, window_height_,
                             SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);

  SDL_SetWindowRelativeMouseMode(window_, true);

  gl_ctx_ = SDL_GL_CreateContext(window_);
  SDL_GL_MakeCurrent(window_, gl_ctx_);
  SDL_GL_SetSwapInterval(1);  // enable vsync

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

  glClearColor(0.2f, 0.2f, 0.25f, 1.0f);  // lighter background

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, static_cast<float>(window_width_) / window_height_, 0.1,
                 100.0);

  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  time_ = 0.;
  frame_start_ = -1000000;
  last_step_ = -1;

  ms_per_frame_ = 1000 / frames_per_sec_;

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
    case SDL_EVENT_KEY_DOWN:
      if (event->key.key == SDLK_Q)
        return SDL_APP_SUCCESS;
    default:
      return SDL_APP_CONTINUE;
  }

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

SDL_AppResult Simulation::update(void* appstate) {
  static int count = 0;

  Uint64 duration = SDL_GetTicks() - frame_start_;
  if (duration > ms_per_frame_) {
    // physics_interface_.update(ms_per_frame_ / 1000.);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(camera_.zoom_,
                   static_cast<float>(window_width_) / window_height_, 0.1,
                   100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glLoadMatrixf(glm::value_ptr(camera_.getViewMatrix()));
    // gluLookAt(3, 3, 3,   // eye: up and to the side
    //           0, 0, 0,   // center: look at origin
    //           0, 0, 1);  // up: Z axis up (assuming Z is vertical)

    world_->draw();
  }

  Uint64 frame_start_ = SDL_GetTicks();

  if (last_step_ < 0) {
    last_step_ = SDL_GetTicks();
    return SDL_APP_CONTINUE;
  }

  // DynamicDisplayWrapper<13, 4>* drone =
  //   (DynamicDisplayWrapper<13, 4>*) world_->getChild("drone");

  double dt = (SDL_GetTicks() - last_step_) / 1000.;
  Eigen::Vector4d u = Eigen::Vector4d::Zero();
  // Eigen::Vector4d u = Eigen::Vector4d(.95,1.05,1.05,.95) * 4.34 * 9.81/4.;
  // drone->update(u, dt);
  physics_interface_.update(ms_per_frame_ / 1000.);

  // physics_interface_.update(dt);

  last_step_ = SDL_GetTicks();

  SDL_GL_SwapWindow(window_);

  // if (count++ > 2)
  //   exit(0);

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

void Simulation::quitSDL(void* appstate, SDL_AppResult result) {

  SDL_GL_DestroyContext(gl_ctx_);
  SDL_Quit();
}
