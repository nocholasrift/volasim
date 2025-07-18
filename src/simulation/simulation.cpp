#include <GL/glu.h>
#include <GL/glut.h>  // for glutSolidSphere
#include <math.h>
#include <volasim/simulation/simulation.h>
#include <volasim/vehicles/drone.h>

// EventDispatcher& event_handler_ = EventDispatcher::getInstance();
// PhysicsInterface& physics_interface_ = PhysicsInterface::getInstance();

Simulation::Simulation(int win_width, int win_height, int fps) : event_handler_(EventDispatcher::getInstance()), physics_interface_(PhysicsInterface::getInstance()) {

  // event_handler_ = EventDispatcher::getInstance();
  // physics_interface_ = PhysicsInterface::getInstance();

  window_width_ = win_width;
  window_height_ = win_height;

  frames_per_sec_ = fps;

  event_handler_.addEventListener(
      &PhysicsInterface::getInstance(), "OBJ_ADD");
  event_handler_.addEventListener(
      &PhysicsInterface::getInstance(), "OBJ_RM");

  world_ = std::make_unique<DisplayObjectContainer>("world");
  // world_->makeInvisible();

  Eigen::Matrix3d J;
  J << 0.0820, 0., 0.,
       0., 0.0845, 0.,
       0., 0., .1377;
  double mass = 4.34;
  double length = 0.315;
  double c_torque = 8.004e-4;

  std::unique_ptr<DynamicObject<13,4>> drone = std::make_unique<Drone>(J, c_torque, length, mass, 1./static_cast<double>(fps));
  drone->setTranslation(glm::vec3(0., 0., 2.));
  DynamicDisplayWrapper<13, 4>* drone_wrapper = new DynamicDisplayWrapper<13, 4>("drone", drone);
  ShapeMetadata sphere_data;
  sphere_data.scale = 0.3;
  // sphere_data.height = 0.0;
  // sphere_data.slices = 32;
  // sphere_data.stacks = 32;
  drone_wrapper->setRenderable(ShapeType::kCube, sphere_data);
  world_->addChild(drone_wrapper);

  DisplayObject* plane = new DisplayObject("ground plane");
  plane->setRenderable(ShapeType::kPlane, ShapeMetadata());
  world_->addChild(plane);

  // DisplayObject* cube = new DisplayObject("cube");
  // ShapeMetadata cube_data;
  // cube_data.scale = 0.3;
  // cube->setRenderable(ShapeType::kCube, cube_data);
  // cube->setTranslation(glm::vec3(0.5, 0., 0.));
  // sphere->addChild(cube);
}

SDL_AppResult Simulation::initSDL(void** appstate, int argc, char* argv[]) {
  SDL_SetAppMetadata("Vola Simulator", "1.0",
                     "github.com/nocholasrift/volasim");

  glutInit(&argc, argv);

  window_ = SDL_CreateWindow("Floating Sphere", window_width_, window_height_,
                             SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);

  gl_ctx_ = SDL_GL_CreateContext(window_);
  SDL_GL_MakeCurrent(window_, gl_ctx_);
  SDL_GL_SetSwapInterval(1);  // enable vsync

  glEnable(GL_DEPTH_TEST);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  GLfloat light_pos[] = {1.0f, 1.0f, 2.0f, 1.0f};
  glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, 800.0 / 600.0, 0.1, 100.0);

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
  if (event->type == SDL_EVENT_QUIT) {
    return SDL_APP_SUCCESS; /* end the program, reporting success to the OS. */
  }
  return SDL_APP_CONTINUE; /* carry on with the program! */
}

SDL_AppResult Simulation::update(void* appstate) {

  
  Uint64 duration = SDL_GetTicks() - frame_start_;
  if (duration > ms_per_frame_){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(3, 3, 3,   // eye: up and to the side
              0, 0, 0,   // center: look at origin
              0, 0, 1);  // up: Z axis up (assuming Z is vertical)

    world_->draw();
  }

  Uint64 frame_start_ = SDL_GetTicks();

  if (last_step_ < 0){
    last_step_ = SDL_GetTicks();
    return SDL_APP_CONTINUE;
  }

  DynamicDisplayWrapper<13, 4>* drone = 
    (DynamicDisplayWrapper<13, 4>*) world_->getChild("drone");

  double dt = (SDL_GetTicks() - last_step_) / 1000.;
  Eigen::Vector4d u = Eigen::Vector4d::Zero();
  // Eigen::Vector4d u = Eigen::Vector4d(.95,1.05,1.05,.95) * 4.34 * 9.81/4.;
  drone->update(u, dt);

  physics_interface_.update(dt);

  last_step_ = SDL_GetTicks();

  SDL_GL_SwapWindow(window_);


  return SDL_APP_CONTINUE; /* carry on with the program! */
}

void Simulation::quitSDL(void* appstate, SDL_AppResult result) {

  SDL_GL_DestroyContext(gl_ctx_);
  SDL_Quit();
}
