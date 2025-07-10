#include <GL/glu.h>
#include <GL/glut.h>  // for glutSolidSphere
#include <math.h>
#include <volasim/simulation/simulation.h>
#include <volasim/vehicles/drone.h>

Simulation::Simulation(int win_width, int win_height, int fps) {

  window_width_ = win_width;
  window_height_ = win_height;

  frames_per_sec_ = fps;

  world_ = std::make_unique<DisplayObjectContainer>("world");
  world_->makeInvisible();

  DisplayObjectContainer* sphere = new DisplayObjectContainer("sphere");
  ShapeMetadata sphere_data;
  sphere_data.scale = 0.3;
  sphere_data.height = 0.0;
  sphere_data.slices = 32;
  sphere_data.stacks = 32;
  sphere->setRenderable(ShapeType::kSphere, sphere_data);
  world_->addChild(sphere);

  DisplayObject* cube = new DisplayObject("cube");
  ShapeMetadata cube_data;
  cube_data.scale = 0.3;
  cube->setRenderable(ShapeType::kCube, cube_data);
  sphere->addChild(cube);
  cube->setTranslation(glm::vec3(0.5, 0., 0.));
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

  Uint64 frame_start = SDL_GetTicks();

  float x = 0.f;
  float y = sin(time_) * 1.0f;
  float z = cos(time_) * 1.0f;
  time_ += 0.01f;

  DisplayObjectContainer* sphere =
      (DisplayObjectContainer*)world_->getChild("sphere");
  sphere->setTranslation(glm::vec3(y, z, x));

  sphere->getChild("cube")->setRotation(glm::vec3(0., 0., 2 * time_));

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(0, 0, 3, 0, 0, 0, 0, 1, 0);

  world_->draw();

  SDL_GL_SwapWindow(window_);

  Uint64 duration = SDL_GetTicks() - frame_start;
  if (duration < ms_per_frame_)
    SDL_Delay(ms_per_frame_ - duration);

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

void Simulation::quitSDL(void* appstate, SDL_AppResult result) {

  SDL_GL_DestroyContext(gl_ctx_);
  SDL_Quit();
}
