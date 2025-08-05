
/*
 * This example creates an SDL window and renderer, and then draws some
 * rectangles to it every frame.
 *
 * This code is public domain. Feel free to use it for any purpose!
 */

#include <thread>
#define SDL_MAIN_USE_CALLBACKS 1 /* use the callbacks instead of main() */
#include <volasim/comms/zmq_server.h>
#include <volasim/simulation/simulation.h>

#include <GL/glu.h>
#include <GL/glut.h>  // for glutSolidSphere
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_opengl.h>

#include <math.h>
#include <chrono>

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 960
#define FPS 120

// possibly add mutex to a SimState struct if needed in the future
// removes the global variable here...
std::thread t1;

Simulation& sim = Simulation::getInstance(WINDOW_WIDTH, WINDOW_HEIGHT, FPS);
ZMQServer& server = ZMQServer::getInstance();

/* This function runs once at startup. */
SDL_AppResult SDL_AppInit(void** appstate, int argc, char* argv[]) {
  t1 = std::thread([&]() {
    // blocking call to getSimState before loop.
    // ensured sim is running before going to loop
    sim.getSimState();
    auto next_time = std::chrono::steady_clock::now();
    while (sim.isRunning()) {
      server.publishInfo(sim.getSimState());
      next_time =
          std::chrono::steady_clock::now() + std::chrono::microseconds(2000);
      std::this_thread::sleep_until(next_time);

      std::string buffer;
      if (server.receiveInfo(buffer)) {
        sim.setInputs(buffer);
      }
    }
  });
  return sim.initSDL(appstate, argc, argv);
}

/* This function runs when a new event (mouse input, keypresses, etc) occurs. */
SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
  return sim.SDLEvent(appstate, event);
}

/* This function runs once per frame, and is the heart of the program. */
SDL_AppResult SDL_AppIterate(void* appstate) {
  return sim.update(appstate);
}

/* This function runs once at shutdown. */
void SDL_AppQuit(void* appstate, SDL_AppResult result) {
  sim.quitSDL(appstate, result);

  t1.join();
}
