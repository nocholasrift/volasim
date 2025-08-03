
/*
 * This example creates an SDL window and renderer, and then draws some
 * rectangles to it every frame.
 *
 * This code is public domain. Feel free to use it for any purpose!
 */

#define SDL_MAIN_USE_CALLBACKS 1 /* use the callbacks instead of main() */
#include <volasim/simulation/simulation.h>
#include <volasim/simulation/zmq_server.h>

#include <GL/glu.h>
#include <GL/glut.h>  // for glutSolidSphere
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_opengl.h>
#include <math.h>

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 960
#define FPS 120

Simulation& sim = Simulation::getInstance(WINDOW_WIDTH, WINDOW_HEIGHT, FPS);
ZMQServer& server = ZMQServer::getInstance();

/* This function runs once at startup. */
SDL_AppResult SDL_AppInit(void** appstate, int argc, char* argv[]) {
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
}
