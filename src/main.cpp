
/*
 * This example creates an SDL window and renderer, and then draws some
 * rectangles to it every frame.
 *
 * This code is public domain. Feel free to use it for any purpose!
 */

#include <thread>
#define SDL_MAIN_USE_CALLBACKS 1 /* use the callbacks instead of main() */
#include <volasim/args.h>
#include <volasim/comms/topics.h>
#include <volasim/comms/zmq_server.h>
#include <volasim/sensors/depth_frame.h>
#include <volasim/simulation/simulation.h>

#ifdef USE_APPLE_OPENGL_HEADERS
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#include <GL/glut.h>
#endif
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_opengl.h>

#include <math.h>
#include <chrono>

// possibly add mutex to a SimState struct if needed in the future
// removes the global variable here...
std::thread t1;

Simulation& sim    = Simulation::getInstance();
ZMQServer&  server = ZMQServer::getInstance();

/* This function runs once at startup. */
SDL_AppResult SDL_AppInit(void** appstate, int argc, char* argv[]) {
  Args args;
  try {
    args = parseArgs(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "[volasim] " << e.what() << "\n";
    return SDL_APP_FAILURE;
  }

  t1 = std::thread([&]() {
    // block until the world is loaded and physics is running before publishing
    sim.waitUntilRunning();
    auto next_time = std::chrono::steady_clock::now();
    while (sim.isRunning()) {
      // one message per drone; the topic carries the drone id so subscribers
      // filter by drone/<id>/ in the ZMQ layer
      for (const auto& [drone_id, state_bytes] : sim.getSimState()) {
        server.publishState(volasim::topics::state(drone_id), state_bytes);
      }

      // depth frames arrive at sensor_hz; drain whatever is ready and send the
      // raw uint16 payload zero-copy alongside its header
      for (DepthFrame& frame : sim.drainCloudFrames()) {
        zmq::message_t payload(frame.depth_mm.data(),
                               frame.depth_mm.size() * sizeof(uint16_t));
        server.publishSensor(frame.topic, frame.header, std::move(payload));
      }

      next_time =
          std::chrono::steady_clock::now() + std::chrono::microseconds(2000);
      std::this_thread::sleep_until(next_time);

      std::string buffer;
      if (server.receiveInfo(buffer)) {
        sim.setInputs(buffer);
      }
    }
  });
  return sim.initSDL(appstate, argc, argv, args);
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

  // SDL runs this even when init failed, so the thread may never have started
  if (t1.joinable()) {
    t1.join();
  }
}
