# volasim

Quadrotor flight simulator. Renders a physics-driven world via OpenGL, communicates state to a ROS node over ZMQ.

## Build & Run

```bash
cmake -B build && cmake --build build -j$(nproc)
./build/volasim          # must run from project root — asset paths are relative
```


Assets are copied to the build directory by the `copy_assets` CMake target; definitions live in `definitions/`.

## Architecture

**Singletons** (use `getInstance()`): `Simulation`, `PhysicsInterface`, `EventDispatcher`

**Render loop**: `Simulation` owns the SDL3 window, OpenGL context, camera list, and scene graph. The loop runs in `Simulation::update`.

**Scene graph**: `DisplayObjectContainer` → `DisplayObject` → `Renderable` (`ShapeRenderable` for primitives, `MeshRenderable` for OBJ meshes).

**Physics**: `PhysicsInterface` wraps JoltPhysics. Dynamic objects register via `OBJ_ADD`/`OBJ_RM` events through `EventDispatcher`.

**Cameras**: `Simulation` holds a `std::vector<Camera> cameras_` and `active_camera` index. `Camera::fromXML` owns all XML field knowledge (`cam_yaw`, `cam_pitch`, `cam_distance`, `fov_deg`, `fps`, `window_width`, `window_height`). `XMLParser::loadCamera` navigates to the `<gui>` node and delegates to `Camera::fromXML` — the parser never reads camera field names directly.

**Sensors**: `GPUSensor` / `DepthSensor` are GPU-based depth cameras attached to a `DynamicObject`.

**Comms**: ZMQ server in `src/comms/` bridges sim state to the ROS node. `volasim_node/` is a separate ROS1/ROS2 package built with colcon/catkin.

## File Layout

```
include/volasim/          — all headers (mirrors src/)
src/                      — simulator sources
definitions/worlds/       — XML world definitions
definitions/meshes/       — OBJ mesh files
volasim_node/             — ROS bridge node (separate build)
third_party/              — SDL3, JoltPhysics, pugixml, cppzmq, assimp
```

## Key Design Decisions

- `Camera::fromXML` is a static factory so XML field names are encapsulated in `Camera`, not the parser. When multi-camera support is added, `XMLParser::loadCamera` becomes `loadCameras()` returning `std::vector<Camera>`, and cycling is `active_camera = (active_camera + 1) % cameras_.size()`.
- XML includes (in-progress): use iterative DFS with a `std::unordered_set<std::string>` visited set rather than recursion. Parse included files immediately on encounter; merge their definitions into `renderables_` / `class_settings_`.
- `pugi::xml_node` is forward-declared in `camera.h` (not included) to avoid pulling pugixml into everything that includes `camera.h`.

## Platform

macOS: `USE_APPLE_OPENGL_HEADERS` flag switches to `GLUT/glut.h` / `OpenGL/glu.h`. `GL_SILENCE_DEPRECATION` and `GLUT_SILENCE_DEPRECATION` suppress Apple deprecation warnings.
