#ifndef VOLASIM_GUI_H
#define VOLASIM_GUI_H

#include <SDL3/SDL.h>

#include <vector>

struct ImFont;
struct ImVec2;

// The GUI sees the sim only through the plain structs below, which the sim
// rebuilds each frame. It never includes a sim header and the sim never
// includes ImGui, so panels can be added here without touching sim code.
//
// Everything in this module runs on the render thread, which also owns every
// flag and texture the structs point at, so no locking is needed.
namespace volasim::gui {

// Strings are borrowed from the sim and only need to outlive the draw() call.

// Something the layers panel can switch on and off. `visible` points at the
// drawable's own flag, so a toggle lands on the very next draw.
struct Layer {
  const char* group;
  const char* name;
  bool*       visible;
};

// An offscreen image the sim renders on request. `open` is set while the
// feed's window is showing; the sim skips rendering closed feeds.
struct Feed {
  const char*  name;
  unsigned int texture;  // GL texture id
  int          width;
  int          height;
  bool*        open;
};

class Gui {
 public:
  // Needs the GL context current. Returns false if a backend fails to start.
  bool init(SDL_Window* window, SDL_GLContext context);
  void shutdown();

  // Forwards the event to ImGui. Returns true when the GUI owns it and the
  // scene (camera orbit, hotkeys) should ignore it.
  bool handleEvent(const SDL_Event& event);

  // Draws over whatever is already in the back buffer; call just before swap.
  void draw(const std::vector<Layer>& layers, const std::vector<Feed>& feeds);

 private:
  // The pull tab plus, while it is out, the sliding bar it opens.
  void drawLayersPanel(const std::vector<Layer>& layers,
                       const std::vector<Feed>&  feeds);
  void drawLayersBar(const ImVec2& pos, float height,
                     const std::vector<Layer>& layers,
                     const std::vector<Feed>&  feeds);
  void drawFeedWindows(const std::vector<Feed>& feeds);

  bool    initialized_{false};
  bool    panel_open_{false};
  float   panel_slide_{0.F};  // 0 = tucked away, 1 = fully out
  ImFont* bold_font_{nullptr};
};

}  // namespace volasim::gui

#endif
