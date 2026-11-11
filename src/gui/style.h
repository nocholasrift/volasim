#ifndef VOLASIM_GUI_STYLE_H
#define VOLASIM_GUI_STYLE_H

// Private to the GUI module: a macOS-flavoured look (rounded floating panels,
// translucent backgrounds, system-blue accent) in light and dark variants.

struct ImFont;

namespace volasim::gui {

// kDark is the default; kLight (cream) is kept for a future toggle.
enum class Theme { kDark, kLight };

struct Fonts {
  ImFont* regular{nullptr};
  ImFont* bold{nullptr};  // falls back to regular when the file is missing
};

// Loads Inter from `dir`, or ImGui's built-in font when it can't be found.
Fonts loadFonts(const char* dir);

void applyStyle(Theme theme);

}  // namespace volasim::gui

#endif
