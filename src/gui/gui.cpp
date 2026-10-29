#include "style.h"

#include <volasim/gui/gui.h>

#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl3.h>

#include <algorithm>
#include <cstring>

namespace volasim::gui {

namespace {

// Fonts live with the other assets, relative to the working directory.
constexpr const char* kFontDir = "./definitions/fonts";

constexpr float kPanelWidth = 280.F;

// Gap between floating windows and the edges of the screen.
constexpr float kMargin = 12.F;

// The pull tab that opens and closes the layers panel. It sits on the left
// edge when the panel is tucked away and rides the panel's edge as it slides.
constexpr float kHandleWidth  = 24.F;
constexpr float kHandleHeight = 48.F;
constexpr float kHandleGap    = 8.F;

constexpr float kSlideSeconds = 0.18F;

// On-screen width a feed window opens at; it can be resized from there.
constexpr float kFeedWidth = 400.F;

// U+203A / U+2039, single guillemets for the pull tab's open and close arrows
constexpr const char* kOpenArrow  = "\xE2\x80\xBA";
constexpr const char* kCloseArrow = "\xE2\x80\xB9";

// Moves `value` toward `target` by at most `step`.
float approach(float value, float target, float step) {
  return value < target ? std::min(target, value + step)
                        : std::max(target, value - step);
}

// A collapsing section styled like a macOS sidebar heading: small, bold and
// muted, with no bar behind it until hovered.
bool sectionHeader(const char* label, ImFont* bold) {
  const ImGuiStyle& style = ImGui::GetStyle();

  ImGui::PushFont(bold, style.FontSizeBase * 0.9F);
  ImGui::PushStyleColor(ImGuiCol_Text, style.Colors[ImGuiCol_TextDisabled]);
  ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.F, 0.F, 0.F, 0.F));
  const bool open =
      ImGui::CollapsingHeader(label, ImGuiTreeNodeFlags_DefaultOpen);
  ImGui::PopStyleColor(2);
  ImGui::PopFont();
  return open;
}

}  // namespace

bool Gui::init(SDL_Window* window, SDL_GLContext context) {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  ImGuiIO& io     = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

  bold_font_ = loadFonts(kFontDir).bold;
  applyStyle(Theme::kDark);

  if (!ImGui_ImplSDL3_InitForOpenGL(window, context)) {
    ImGui::DestroyContext();
    return false;
  }

  // nullptr lets the backend pick a GLSL version that suits the platform
  if (!ImGui_ImplOpenGL3_Init(nullptr)) {
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    return false;
  }

  initialized_ = true;
  return true;
}

void Gui::shutdown() {
  if (!initialized_) {
    return;
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL3_Shutdown();
  ImGui::DestroyContext();
  initialized_ = false;
}

bool Gui::handleEvent(const SDL_Event& event) {
  if (!initialized_) {
    return false;
  }

  ImGui_ImplSDL3_ProcessEvent(&event);

  const ImGuiIO& io = ImGui::GetIO();
  switch (event.type) {
    case SDL_EVENT_MOUSE_MOTION:
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
    case SDL_EVENT_MOUSE_WHEEL:
      return io.WantCaptureMouse;
    case SDL_EVENT_KEY_DOWN:
    case SDL_EVENT_KEY_UP:
    case SDL_EVENT_TEXT_INPUT:
      return io.WantCaptureKeyboard;
    default:
      // Button-up always reaches the scene, so a camera orbit that started
      // there can't get stuck on when the release lands over a window.
      return false;
  }
}

void Gui::draw(const std::vector<Layer>& layers,
               const std::vector<Feed>&  feeds) {
  if (!initialized_) {
    return;
  }

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplSDL3_NewFrame();
  ImGui::NewFrame();

  // Lets feed windows dock to the screen edges while the scene stays visible
  // (and clickable) through the empty middle.
  ImGui::DockSpaceOverViewport(0, nullptr,
                               ImGuiDockNodeFlags_PassthruCentralNode);

  drawLayersPanel(layers, feeds);
  drawFeedWindows(feeds);

  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void Gui::drawLayersPanel(const std::vector<Layer>& layers,
                          const std::vector<Feed>&  feeds) {
  const ImGuiViewport*       viewport = ImGui::GetMainViewport();
  constexpr ImGuiWindowFlags kPinned =
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDocking |
      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar |
      ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse;

  // Slide the bar in from off-screen with an ease-out, so it decelerates into
  // place instead of popping.
  panel_slide_          = approach(panel_slide_, panel_open_ ? 1.F : 0.F,
                                   ImGui::GetIO().DeltaTime / kSlideSeconds);
  const float remaining = 1.F - panel_slide_;
  const float eased     = 1.F - (remaining * remaining * remaining);

  const float hidden_x = viewport->WorkPos.x - kPanelWidth;
  const float shown_x  = viewport->WorkPos.x + kMargin;
  const float panel_x  = hidden_x + (eased * (shown_x - hidden_x));

  if (panel_slide_ > 0.F) {
    drawLayersBar(ImVec2(panel_x, viewport->WorkPos.y + kMargin),
                  viewport->WorkSize.y - (2.F * kMargin), layers, feeds);
  }

  // The pull tab rides the bar's right edge; with the bar tucked away that
  // puts it just in from the left edge of the screen.
  const ImGuiStyle& style = ImGui::GetStyle();
  ImGui::SetNextWindowPos(ImVec2(
      panel_x + kPanelWidth + kHandleGap,
      viewport->WorkPos.y + ((viewport->WorkSize.y - kHandleHeight) * 0.5F)));
  ImGui::SetNextWindowSize(ImVec2(kHandleWidth, kHandleHeight));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.F, 0.F));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.F);
  ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 8.F);
  // ImGui's default 32px minimum would leave the tab wider than its button.
  ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize,
                      ImVec2(kHandleWidth, kHandleHeight));
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.F, 0.F, 0.F, 0.F));
  ImGui::Begin("##layers_handle", nullptr,
               kPinned | ImGuiWindowFlags_NoScrollbar |
                   ImGuiWindowFlags_NoScrollWithMouse);
  ImGui::PushFont(bold_font_, style.FontSizeBase * 1.4F);
  if (ImGui::Button(panel_open_ ? kCloseArrow : kOpenArrow,
                    ImVec2(kHandleWidth, kHandleHeight))) {
    panel_open_ = !panel_open_;
  }
  ImGui::PopFont();
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(panel_open_ ? "Hide layers" : "Show layers");
  }
  ImGui::End();
  ImGui::PopStyleColor();
  ImGui::PopStyleVar(4);
}

void Gui::drawLayersBar(const ImVec2& pos, float height,
                        const std::vector<Layer>& layers,
                        const std::vector<Feed>&  feeds) {
  ImGui::SetNextWindowPos(pos);
  ImGui::SetNextWindowSize(ImVec2(kPanelWidth, height));
  ImGui::Begin("Layers", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDocking |
                   ImGuiWindowFlags_NoSavedSettings |
                   ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoCollapse);

  ImGui::PushFont(bold_font_, ImGui::GetStyle().FontSizeBase * 1.15F);
  ImGui::TextUnformatted("Layers");
  ImGui::PopFont();
  ImGui::Spacing();

  // Groups keep the order they first appear in; entries within a group are
  // sorted so they don't reshuffle as the sim's containers rehash.
  std::vector<const char*> groups;
  for (const Layer& layer : layers) {
    const bool seen =
        std::any_of(groups.begin(), groups.end(), [&](const char* group) {
          return std::strcmp(group, layer.group) == 0;
        });
    if (!seen) {
      groups.push_back(layer.group);
    }
  }

  std::vector<const Layer*> members;
  for (const char* group : groups) {
    ImGui::PushID(group);

    if (sectionHeader(group, bold_font_)) {
      members.clear();
      for (const Layer& layer : layers) {
        if (std::strcmp(layer.group, group) == 0) {
          members.push_back(&layer);
        }
      }
      std::sort(members.begin(), members.end(),
                [](const Layer* a, const Layer* b) {
                  return std::strcmp(a->name, b->name) < 0;
                });

      for (const Layer* layer : members) {
        ImGui::Checkbox(layer->name, layer->visible);
      }

      if (ImGui::SmallButton("Show all")) {
        for (const Layer* layer : members) {
          *layer->visible = true;
        }
      }
      ImGui::SameLine();
      if (ImGui::SmallButton("Hide all")) {
        for (const Layer* layer : members) {
          *layer->visible = false;
        }
      }
      ImGui::Spacing();
    }

    ImGui::PopID();
  }

  ImGui::PushID("Cameras");
  if (sectionHeader("Cameras", bold_font_)) {
    if (feeds.empty()) {
      ImGui::TextDisabled("No sensors in this world");
    }
    for (const Feed& feed : feeds) {
      ImGui::Checkbox(feed.name, feed.open);
    }
  }
  ImGui::PopID();

  ImGui::End();
}

void Gui::drawFeedWindows(const std::vector<Feed>& feeds) {
  const ImGuiViewport* viewport = ImGui::GetMainViewport();

  int slot = 0;
  for (const Feed& feed : feeds) {
    if (!*feed.open) {
      continue;
    }

    const float aspect =
        static_cast<float>(feed.height) / static_cast<float>(feed.width);

    // First open stacks feeds down the right edge; after that ImGui remembers
    // wherever the window was moved or docked.
    const ImVec2 size(kFeedWidth,
                      (kFeedWidth * aspect) + ImGui::GetFrameHeight());
    ImGui::SetNextWindowSize(size, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(
        ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - size.x - kMargin,
               viewport->WorkPos.y + kMargin + slot * (size.y + kMargin)),
        ImGuiCond_FirstUseEver);
    ++slot;

    // No padding: the image runs edge to edge under the title bar, and the
    // window's own rounded corners frame it.
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.F, 0.F));
    if (ImGui::Begin(feed.name, feed.open,
                     ImGuiWindowFlags_NoScrollbar |
                         ImGuiWindowFlags_NoScrollWithMouse)) {
      // Letterbox so a resized window never stretches the image.
      const ImVec2 avail = ImGui::GetContentRegionAvail();
      float        w     = avail.x;
      float        h     = w * aspect;
      if (h > avail.y) {
        h = avail.y;
        w = h / aspect;
      }

      const ImVec2 origin = ImGui::GetCursorScreenPos();
      const ImVec2 p_min(origin.x + ((avail.x - w) * 0.5F),
                         origin.y + ((avail.y - h) * 0.5F));
      const ImVec2 p_max(p_min.x + w, p_min.y + h);

      // GL textures start at the bottom-left, so flip V to show it upright.
      ImGui::GetWindowDrawList()->AddImageRounded(
          static_cast<ImTextureID>(feed.texture), p_min, p_max,
          ImVec2(0.F, 1.F), ImVec2(1.F, 0.F), IM_COL32_WHITE,
          ImGui::GetStyle().WindowRounding, ImDrawFlags_RoundCornersBottom);
      ImGui::Dummy(avail);
    }
    ImGui::End();
    ImGui::PopStyleVar();
  }
}

}  // namespace volasim::gui
