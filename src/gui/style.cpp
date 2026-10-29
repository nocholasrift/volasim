#include "style.h"

#include <imgui.h>

#include <filesystem>
#include <string>

namespace volasim::gui {

namespace {

// Logical pixels; ImGui rasterizes at the display's density, so text stays
// crisp on Retina.
constexpr float kFontSize = 14.F;

ImFont* loadFont(const std::string& path) {
  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) {
    return nullptr;
  }
  return ImGui::GetIO().Fonts->AddFontFromFileTTF(path.c_str(), kFontSize);
}

}  // namespace

Fonts loadFonts(const char* dir) {
  const std::string base(dir);

  Fonts fonts;
  fonts.regular = loadFont(base + "/Inter-Regular.ttf");
  if (fonts.regular == nullptr) {
    fonts.regular = ImGui::GetIO().Fonts->AddFontDefault();
  }

  fonts.bold = loadFont(base + "/Inter-SemiBold.ttf");
  if (fonts.bold == nullptr) {
    fonts.bold = fonts.regular;
  }

  ImGui::GetIO().FontDefault = fonts.regular;
  return fonts;
}

void applyStyle(Theme theme) {
  ImGuiStyle& style = ImGui::GetStyle();

  // Shape: rounded, airy, and chrome-light, like a macOS HUD panel.
  style.FontSizeBase             = kFontSize;
  style.WindowRounding           = 12.F;
  style.ChildRounding            = 8.F;
  style.FrameRounding            = 6.F;
  style.PopupRounding            = 10.F;
  style.ScrollbarRounding        = 8.F;
  style.GrabRounding             = 6.F;
  style.TabRounding              = 6.F;
  style.WindowPadding            = ImVec2(12.F, 12.F);
  style.FramePadding             = ImVec2(8.F, 4.F);
  style.ItemSpacing              = ImVec2(8.F, 6.F);
  style.ItemInnerSpacing         = ImVec2(6.F, 4.F);
  style.WindowBorderSize         = 1.F;
  style.PopupBorderSize          = 1.F;
  style.FrameBorderSize          = 0.F;
  style.ScrollbarSize            = 10.F;
  style.WindowTitleAlign         = ImVec2(0.5F, 0.5F);
  style.WindowMenuButtonPosition = ImGuiDir_None;

  const bool dark = theme == Theme::kDark;

  // Most chrome is an ink colour at a low alpha, so it reads as a tint of
  // whatever is behind it. The light theme's ink is a warm brown so hovers and
  // fills stay creamy rather than going grey.
  const ImVec4 ink =
      dark ? ImVec4(1.F, 1.F, 1.F, 1.F) : ImVec4(0.30F, 0.22F, 0.12F, 1.F);
  auto tint = [ink](float alpha) {
    return ImVec4(ink.x, ink.y, ink.z, alpha);
  };

  // systemBlue: #0A84FF in dark mode, #007AFF in light
  const ImVec4 accent =
      dark ? ImVec4(0.04F, 0.52F, 1.F, 1.F) : ImVec4(0.F, 0.48F, 1.F, 1.F);
  auto accent_a = [accent](float alpha) {
    return ImVec4(accent.x, accent.y, accent.z, alpha);
  };

  // light is a warm cream (#FAF6ED) rather than a cool grey
  const ImVec4 window       = dark ? ImVec4(0.12F, 0.12F, 0.13F, 0.82F)
                                   : ImVec4(0.98F, 0.965F, 0.93F, 0.94F);
  const ImVec4 popup        = dark ? ImVec4(0.14F, 0.14F, 0.15F, 0.96F)
                                   : ImVec4(0.99F, 0.98F, 0.955F, 0.98F);
  const ImVec4 title_active = dark ? ImVec4(0.16F, 0.16F, 0.17F, 0.90F)
                                   : ImVec4(0.95F, 0.93F, 0.89F, 0.96F);

  ImVec4* c                = style.Colors;
  c[ImGuiCol_Text]         = dark ? ImVec4(0.92F, 0.92F, 0.94F, 1.F)
                                  : ImVec4(0.17F, 0.15F, 0.13F, 1.F);
  c[ImGuiCol_TextDisabled] = dark ? ImVec4(0.56F, 0.56F, 0.59F, 1.F)
                                  : ImVec4(0.55F, 0.51F, 0.46F, 1.F);

  c[ImGuiCol_WindowBg]     = window;
  c[ImGuiCol_ChildBg]      = ImVec4(0.F, 0.F, 0.F, 0.F);
  c[ImGuiCol_PopupBg]      = popup;
  c[ImGuiCol_Border]       = tint(dark ? 0.10F : 0.12F);
  c[ImGuiCol_BorderShadow] = ImVec4(0.F, 0.F, 0.F, 0.F);

  c[ImGuiCol_FrameBg]        = tint(dark ? 0.08F : 0.11F);
  c[ImGuiCol_FrameBgHovered] = tint(dark ? 0.12F : 0.15F);
  c[ImGuiCol_FrameBgActive]  = tint(dark ? 0.16F : 0.19F);

  c[ImGuiCol_TitleBg]          = window;
  c[ImGuiCol_TitleBgActive]    = title_active;
  c[ImGuiCol_TitleBgCollapsed] = window;
  c[ImGuiCol_MenuBarBg]        = window;

  c[ImGuiCol_ScrollbarBg]          = ImVec4(0.F, 0.F, 0.F, 0.F);
  c[ImGuiCol_ScrollbarGrab]        = tint(0.20F);
  c[ImGuiCol_ScrollbarGrabHovered] = tint(0.30F);
  c[ImGuiCol_ScrollbarGrabActive]  = tint(0.40F);

  // macOS checkboxes: a solid blue box with a white tick
  c[ImGuiCol_CheckMark]          = ImVec4(1.F, 1.F, 1.F, 1.F);
  c[ImGuiCol_CheckboxSelectedBg] = accent;
  c[ImGuiCol_SliderGrab]         = accent;
  c[ImGuiCol_SliderGrabActive]   = accent_a(0.85F);

  c[ImGuiCol_Button]        = tint(dark ? 0.10F : 0.07F);
  c[ImGuiCol_ButtonHovered] = tint(dark ? 0.16F : 0.11F);
  c[ImGuiCol_ButtonActive]  = accent_a(0.80F);

  c[ImGuiCol_Header]        = tint(dark ? 0.06F : 0.05F);
  c[ImGuiCol_HeaderHovered] = tint(dark ? 0.10F : 0.08F);
  c[ImGuiCol_HeaderActive]  = tint(dark ? 0.14F : 0.11F);

  c[ImGuiCol_Separator]        = tint(0.10F);
  c[ImGuiCol_SeparatorHovered] = accent_a(0.60F);
  c[ImGuiCol_SeparatorActive]  = accent;

  c[ImGuiCol_ResizeGrip]        = ImVec4(0.F, 0.F, 0.F, 0.F);
  c[ImGuiCol_ResizeGripHovered] = accent_a(0.50F);
  c[ImGuiCol_ResizeGripActive]  = accent_a(0.80F);

  c[ImGuiCol_Tab]                       = tint(0.04F);
  c[ImGuiCol_TabHovered]                = tint(0.12F);
  c[ImGuiCol_TabSelected]               = tint(dark ? 0.12F : 0.09F);
  c[ImGuiCol_TabSelectedOverline]       = accent;
  c[ImGuiCol_TabDimmed]                 = tint(0.03F);
  c[ImGuiCol_TabDimmedSelected]         = tint(0.08F);
  c[ImGuiCol_TabDimmedSelectedOverline] = ImVec4(0.F, 0.F, 0.F, 0.F);

  c[ImGuiCol_DockingPreview] = accent_a(0.35F);
  c[ImGuiCol_DockingEmptyBg] = ImVec4(0.F, 0.F, 0.F, 0.F);

  c[ImGuiCol_TextSelectedBg] = accent_a(0.35F);
  c[ImGuiCol_TextLink]       = accent;
  c[ImGuiCol_NavCursor]      = accent;
  c[ImGuiCol_DragDropTarget] = accent;
}

}  // namespace volasim::gui
