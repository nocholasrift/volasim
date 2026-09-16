#version 330 core
in vec2 v_uv;
uniform sampler2D depth_tex;
uniform vec2 z_range; // x = near, y = far
out uint depth_mm;
void main() {
  float d = texture(depth_tex, v_uv).r;
  if (d >= 1.0) {
    depth_mm = 0u; // background / no return
    return;
  }
  float z_ndc = 2.0 * d - 1.0;
  float zn = z_range.x;
  float zf = z_range.y;
  float depth_m = (2.0 * zn * zf) / (zf + zn - z_ndc * (zf - zn));
  depth_mm = uint(clamp(depth_m * 1000.0, 0.0, 65535.0));
}
