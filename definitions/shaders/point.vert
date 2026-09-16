#version 330 core
uniform sampler2D depth_tex;
uniform mat4 sensor_inv_vp;
uniform mat4 main_mvp;
uniform vec2 sensor_size;
void main() {
  int j = gl_VertexID % int(sensor_size.x);
  int row = gl_VertexID / int(sensor_size.x); // row 0 = bottom (OpenGL convention)
  float px = float(j) + 0.5; // pixel center
  float py = float(row) + 0.5;
  vec2 uv = vec2(px / sensor_size.x, py / sensor_size.y);
  float d = texture(depth_tex, uv).r;
  if (d >= 1.0) {
    gl_Position = vec4(0.0, 0.0, 2.0, 1.0); // behind far plane — clipped
    gl_PointSize = 0.0;
    return;
  }
  float x_ndc = 2.0 * px / sensor_size.x - 1.0;
  float y_ndc = 2.0 * py / sensor_size.y - 1.0;
  float z_ndc = 2.0 * d - 1.0;
  vec4 world_pos = sensor_inv_vp * vec4(x_ndc, y_ndc, z_ndc, 1.0);
  world_pos /= world_pos.w;
  gl_Position = main_mvp * world_pos;
  gl_PointSize = 1.0;
}
