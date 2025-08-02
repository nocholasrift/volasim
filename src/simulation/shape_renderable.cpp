#include <glad/glad.h>

#include <volasim/simulation/shape_renderable.h>

/*#include <GL/gl.h>*/
/*#include <GL/glut.h>*/

#include <array>
#include <iostream>
#include <stdexcept>

ShapeRenderable::ShapeRenderable(const ShapeMetadata& meta) : meta_(meta) {}

ShapeRenderable::~ShapeRenderable() {}

void ShapeRenderable::draw(Shader& shader) {

  // std::cout << "indices in draw: " << meta_.index_count << std::endl;
  shader.setUniformVec3("color", hexToRGB(meta_.color));
  glBindVertexArray(meta_.vao);
  glDrawElements(GL_TRIANGLES, meta_.index_count, GL_UNSIGNED_INT, (void*)0);
  glBindVertexArray(0);
  /*glDrawArrays(GL_TRIANGLE_FAN, 0, 4);*/
}

glm::vec3 ShapeRenderable::hexToRGB(std::string_view hex_str) {
  if (hex_str[0] != '#' || hex_str.length() != 7) {
    std::string err_str =
        "[ShapeRenderable] Invalid color string! " + std::string(hex_str) +
        "\nShould be formatted as 6 hex digits preceeded by #";
    throw std::invalid_argument(err_str);
  }

  glm::vec3 ret;
  std::string_view str_r = hex_str.substr(1, 2);
  std::string_view str_g = hex_str.substr(3, 2);
  std::string_view str_b = hex_str.substr(5, 2);

  std::array<std::string_view, 3> strs = {str_r, str_g, str_b};

  auto hexToInt = [](char c) -> uint8_t {
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    else if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    else if (c >= '0' && c <= '9')
      return c - '0';

    throw std::invalid_argument("[ShapeRenderable] Invalid hex character: " +
                                std::string(1, c));
  };

  // we know there will only ever be 2 hex chars per channel,
  // keep simple impl for now
  int i = 0;
  for (std::string_view hex_str : strs) {
    char h0 = hexToInt(hex_str[0]);
    char h1 = hexToInt(hex_str[1]);
    ret[i++] = static_cast<float>(h0 * 16 + h1) / 255.;
  }

  return ret;
}
