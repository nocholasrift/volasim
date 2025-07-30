#ifndef VOLASIM_SHADER_H
#define VOLASIM_SHADER_H

// inspired by gl_depth_sim shader_program.cpp

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <Eigen/Core>

// #include <stdexcept>
// #include <string>

class Shader {
 public:
  Shader() = default;
  Shader(const std::string& vertex_shader, const std::string& fragment_shader) {

    unsigned int vertex, fragment;
    int success;
    char infoLog[512];

    vertex = glCreateShader(GL_VERTEX_SHADER);
    const char* vertex_source = vertex_shader.c_str();
    glShaderSource(vertex, 1, &vertex_source, NULL);
    glCompileShader(vertex);

    glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);

    if (!success) {
      glGetShaderInfoLog(vertex, 512, NULL, infoLog);
      throw std::runtime_error(infoLog);
    }

    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    const char* fragment_source = fragment_shader.c_str();
    glShaderSource(fragment, 1, &fragment_source, NULL);
    glCompileShader(fragment);

    glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);

    if (!success) {
      glGetShaderInfoLog(fragment, 512, NULL, infoLog);
      throw std::runtime_error(infoLog);
    }

    // link shaders together
    id_ = glCreateProgram();
    glAttachShader(id_, vertex);
    glAttachShader(id_, fragment);
    glLinkProgram(id_);

    // check for error
    glGetProgramiv(id_, GL_LINK_STATUS, &success);
    if (!success) {
      glGetProgramInfoLog(id_, 512, NULL, infoLog);
      throw std::runtime_error(infoLog);
    }

    glDeleteShader(vertex);
    glDeleteShader(fragment);
  }

  unsigned int getID() { return id_; }

  void setUniformMat4(const std::string& attr, const glm::mat4& mat) {
    GLuint loc = glGetUniformLocation(id_, attr.c_str());
    glUniformMatrix4fv(loc, 1, GL_FALSE, &mat[0][0]);
  }

  void setUniformVec3(const std::string& attr, const glm::vec3& vec) {
    GLuint loc = glGetUniformLocation(id_, attr.c_str());
    glUniformMatrix4fv(loc, 1, GL_FALSE, &vec[0]);
  }

 private:
  unsigned int id_;
};

#endif
