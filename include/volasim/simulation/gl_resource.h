#ifndef GLRESOURCE_H
#define GLRESOURCE_H

#include <glad/glad.h>

// Deleters for the GL object kinds we manage.
struct FboDeleter {
  void operator()(GLuint id) const { glDeleteFramebuffers(1, &id); }
};
struct TexDeleter {
  void operator()(GLuint id) const { glDeleteTextures(1, &id); }
};
struct VaoDeleter {
  void operator()(GLuint id) const { glDeleteVertexArrays(1, &id); }
};
struct BufferDeleter {
  void operator()(GLuint id) const { glDeleteBuffers(1, &id); }
};

// RAII owner for a single GL object handle. Move-only; a moved-from instance is
// left holding 0 so its destructor is a no-op. Lets owners stay rule-of-zero.
template <class Deleter>
class GLResource {
 public:
  GLResource() = default;
  ~GLResource() { reset(); }

  GLResource(GLResource&& other) noexcept : id_(other.id_) { other.id_ = 0; }
  GLResource& operator=(GLResource&& other) noexcept {
    if (this != &other) {
      reset();
      id_       = other.id_;
      other.id_ = 0;
    }
    return *this;
  }

  GLResource(const GLResource&)            = delete;
  GLResource& operator=(const GLResource&) = delete;

  GLuint  get() const { return id_; }
  GLuint* addr() { return &id_; }  // for glGen*(1, x.addr())
  void    reset() {
    if (id_) {
      Deleter{}(id_);
      id_ = 0;
    }
  }
  explicit operator bool() const { return id_ != 0; }

 private:
  GLuint id_ = 0;
};

#endif
