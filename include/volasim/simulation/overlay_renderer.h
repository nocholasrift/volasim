#ifndef VOLASIM_OVERLAY_RENDERER_H
#define VOLASIM_OVERLAY_RENDERER_H

#include <volasim/simulation/gl_resource.h>
#include <volasim/simulation/overlay_drawable.h>
#include <volasim/simulation/shader.h>
#include <volasim/simulation/tube_shape.h>

#include <glm/glm.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

class Entity;
class WorldSnapshot;

class OverlayRenderer {
 public:
  void init();

  void buildFrameMap(const Entity& root);

  // Thread-safe. Comms thread writes latest data per topic.
  void submit(const std::string& topic, TrajectoryData data);

  // Render thread only. Absorbs dirty staging entries, then draws.
  void draw(const glm::mat4& view, const glm::mat4& proj, Shader& shader,
            const WorldSnapshot& snapshot);

  void setVisible(const std::string& topic, bool visible);
  void setAllVisible(bool visible);

 private:
  void      absorbStaging();
  glm::mat4 resolveFrame(const std::string&   frame_id,
                         const WorldSnapshot& snapshot);

  std::mutex                                      staging_mtx_;
  std::unordered_map<std::string, TrajectoryData> staging_;

  std::unordered_map<std::string, std::unique_ptr<OverlayDrawable>> drawables_;
  // frame_id stored per-drawable via TubeShape::frameId()

  std::unordered_map<std::string, const Entity*> frame_map_;

  GLResource<VaoDeleter>    cylinder_vao_;
  GLResource<BufferDeleter> cylinder_vbo_;
  GLResource<BufferDeleter> cylinder_ebo_;
  GLsizei                   cylinder_index_count_{0};
};

#endif
