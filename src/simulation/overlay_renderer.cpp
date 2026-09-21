#include <glad/glad.h>

#include <volasim/simulation/entity.h>
#include <volasim/simulation/mesh_primitives.h>
#include <volasim/simulation/overlay_renderer.h>
#include <volasim/simulation/world_buffer.h>

#include <iostream>
#include <queue>
#include <utility>

void OverlayRenderer::init() {
  auto mesh = volasim::primitives::cylinder();

  glGenVertexArrays(1, cylinder_vao_.addr());
  glBindVertexArray(cylinder_vao_.get());

  glGenBuffers(1, cylinder_vbo_.addr());
  glBindBuffer(GL_ARRAY_BUFFER, cylinder_vbo_.get());
  glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(float),
               mesh.vertices.data(), GL_STATIC_DRAW);

  glGenBuffers(1, cylinder_ebo_.addr());
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cylinder_ebo_.get());
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               mesh.indices.size() * sizeof(unsigned int), mesh.indices.data(),
               GL_STATIC_DRAW);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        (void*)(3 * sizeof(float)));

  glBindVertexArray(0);

  cylinder_index_count_ = static_cast<GLsizei>(mesh.indices.size());
}

void OverlayRenderer::buildFrameMap(const Entity& root) {
  frame_map_.clear();

  // BFS over the entity tree, mapping each entity's name to its pointer.
  std::queue<const Entity*> queue;
  queue.push(&root);

  while (!queue.empty()) {
    const Entity* node = queue.front();
    queue.pop();

    if (!node->getName().empty()) {
      frame_map_[node->getName()] = node;
    }

    for (const auto& child : node->children()) {
      queue.push(child.get());
    }
  }
}

void OverlayRenderer::submit(const std::string& topic, TrajectoryData data) {
  std::lock_guard<std::mutex> lock(staging_mtx_);
  staging_[topic] = std::move(data);
}

void OverlayRenderer::absorbStaging() {
  std::lock_guard<std::mutex> lock(staging_mtx_);

  for (auto& [topic, data] : staging_) {
    std::cout << "[overlay] absorb: topic='" << topic
              << "' points=" << data.points.size() << "\n";
    auto it = drawables_.find(topic);
    if (it == drawables_.end()) {
      auto tube = std::make_unique<TubeShape>(cylinder_vao_.get(),
                                              cylinder_index_count_);
      tube->update(std::move(data));
      drawables_[topic] = std::move(tube);
    } else {
      static_cast<TubeShape*>(it->second.get())->update(std::move(data));
    }
  }

  staging_.clear();
}

void OverlayRenderer::draw(const glm::mat4& view, const glm::mat4& proj,
                           Shader& shader, const WorldSnapshot& snapshot) {
  absorbStaging();

  for (auto& [topic, drawable] : drawables_) {
    if (!drawable->visible) {
      continue;
    }

    const std::string& frame_id =
        static_cast<TubeShape*>(drawable.get())->frameId();
    glm::mat4 frame_tf = resolveFrame(frame_id, snapshot);
    drawable->draw(view, proj, frame_tf, shader);
  }
}

glm::mat4 OverlayRenderer::resolveFrame(const std::string&   frame_id,
                                        const WorldSnapshot& snapshot) {
  if (frame_id.empty()) {
    return glm::mat4(1.f);
  }

  auto it = frame_map_.find(frame_id);
  if (it == frame_map_.end()) {
    return glm::mat4(1.f);
  }

  return it->second->getGlobalTransform(snapshot);
}

void OverlayRenderer::setVisible(const std::string& topic, bool visible) {
  auto it = drawables_.find(topic);
  if (it != drawables_.end()) {
    it->second->visible = visible;
  }
}

void OverlayRenderer::setAllVisible(bool visible) {
  for (auto& [topic, drawable] : drawables_) {
    drawable->visible = visible;
  }
}
