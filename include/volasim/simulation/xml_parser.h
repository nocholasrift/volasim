#ifndef XMLPARSER_H
#define XMLPARSER_H

#include <pugixml.hpp>

#include <volasim/simulation/camera.h>
#include <volasim/simulation/display_object_container.h>
#include <volasim/simulation/shape_renderable.h>

#include <memory>
#include <string_view>
#include <unordered_map>
#include <vector>

enum class XMLTags {
  kGUI = 0,
  kElement,
  kBlockDefinition,
  kBlock,
  kVehicle,
};

class XMLParser {
 public:
  XMLParser() = delete;
  XMLParser(const std::string& fname);

  void loadWorldFromXML(DisplayObjectContainer* world);

  CameraSettings getCameraSettings();

 private:
  void throwError(std::string_view fname, const pugi::xml_parse_result& result);

  void generateShapeBuffers(ShapeMetadata& settings,
                            const pugi::xml_node& geometry_node);

  void handleVehicle(const pugi::xml_node& item, DisplayObjectContainer* world);

  void handleElement(const pugi::xml_node& item, DisplayObjectContainer* world);

  void handleBlockDefinition(const pugi::xml_node& item);

  void handleBlock(const pugi::xml_node& item, DisplayObjectContainer* world);

  void createAndAddRenderable(const std::string& name,
                              const ShapeMetadata& settings,
                              const glm::vec3& pos,
                              DisplayObjectContainer* world);

 private:
  pugi::xml_document doc_;

  std::unordered_map<std::string, int> defined_class_count_;
  std::unordered_map<std::string, ShapeMetadata> class_settings_;

  std::unordered_map<std::string_view, XMLTags> type_map_ = {
      {"element", XMLTags::kElement},
      {"block:class", XMLTags::kBlockDefinition},
      {"block", XMLTags::kBlock},
      {"vehicle", XMLTags::kVehicle}};

  std::unordered_map<std::string_view, ShapeType> shape_map_ = {
      {"sphere", ShapeType::kSphere},
      {"cylinder", ShapeType::kCylinder},
      {"cube", ShapeType::kCube},
      {"plane", ShapeType::kPlane}};
};

#endif
