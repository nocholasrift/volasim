#ifndef XMLPARSER_H
#define XMLPARSER_H

#include <pugixml.hpp>

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
};

class XMLParser {
 public:
  XMLParser(const std::string& fname);

  std::vector<ShapeMetadata> getRenderables();

 private:
  void throwError(std::string_view fname, const pugi::xml_parse_result& result);
  pugi::xml_document doc_;

  std::unordered_map<std::string_view, XMLTags> type_map_ = {
      {"element", XMLTags::kElement},
      {"block:class", XMLTags::kBlockDefinition},
      {"block", XMLTags::kBlock}};

  std::unordered_map<std::string_view, ShapeType> shape_map_ = {
      {"sphere", ShapeType::kSphere},
      {"cylinder", ShapeType::kCylinder},
      {"cube", ShapeType::kCube},
      {"plane", ShapeType::kPlane}};
};

#endif
