#ifndef XMLPARSER_H
#define XMLPARSER_H

#include <pugixml.hpp>

#include <volasim/sensors/depth_sensor.h>
#include <volasim/simulation/camera.h>
#include <volasim/simulation/entity.h>
#include <volasim/simulation/shape_renderable.h>

#include <list>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

enum class XMLTags {
  kGUI = 0,
  kElement,
  kBlockDefinition,
  kBlock,
  kVehicle,
  kVehicleDefinition,
  kSensor,
  kInclude,
  kInitPose,
};

class XMLParser {
 public:
  XMLParser() = delete;
  XMLParser(const std::string& fname);

  Camera                  loadCamera();
  std::unique_ptr<Entity> loadWorldFromXML(std::list<GPUSensor>& sensors);

 private:
  void loadDoc(pugi::xml_document& doc, const std::string& fname);

  // Returns the raw, unsubstituted file text, cached so repeated includes of
  // the same file avoid re-reading from disk.
  const std::string& readFile(const std::string& fname);

  void throwError(std::string_view fname, const pugi::xml_parse_result& result);

  void parseChildren(const pugi::xml_node& parent, Entity& root,
                     std::list<GPUSensor>& sensors, const std::string& source);

  void handleSensor(const pugi::xml_node& item, Entity& root,
                    std::list<GPUSensor>& sensors);

  Entity& handleVehicle(const pugi::xml_node& vehicle_node, Entity& world);

  void handleElement(const pugi::xml_node& item, Entity& world);

  void handleBlockDefinition(const pugi::xml_node& item);

  void handleVehicleDefinition(const pugi::xml_node& item);

  void handleBlock(const pugi::xml_node& item, Entity& world);

  void createAndAddRenderable(const std::string&                 name,
                              const std::shared_ptr<Renderable>& renderable,
                              const glm::vec3& pos, Entity& world);

 private:
  struct VehicleClassDef {
    std::shared_ptr<Renderable> renderable;
    std::string                 dynamics_type;
    pugi::xml_node              xml_node;
  };

  std::string        fname_;
  pugi::xml_document doc_;

  // Parsed include docs are kept alive here because definitions taken from them
  // (e.g. vehicle:class nodes) are referenced after parsing returns.
  std::vector<std::shared_ptr<pugi::xml_document>> included_docs_;
  // Raw file text keyed by path; unsubstituted so it can be reused per include.
  std::unordered_map<std::string, std::string> file_cache_;

  std::unordered_map<std::string, int>           defined_class_count_;
  std::unordered_map<std::string, ShapeMetadata> class_settings_;

  std::unordered_map<std::string, std::shared_ptr<Renderable>> renderables_;
  std::unordered_map<std::string, VehicleClassDef> vehicle_name_to_definition_;

  std::unordered_map<std::string_view, XMLTags> type_map_{
      {{"gui", XMLTags::kGUI},
       {"element", XMLTags::kElement},
       {"init_pose", XMLTags::kInitPose},
       {"block:class", XMLTags::kBlockDefinition},
       {"block", XMLTags::kBlock},
       {"vehicle", XMLTags::kVehicle},
       {"vehicle:class", XMLTags::kVehicleDefinition},
       {"sensor", XMLTags::kSensor},
       {"include", XMLTags::kInclude}}};

  EntityFactory entity_factory_;

  // vehicle registries
  std::unordered_map<std::string,
                     std::function<DynamicObject*(const pugi::xml_node&)>>
      vehicle_registry_;
};

#endif
