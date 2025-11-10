#include <glad/glad.h>

#include <volasim/simulation/mesh_renderable.h>
#include <volasim/simulation/physics_interface.h>
#include <volasim/simulation/shape_renderable.h>
#include <volasim/simulation/xml_parser.h>
#include <volasim/vehicles/drone.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

XMLParser::XMLParser(const std::string& fname) {
  pugi::xml_parse_result result = doc_.load_file(fname.c_str());

  if (!result)
    throwError(fname, result);

  type_map_.insert({"element", XMLTags::kElement});
  type_map_.insert({"block:class", XMLTags::kBlockDefinition});
}

CameraSettings XMLParser::getCameraSettings() {
  CameraSettings ret;

  pugi::xml_node curr_node = doc_.child("volasim_world");
  pugi::xml_node camera_xml = curr_node.child("gui");

  if (!camera_xml) {
    throw std::runtime_error("[XMLParser] Missing 'gui' node in world XML");
  }

  try {
    ret.yaw = std::stof(camera_xml.child_value("cam_yaw"));
    ret.pitch = std::stof(camera_xml.child_value("cam_pitch"));
    ret.radius = std::stof(camera_xml.child_value("cam_distance"));
    ret.fov = std::stof(camera_xml.child_value("fov_deg"));
    ret.fps = std::stof(camera_xml.child_value("fps"));
  } catch (const std::exception& e) {
    throw std::runtime_error("[XMLParser] Invalid camera settings in XML: " +
                             std::string(e.what()));
  }

  float width = std::stof(camera_xml.child_value("window_width"));
  float height = std::stof(camera_xml.child_value("window_height"));
  ret.window_sz = glm::ivec2(width, height);

  ret.target = nullptr;

  return ret;
}

void XMLParser::loadWorldFromXML(DisplayObjectContainer* world) {
  // ret.resize(50);

  pugi::xml_node curr_node = doc_.child("volasim_world");

  for (pugi::xml_node item = curr_node.first_child(); item;
       item = item.next_sibling()) {
    switch (type_map_[item.name()]) {
      case XMLTags::kGUI:
        break;

      case XMLTags::kElement:
        handleElement(item, world);
        break;

      case XMLTags::kBlockDefinition:
        handleBlockDefinition(item);
        break;

      case XMLTags::kBlock:
        handleBlock(item, world);
        break;

      case XMLTags::kVehicle:
        handleVehicle(item, world);
        break;

      default: {
        std::string err_str =
            std::string("[XMLParser] Invalid tag found ") + item.name();
        throw std::runtime_error(err_str);
      }
    }
  }
}

void XMLParser::throwError(std::string_view fname,
                           const pugi::xml_parse_result& result) {

  std::ostringstream err_msg;
  err_msg << "XML [" << fname << "] parsed with errors, attr value: ["
          << doc_.child("node").attribute("attr").value() << "]\n";
  err_msg << "Error description: " << result.description() << "\n";
  err_msg << "Error offset: " << result.offset << "\n";

  throw std::runtime_error(err_msg.str());
}

void XMLParser::handleVehicle(const pugi::xml_node& item,
                              DisplayObjectContainer* world) {
  std::string vehicle_type = item.attribute("class").as_string();
  DynamicObject* vehicle;
  vehicle = new Drone(item, 1. / 60.);
  // std::cout << "drone address: " << vehicle << std::endl;

  auto rend_iter = renderables_.find(vehicle_type);
  if (rend_iter == renderables_.end()) {
    std::string err_str = "[XMLParser] Unknown vehicle type: " + vehicle_type;
    throw std::runtime_error(err_str);
  }

  std::string pos_str = item.child_value("init_pose");
  std::stringstream ss(pos_str);
  std::string axis;

  glm::vec3 pos;
  int i = 0;
  while (ss >> axis)
    pos[i++] = std::stof(axis);

  vehicle->setTranslation(pos);

  DisplayObject* object = new DisplayObject(item.attribute("name").as_string());

  object->setRenderable(rend_iter->second);
  object->setTranslation(pos);

  PhysicsInterface::getInstance().preRegister(object, vehicle);

  world->addChild(object);
}

void XMLParser::handleElement(const pugi::xml_node& item,
                              DisplayObjectContainer* world) {
  std::string name = item.attribute("class").as_string();

  std::shared_ptr<ShapeRenderable> renderable =
      std::make_shared<ShapeRenderable>();
  renderable->buildFromXML(item);
  renderables_[name] = renderable;

  createAndAddRenderable(name, renderables_[name], glm::vec3(0., 0., 0.),
                         world);
}

void XMLParser::handleBlockDefinition(const pugi::xml_node& item) {
  std::string class_name = item.attribute("name").as_string();
  if (defined_class_count_.find(class_name) != defined_class_count_.end()) {
    std::string err_str = "[XMLParser] class name: " + class_name +
                          " has already been defined more than once.";
    throw std::invalid_argument(err_str);
  }

  defined_class_count_.insert({class_name, 0});

  pugi::xml_node geometry_node = item.child("geometry");

  std::shared_ptr<Renderable> renderable;
  if (std::string(geometry_node.attribute("type").as_string()) == "mesh") {
    renderable = std::make_shared<MeshRenderable>();
  } else {
    renderable = std::make_shared<ShapeRenderable>();
  }

  renderable->buildFromXML(item);
  renderables_[class_name] = renderable;

  // ShapeMetadata settings;
  // pugi::xml_node geometry_node = item.child("geometry");
  // settings.type = shape_map_[geometry_node.attribute("type").as_string()];
  // generateShapeBuffers(settings, item);

  // class_settings_.insert({class_name, settings});
}

void XMLParser::handleBlock(const pugi::xml_node& item,
                            DisplayObjectContainer* world) {
  std::string class_name = item.attribute("class").as_string();

  if (defined_class_count_.find(class_name) == defined_class_count_.end()) {
    std::string err_str =
        "[XMLParser] class name: " + class_name + " has not yet been defined.";
    throw std::invalid_argument(err_str);
  }

  std::string name =
      class_name + std::to_string(defined_class_count_[class_name]);

  std::string pos_str = item.child_value("init_pose");
  std::stringstream ss(pos_str);
  std::string axis;

  glm::vec3 pos;
  int i = 0;
  while (ss >> axis)
    pos[i++] = std::stof(axis);

  createAndAddRenderable(name, renderables_[class_name], pos, world);

  defined_class_count_[class_name]++;
}

void XMLParser::createAndAddRenderable(
    const std::string& name, const std::shared_ptr<Renderable> renderable,
    const glm::vec3& pos, DisplayObjectContainer* world) {

  DisplayObject* object = new DisplayObject(name);
  object->setRenderable(renderable);
  object->setTranslation(pos);
  world->addChild(object);
}
