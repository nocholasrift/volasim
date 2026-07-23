#include <glad/glad.h>

#include <math.h>
#include <volasim/simulation/mesh_renderable.h>
#include <volasim/simulation/shape_renderable.h>
#include <volasim/simulation/xml_parser.h>
#include <volasim/simulation/xml_template.h>
#include <volasim/vehicles/drone.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

XMLParser::XMLParser(const std::string& fname) : fname_(fname) {
  loadDoc(doc_, fname);

  // build vehicle registry
  vehicle_registry_["drone"] = [](const pugi::xml_node& node) {
    return Drone::fromXML(node);
  };
}

Camera XMLParser::loadCamera() {
  pugi::xml_node gui_node = doc_.child("volasim_world").child("gui");
  if (!gui_node)
    throw std::runtime_error("[XMLParser] Missing 'gui' node in world XML");
  return Camera::fromXML(gui_node);
}

std::unique_ptr<Entity> XMLParser::loadWorldFromXML(
    std::list<GPUSensor>& sensors) {

  std::unique_ptr<Entity> world = entity_factory_.create("World");
  parseChildren(doc_.child("volasim_world"), *world, sensors, fname_);
  return world;
}

void XMLParser::loadDoc(pugi::xml_document& doc, const std::string& fname) {
  pugi::xml_parse_result result = doc.load_file(fname.c_str());

  if (!result) {
    throwError(fname, result);
  }
}

const std::string& XMLParser::readFile(const std::string& fname) {
  auto it = file_cache_.find(fname);
  if (it != file_cache_.end()) {
    return it->second;
  }

  std::ifstream file(fname);
  if (!file) {
    throw std::runtime_error("[XMLParser] cannot open include file: " + fname);
  }

  std::stringstream ss;
  ss << file.rdbuf();
  return file_cache_.emplace(fname, ss.str()).first->second;
}

void XMLParser::throwError(std::string_view              fname,
                           const pugi::xml_parse_result& result) {
  std::ostringstream err_msg;
  err_msg << "XML [" << fname << "] parsed with errors\n";
  err_msg << "Error description: " << result.description() << "\n";
  err_msg << "Error offset: " << result.offset << "\n";

  throw std::runtime_error(err_msg.str());
}

void XMLParser::handleVehicleDefinition(const pugi::xml_node& item) {
  std::string vehicle_name = item.attribute("name").as_string();
  std::string dynamics_type =
      item.child("dynamics").attribute("class").as_string();
  pugi::xml_node xml_node = item.child("dynamics");

  pugi::xml_node geometry_node = item.child("geometry");

  VehicleClassDef vehicle_definition;
  if (std::string(geometry_node.attribute("type").as_string()) == "mesh") {
    vehicle_definition.renderable = std::make_shared<MeshRenderable>();
  } else {
    vehicle_definition.renderable = std::make_shared<ShapeRenderable>();
  }

  auto t0 = std::chrono::steady_clock::now();
  vehicle_definition.renderable->buildFromXML(item);
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t0)
                .count();
  std::cout << "[XMLParser] buildFromXML(" << vehicle_name << "): " << us
            << "us\n";

  vehicle_definition.dynamics_type = dynamics_type;
  vehicle_definition.xml_node      = xml_node;

  vehicle_name_to_definition_[vehicle_name] = vehicle_definition;
}

void XMLParser::parseChildren(const pugi::xml_node& parent, Entity& root,
                              std::list<GPUSensor>& sensors,
                              const std::string&    source) {

  for (pugi::xml_node item = parent.first_child(); item;
       item                = item.next_sibling()) {
    auto itr = type_map_.find(item.name());
    if (itr == type_map_.end()) {
      throw std::runtime_error("[XMLParser] " + source + ": invalid tag <" +
                               item.name() + ">");
    }

    // build a label: tag + first meaningful identifier attribute
    std::string label = item.name();
    for (const char* attr : {"name", "class", "file"}) {
      const char* val = item.attribute(attr).value();
      if (*val) {
        label += std::string("(") + val + ")";
        break;
      }
    }

    auto t0 = std::chrono::steady_clock::now();
    try {
      switch (itr->second) {
        case XMLTags::kGUI:
          break;

        case XMLTags::kElement:
          handleElement(item, root);
          break;

        case XMLTags::kBlockDefinition:
          handleBlockDefinition(item);
          break;

        case XMLTags::kBlock:
          handleBlock(item, root);
          break;

        case XMLTags::kVehicle: {
          Entity& vehicle_object = handleVehicle(item, root);
          parseChildren(item, vehicle_object, sensors, source);
          break;
        }

        case XMLTags::kVehicleDefinition:
          handleVehicleDefinition(item);
          break;

        case XMLTags::kSensor:
          handleSensor(item, root, sensors);
          break;

        case XMLTags::kInclude: {
          const std::string include_fname = item.attribute("file").as_string();

          // Every attribute other than "file" is a template parameter.
          xml_template::Params params;
          for (pugi::xml_attribute attr = item.first_attribute(); attr;
               attr                     = attr.next_attribute()) {
            if (std::string(attr.name()) != "file") {
              params.emplace(attr.name(), attr.value());
            }
          }

          std::string expanded =
              xml_template::expand(readFile(include_fname), params);

          auto                   doc = std::make_shared<pugi::xml_document>();
          pugi::xml_parse_result result = doc->load_string(expanded.c_str());
          if (!result) {
            throwError(include_fname, result);
          }
          included_docs_.push_back(doc);

          parseChildren(*doc, root, sensors, include_fname);
          break;
        }

        // added default case to remove compiler warnings on non-included tags
        default:
          break;
      }
    } catch (const std::exception& e) {
      throw std::runtime_error("[XMLParser] " + source + " <" + item.name() +
                               ">: " + e.what());
    }
    // auto us = std::chrono::duration_cast<std::chrono::microseconds>(
    //     std::chrono::steady_clock::now() - t0).count();
    // std::cout << "[XMLParser] " << label << ": " << us << "us\n";
  }
}

Entity& XMLParser::handleVehicle(const pugi::xml_node& vehicle_node,
                                 Entity&               world) {

  std::string vehicle_type = vehicle_node.attribute("class").as_string();

  auto itr = vehicle_name_to_definition_.find(vehicle_type);
  if (itr == vehicle_name_to_definition_.end()) {
    std::string err_str = "[XMLParser] Unknown vehicle type: " + vehicle_type;
    throw std::runtime_error(err_str);
  }

  VehicleClassDef vehicle_definition = itr->second;

  std::string       pos_str = vehicle_node.child_value("init_pose");
  std::stringstream ss(pos_str);
  std::string       axis;

  glm::vec3 pos;
  int       i = 0;
  while (ss >> axis)
    pos[i++] = std::stof(axis);

  DynamicObject* vehicle = vehicle_registry_.at(
      vehicle_definition.dynamics_type)(vehicle_definition.xml_node);
  vehicle->setTranslation(pos);

  std::unique_ptr<Entity> object =
      entity_factory_.create(vehicle_node.attribute("name").as_string());

  object->setRenderable(vehicle_definition.renderable);
  object->setTranslation(pos);

  // Dynamics must be set before addChild: OBJ_ADD reads isDynamic() to decide
  // between a MOVING (dynamic) and a static body.
  object->setDynamics(std::unique_ptr<DynamicObject>(vehicle));

  return world.addChild(std::move(object));
}

void XMLParser::handleElement(const pugi::xml_node& item, Entity& world) {
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
}

void XMLParser::handleBlock(const pugi::xml_node& item, Entity& world) {
  std::string class_name = item.attribute("class").as_string();

  if (defined_class_count_.find(class_name) == defined_class_count_.end()) {
    std::string err_str =
        "[XMLParser] class name: " + class_name + " has not yet been defined.";
    throw std::invalid_argument(err_str);
  }

  std::string name =
      class_name + std::to_string(defined_class_count_[class_name]);

  std::string       pos_str = item.child_value("init_pose");
  std::stringstream ss(pos_str);
  std::string       axis;

  glm::vec3 pos;
  int       i = 0;
  while (ss >> axis)
    pos[i++] = std::stof(axis);

  createAndAddRenderable(name, renderables_[class_name], pos, world);

  defined_class_count_[class_name]++;
}

void XMLParser::handleSensor(const pugi::xml_node& item, Entity& root,
                             std::list<GPUSensor>& sensors) {
  pugi::xml_node geometry_node = item.child("geometry");

  std::shared_ptr<Renderable> renderable;
  if (std::string(geometry_node.attribute("type").as_string()) == "mesh") {
    renderable = std::make_shared<MeshRenderable>();
  } else {
    renderable = std::make_shared<ShapeRenderable>();
  }

  renderable->buildFromXML(item);

  // Mount pose relative to the parent robot: "x y z roll pitch yaw", with the
  // rotation in degrees. Applied to the entity so it moves both the mesh
  // and the depth camera, which tracks this object's frame.
  glm::vec3   translation(0.F);
  glm::vec3   rpy_deg(0.F);
  std::string pose_str = item.child_value("pose");
  if (!pose_str.empty()) {
    constexpr int     kPoseComponents = 6;  // x y z roll pitch yaw
    std::stringstream ss(pose_str);
    std::array<float, kPoseComponents> pose  = {0, 0, 0, 0, 0, 0};
    int                                i     = 0;
    float                              value = 0.F;
    while (ss >> value && i < kPoseComponents) {
      pose[i++] = value;
    }
    translation = glm::vec3(pose[0], pose[1], pose[2]);
    rpy_deg     = glm::vec3(pose[3], pose[4], pose[5]);
  }

  std::string name = item.attribute("name").as_string();
  if (name.empty()) {
    name = "sensor";
  }

  // Purely visual: no convex decomposition means no physics body, so no
  // sensor/robot collision to filter out.
  std::unique_ptr<Entity> object = entity_factory_.create(name);
  object->setRenderable(renderable);
  object->setTranslation(translation);
  object->setRotation(glm::radians(rpy_deg));

  Entity* sensor_entity_ptr = &root.addChild(std::move(object));

  // Parenting to the sensor object makes the depth camera follow the robot's
  // motion through the scene graph.
  sensors.emplace_back(GPUSensor::fromXML(item, sensor_entity_ptr));
}

void XMLParser::createAndAddRenderable(
    const std::string& name, const std::shared_ptr<Renderable>& renderable,
    const glm::vec3& pos, Entity& world) {

  std::unique_ptr<Entity> object = entity_factory_.create(name);
  object->setRenderable(renderable);
  object->setTranslation(pos);
  world.addChild(std::move(object));
}
