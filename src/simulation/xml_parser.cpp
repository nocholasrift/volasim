#include <glad/glad.h>

#include <volasim/simulation/physics_interface.h>
#include <volasim/simulation/shape_renderable.h>
#include <volasim/simulation/xml_parser.h>
#include <volasim/vehicles/drone.h>

#include <iostream>
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

  ret.yaw = std::stof(camera_xml.child_value("cam_yaw"));
  ret.pitch = std::stof(camera_xml.child_value("cam_pitch"));
  ret.radius = std::stof(camera_xml.child_value("cam_distance"));
  ret.fov = std::stof(camera_xml.child_value("fov_deg"));

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
  if (vehicle_type == "drone") {
    vehicle = new Drone(item, 1. / 60.);
    // std::cout << "drone address: " << vehicle << std::endl;

    // hack for now until i include actual drone meshes
    ShapeMetadata drone_settings;
    generateShapeBuffers(drone_settings, item.child("block:class"));

    std::string pos_str = item.child_value("init_pose");
    std::stringstream ss(pos_str);
    std::string axis;

    glm::vec3 pos;
    int i = 0;
    while (ss >> axis)
      pos[i++] = std::stof(axis);

    vehicle->setTranslation(pos);

    DisplayObject* object =
        new DisplayObject(item.attribute("name").as_string());
    object->setRenderable(drone_settings);
    object->setTranslation(pos);

    PhysicsInterface::getInstance().preRegister(object, vehicle);

    world->addChild(object);
    // DisplayObject* drone_obj = new DisplayObject("drone");
    // drone_obj->setRenderable(drone_settings);
    // drone_obj->setTranslation();
  } else {
    std::string err_str = "[XMLParser] Unknown vehicle type: " + vehicle_type;
    throw std::runtime_error(err_str);
  }
}

void XMLParser::generateShapeBuffers(ShapeMetadata& settings,
                                     const pugi::xml_node& item) {

  pugi::xml_node geometry_node = item.child("geometry");

  if (settings.type == ShapeType::kUndefined)
    settings.type = shape_map_[geometry_node.attribute("type").as_string()];

  settings.color = item.child_value("color");

  if (settings.color.length() != 7) {
    std::string err_str =
        "[XMLParser] Invalid color string! " + settings.color +
        "\nShould be formatted as 6 hex digits preceeded by #";
    throw std::invalid_argument(err_str);
  }

  if (settings.color[0] != '#') {
    throw std::invalid_argument("[XMLParser] Color must start with #");
  }

  for (size_t i = 1; i < settings.color.length(); ++i) {
    if (!std::isxdigit(settings.color[i])) {
      throw std::invalid_argument("[XMLParser] Invalid hex digit in color: " +
                                  settings.color);
    }
  }

  glGenVertexArrays(1, &settings.vao);
  glBindVertexArray(settings.vao);

  glGenBuffers(1, &settings.vbo);
  glBindBuffer(GL_ARRAY_BUFFER, settings.vbo);

  glGenBuffers(1, &settings.ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, settings.ebo);

  switch (settings.type) {
    case ShapeType::kSphere:
      break;
    case ShapeType::kCylinder: {
      settings.radius =
          std::stof(geometry_node.attribute("radius").as_string());
      settings.height =
          std::stof(geometry_node.attribute("length").as_string());

      float n_sectors = 32;

      // Cylinder cylinder(settings.radius, settings.radius, settings.height, 32,
      //                   2);
      std::vector<float> vertices;
      std::vector<float> normals;
      for (int i = 0; i < 2; ++i) {
        float h = i * settings.height;

        float sector_angle;
        for (int j = 0; j <= n_sectors; ++j) {
          sector_angle = j * 2 * M_PI / n_sectors;
          float vx = settings.radius * cos(sector_angle);
          float vy = settings.radius * sin(sector_angle);

          vertices.push_back(vx);
          vertices.push_back(vy);
          vertices.push_back(h);

          normals.push_back(vx / settings.radius);
          normals.push_back(vy / settings.radius);
          normals.push_back(0.);
        }
      }

      int base_center_idx = (int)vertices.size() / 3;
      int top_center_idx = base_center_idx + n_sectors + 1;

      for (int i = 0; i < 2; ++i) {
        float h = i * settings.height;
        float nz = 2 * i - 1;

        vertices.push_back(0);
        vertices.push_back(0);
        vertices.push_back(h);

        normals.push_back(0);
        normals.push_back(0);
        normals.push_back(nz);

        float sector_angle;
        for (int j = 0; j <= n_sectors; ++j) {
          sector_angle = j * 2 * M_PI / n_sectors;

          float vx = settings.radius * cos(sector_angle);
          float vy = settings.radius * sin(sector_angle);

          vertices.push_back(vx);
          vertices.push_back(vy);
          vertices.push_back(h);

          normals.push_back(0);
          normals.push_back(0);
          normals.push_back(nz);
        }
      }

      // merge indices and normals together
      std::vector<float> vert_norms;
      for (size_t i = 0; i < vertices.size(); i += 3) {
        vert_norms.push_back(vertices[i]);
        vert_norms.push_back(vertices[i + 1]);
        vert_norms.push_back(vertices[i + 2]);

        vert_norms.push_back(normals[i]);
        vert_norms.push_back(normals[i + 1]);
        vert_norms.push_back(normals[i + 2]);
      }

      // int base_center_idx = 2 * (n_sectors + 1);
      // int top_center_idx = base_center_idx + n_sectors + 1;

      std::vector<int> indices;
      int k1 = 0;
      int k2 = n_sectors + 1;

      // indices for side surface
      for (int i = 0; i < n_sectors; ++i, ++k1, ++k2) {
        // tri 1
        indices.push_back(k1);
        indices.push_back(k1 + 1);
        indices.push_back(k2);

        // tri 2
        indices.push_back(k2);
        indices.push_back(k1 + 1);
        indices.push_back(k2 + 1);
      }

      // indices for base
      for (int i = 0, k = base_center_idx + 1; i < n_sectors; ++i, ++k) {
        if (i < n_sectors - 1) {
          indices.push_back(base_center_idx);
          indices.push_back(k + 1);
          indices.push_back(k);
        } else {
          indices.push_back(base_center_idx);
          indices.push_back(base_center_idx + 1);
          indices.push_back(k);
        }
      }

      // indices for top
      for (int i = 0, k = top_center_idx + 1; i < n_sectors; ++i, ++k) {
        if (i < n_sectors - 1) {
          indices.push_back(top_center_idx);
          indices.push_back(k);
          indices.push_back(k + 1);
        } else {
          indices.push_back(top_center_idx);
          indices.push_back(k);
          indices.push_back(top_center_idx + 1);
        }
      }

      // glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float),
      //              vertices.data(), GL_STATIC_DRAW);
      glBufferData(GL_ARRAY_BUFFER, vert_norms.size() * sizeof(float),
                   vert_norms.data(), GL_STATIC_DRAW);

      // copy index data to VBO
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(int),
                   indices.data(), GL_STATIC_DRAW);

      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)0);

      // normals
      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)(3 * sizeof(float)));

      settings.index_count = indices.size();

      break;
    }  // end case kCylinder
    case ShapeType::kPlane: {
      settings.x_min = std::stof(item.child_value("x_min"));
      settings.x_max = std::stof(item.child_value("x_max"));
      settings.y_min = std::stof(item.child_value("y_min"));
      settings.y_max = std::stof(item.child_value("y_max"));
      settings.z = std::stof(item.child_value("z"));
      settings.name = item.attribute("class").as_string();

      float ground_verts[] = {
          //positions
          settings.x_max, settings.y_max, settings.z, 0, 0, 1,
          settings.x_max, settings.y_min, settings.z, 0, 0, 1,
          settings.x_min, settings.y_min, settings.z, 0, 0, 1,
          settings.x_min, settings.y_max, settings.z, 0, 0, 1,
      };

      unsigned int indices[] = {0, 1, 3, 1, 2, 3};

      // float normals[] = {0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1};

      glBufferData(GL_ARRAY_BUFFER, sizeof(ground_verts), ground_verts,
                   GL_STATIC_DRAW);

      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)0);

      // normals
      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)(3 * sizeof(float)));

      glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices,
                   GL_STATIC_DRAW);
      settings.index_count = 6;

      break;
    }  // end case kPlane
    case ShapeType::kCube: {
      // settings.x_mi
      float sz = std::stof(geometry_node.attribute("size").as_string());
      settings.size = sz;

      // float verts[] = {0 - sz / 2, -sz / 2,    -sz / 2,  1 - sz / 2, sz / 2,
      //                  -sz / 2,    2 - sz / 2, sz / 2,   sz / 2,     3 - sz / 2,
      //                  -sz / 2,    sz / 2,     4 sz / 2, -sz / 2,    sz / 2,
      //                  5 sz / 2,   sz / 2,     sz / 2,   6 sz / 2,   sz / 2,
      //                  -sz / 2,    7 sz / 2,   -sz / 2,  -sz / 2};
      //
      // unsigned int indices[] = {// left face
      //                           0, 1, 2, 0, 2, 3,
      //                           // back face
      //                           0, 3, 7, 3, 4, 7,
      //                           // right face
      //                           4, 5, 7, 5, 6, 7,
      //                           // front face
      //                           1, 2, 6, 2, 5, 6,
      //                           // top face
      //                           2, 3, 4, 2, 4, 5,
      //                           // bottom face
      //                           0, 1, 6, 0, 6, 7};

      float verts[] = {
          // left face
          -sz / 2, -sz / 2, -sz / 2, -1, 0, 0, -sz / 2, sz / 2, -sz / 2, -1, 0,
          0, -sz / 2, sz / 2, sz / 2, -1, 0, 0, -sz / 2, -sz / 2, -sz / 2, -1,
          0, 0, -sz / 2, sz / 2, sz / 2, -1, 0, 0, -sz / 2, -sz / 2, sz / 2, -1,
          0, 0,

          // back face
          -sz / 2, -sz / 2, -sz / 2, 0, -1, 0, -sz / 2, -sz / 2, sz / 2, 0, -1,
          0, sz / 2, -sz / 2, -sz / 2, 0, -1, 0, -sz / 2, -sz / 2, sz / 2, 0,
          -1, 0, sz / 2, -sz / 2, sz / 2, 0, -1, 0, sz / 2, -sz / 2, -sz / 2, 0,
          -1, 0,

          // right face
          sz / 2, -sz / 2, sz / 2, 1, 0, 0, sz / 2, sz / 2, sz / 2, 1, 0, 0,
          sz / 2, -sz / 2, -sz / 2, 1, 0, 0, sz / 2, sz / 2, sz / 2, 1, 0, 0,
          sz / 2, sz / 2, -sz / 2, 1, 0, 0, sz / 2, -sz / 2, -sz / 2, 1, 0, 0,

          // front face
          -sz / 2, sz / 2, -sz / 2, 0, 1, 0, -sz / 2, sz / 2, sz / 2, 0, 1, 0,
          sz / 2, sz / 2, -sz / 2, 0, 1, 0, -sz / 2, sz / 2, sz / 2, 0, 1, 0,
          sz / 2, sz / 2, sz / 2, 0, 1, 0, sz / 2, sz / 2, -sz / 2, 0, 1, 0,

          // top face
          -sz / 2, sz / 2, sz / 2, 0, 0, 1, -sz / 2, -sz / 2, sz / 2, 0, 0, 1,
          sz / 2, -sz / 2, sz / 2, 0, 0, 1, -sz / 2, sz / 2, sz / 2, 0, 0, 1,
          sz / 2, -sz / 2, sz / 2, 0, 0, 1, sz / 2, sz / 2, sz / 2, 0, 0, 1,

          // bottom face
          -sz / 2, -sz / 2, -sz / 2, 0, 0, -1, -sz / 2, sz / 2, -sz / 2, 0, 0,
          -1, sz / 2, sz / 2, -sz / 2, 0, 0, -1, -sz / 2, -sz / 2, -sz / 2, 0,
          0, -1, sz / 2, sz / 2, -sz / 2, 0, 0, -1, sz / 2, -sz / 2, -sz / 2, 0,
          0, -1

      };

      int indices[36];
      for (int i = 0; i < 36; ++i)
        indices[i] = i;

      // float normals[] = {-1, 0, 0, -1, 0, 0, 0, -1, 0,  0, -1, 0,
      //                    1,  0, 0, 1,  0, 0, 0, 1,  0,  0, 1,  0,
      //                    0,  0, 1, 0,  0, 1, 0, 0,  -1, 0, 0,  -1};

      glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);

      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)0);

      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)(3 * sizeof(float)));

      glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices,
                   GL_STATIC_DRAW);

      settings.index_count = 36;

      break;
    }
    default:
      break;
  }

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void XMLParser::handleElement(const pugi::xml_node& item,
                              DisplayObjectContainer* world) {
  ShapeMetadata settings;
  settings.type = ShapeType::kPlane;
  generateShapeBuffers(settings, item);

  std::string name = item.attribute("class").as_string();
  createAndAddRenderable(name, settings, glm::vec3(0., 0., 0.), world);
}

void XMLParser::handleBlockDefinition(const pugi::xml_node& item) {
  std::string class_name = item.attribute("name").as_string();
  if (defined_class_count_.find(class_name) != defined_class_count_.end()) {
    std::string err_str = "[XMLParser] class name: " + class_name +
                          " has already been defined more than once.";
    throw std::invalid_argument(err_str);
  }

  defined_class_count_.insert({class_name, 0});

  ShapeMetadata settings;
  pugi::xml_node geometry_node = item.child("geometry");
  settings.type = shape_map_[geometry_node.attribute("type").as_string()];
  generateShapeBuffers(settings, item);

  class_settings_.insert({class_name, settings});
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

  createAndAddRenderable(name, class_settings_[class_name], pos, world);

  defined_class_count_[class_name]++;
}

void XMLParser::createAndAddRenderable(const std::string& name,
                                       const ShapeMetadata& settings,
                                       const glm::vec3& pos,
                                       DisplayObjectContainer* world) {

  DisplayObject* object = new DisplayObject(name);
  object->setRenderable(settings);
  object->setTranslation(pos);
  world->addChild(object);
}
