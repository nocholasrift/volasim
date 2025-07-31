#include <glad/glad.h>

#include <volasim/models/cylinder.h>
#include <volasim/simulation/shape_renderable.h>
#include <volasim/simulation/xml_parser.h>

#include <iostream>
#include <sstream>
#include <stdexcept>

XMLParser::XMLParser(const std::string& fname) {
  pugi::xml_parse_result result = doc_.load_file(fname.c_str());

  if (!result)
    throwError(fname, result);

  pugi::xml_node stuff = doc_.child("volasim_world");

  type_map_.insert({"element", XMLTags::kElement});
  type_map_.insert({"block:class", XMLTags::kBlockDefinition});
}

std::vector<ShapeMetadata> XMLParser::getRenderables() {
  std::vector<ShapeMetadata> ret;
  // ret.resize(50);

  std::unordered_map<std::string, int> defined_class_count;
  std::unordered_map<std::string, ShapeMetadata> class_settings;

  pugi::xml_node curr_node = doc_.child("volasim_world");

  for (pugi::xml_node item = curr_node.first_child(); item;
       item = item.next_sibling()) {
    switch (type_map_[item.name()]) {
      case XMLTags::kGUI:
        break;

      case XMLTags::kElement: {
        pugi::xml_node geometry_node = item.child("geometry");

        ShapeMetadata& settings = ret.emplace_back();
        settings.type = ShapeType::kPlane;
        generateShapeBuffers(settings, item);
        break;
      }

      case XMLTags::kBlockDefinition: {
        std::string class_name = item.attribute("name").as_string();
        if (defined_class_count.find(class_name) != defined_class_count.end()) {
          std::string err_str = "[XMLParser] class name: " + class_name +
                                " has already been defined more than once.";
          throw std::invalid_argument(err_str);
        }

        defined_class_count.insert({class_name, 0});

        ShapeMetadata settings;
        pugi::xml_node geometry_node = item.child("geometry");
        settings.type = shape_map_[geometry_node.attribute("type").as_string()];
        generateShapeBuffers(settings, item);

        class_settings.insert({class_name, settings});
        break;
      }

      case XMLTags::kBlock: {
        std::string class_name = item.attribute("class").as_string();

        if (defined_class_count.find(class_name) == defined_class_count.end()) {
          std::string err_str = "[XMLParser] class name: " + class_name +
                                " has not yet been defined.";
          throw std::invalid_argument(err_str);
        }

        ShapeMetadata& settings = ret.emplace_back();
        settings.type = class_settings[class_name].type;
        settings.color = class_settings[class_name].color;
        settings.radius = class_settings[class_name].radius;
        settings.height = class_settings[class_name].height;
        settings.name =
            class_name + std::to_string(defined_class_count[class_name]);

        settings.vao = class_settings[class_name].vao;
        settings.vbo = class_settings[class_name].vbo;
        settings.ebo = class_settings[class_name].ebo;
        settings.index_count = class_settings[class_name].index_count;

        std::string pos_str = item.child_value("init_pose");
        std::stringstream ss(pos_str);
        std::string axis;

        int i = 0;
        while (ss >> axis)
          settings.pos[i++] = std::stof(axis);

        defined_class_count[class_name]++;

        break;
      }

      case XMLTags::kVehicle: {

        break;
      }

      default:
        std::string err_str =
            std::string("[XMLParser] Invalid tag found ") + item.name();
        throw std::runtime_error(err_str);
    }
  }

  return ret;
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
        }
      }

      int base_center_idx = (int)vertices.size() / 3;
      int top_center_idx = base_center_idx + n_sectors + 1;

      for (int i = 0; i < 2; ++i) {
        float h = i * settings.height;

        vertices.push_back(0);
        vertices.push_back(0);
        vertices.push_back(h);

        float sector_angle;
        for (int j = 0; j <= n_sectors; ++j) {
          sector_angle = j * 2 * M_PI / n_sectors;

          float vx = settings.radius * cos(sector_angle);
          float vy = settings.radius * sin(sector_angle);

          vertices.push_back(vx);
          vertices.push_back(vy);
          vertices.push_back(h);
        }
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

      glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float),
                   vertices.data(), GL_STATIC_DRAW);

      // copy index data to VBO
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(int),
                   indices.data(), GL_STATIC_DRAW);

      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                            (void*)0);

      std::cout << "indices size is: " << indices.size() << std::endl;
      settings.index_count = indices.size();

      break;
    }  // end case kCylinder
    case ShapeType::kPlane: {
      std::cout << "handling plane object" << std::endl;
      settings.x_min = std::stof(item.child_value("x_min"));
      settings.x_max = std::stof(item.child_value("x_max"));
      settings.y_min = std::stof(item.child_value("x_min"));
      settings.y_max = std::stof(item.child_value("y_max"));
      settings.z = std::stof(item.child_value("z"));
      settings.name = item.attribute("class").as_string();

      float ground_verts[] = {//positions
                              settings.x_max, settings.y_max, settings.z,
                              settings.x_max, settings.y_min, settings.z,
                              settings.x_min, settings.y_min, settings.z,
                              settings.x_min, settings.y_max, settings.z};

      unsigned int indices[] = {0, 1, 3, 1, 2, 3};

      glBufferData(GL_ARRAY_BUFFER, sizeof(ground_verts), ground_verts,
                   GL_STATIC_DRAW);

      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                            (void*)0);

      glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices,
                   GL_STATIC_DRAW);
      settings.index_count = 6;

      break;
    }  // end case kPlane
  }

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
