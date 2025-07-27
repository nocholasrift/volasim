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

  // for (pugi::xml_node item = stuff.first_child(); item;
  //      item = item.next_sibling()) {
  //   std::cout << item.name() << ": ";
  //   for (pugi::xml_node attr = item.first_child(); attr;
  //        attr = attr.next_sibling()) {
  //     std::cout << attr.name() << ": " << attr.child_value() << "\n";
  //   }
  // }

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
        ShapeMetadata& settings = ret.emplace_back();
        settings.type = ShapeType::kPlane;
        settings.x_min = std::stof(item.child_value("x_min"));
        settings.x_max = std::stof(item.child_value("x_max"));
        settings.y_min = std::stof(item.child_value("x_min"));
        settings.y_max = std::stof(item.child_value("y_max"));
        settings.z = std::stof(item.child_value("z"));
        settings.color = item.child_value("color");
        settings.name = item.attribute("class").as_string();

        if (settings.color.length() != 7) {
          std::string err_str =
              "[XMLParser] Invalid color string! " + settings.color +
              "\nShould be formatted as 6 hex digits preceeded by #";
          throw std::invalid_argument(err_str);
        }
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

        pugi::xml_node geometry_node = item.child("geometry");

        ShapeMetadata settings;
        settings.type = shape_map_[geometry_node.attribute("type").as_string()];
        settings.radius =
            std::stof(geometry_node.attribute("radius").as_string());
        settings.height =
            std::stof(geometry_node.attribute("length").as_string());
        settings.color = item.child_value("color");

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

        std::string pos_str = item.child_value("init_pose");
        std::stringstream ss(pos_str);
        std::string axis;

        int i = 0;
        while (ss >> axis)
          settings.pos[i++] = std::stof(axis);

        defined_class_count[class_name]++;

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
