#ifndef XMLTEMPLATE_H
#define XMLTEMPLATE_H

#include <string>
#include <unordered_map>

// MVSIM-style text preprocessor for XML include files. Purely operates on
// strings so it stays decoupled from the parser and the scene graph.
namespace xml_template {

using Params = std::unordered_map<std::string, std::string>;

// Replaces every ${name|default} / ${name} placeholder with the matching param
// value, falling back to the inline default. Throws if a ${name} without a
// default is left unresolved.
std::string expand(const std::string& text, const Params& params);

}  // namespace xml_template

#endif
