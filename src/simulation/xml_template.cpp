#include <volasim/simulation/xml_template.h>

#include <stdexcept>

namespace xml_template {

// Anonymous namespace keeps trim() private to this translation unit.
namespace {

std::string trim(const std::string& s) {
  size_t begin = s.find_first_not_of(" \t\n\r");
  if (begin == std::string::npos) {
    return "";
  }
  size_t end = s.find_last_not_of(" \t\n\r");
  return s.substr(begin, end - begin + 1);
}

}  // namespace

std::string expand(const std::string& text, const Params& params) {
  std::string out;
  out.reserve(text.size());

  size_t i = 0;
  while (i < text.size()) {
    if (text[i] != '$' || i + 1 >= text.size() || text[i + 1] != '{') {
      out += text[i++];
      continue;
    }

    size_t close = text.find('}', i + 2);
    if (close == std::string::npos) {
      throw std::runtime_error("[xml_template] unterminated '${' in template");
    }

    std::string token = text.substr(i + 2, close - (i + 2));
    size_t bar = token.find('|');
    std::string name = trim(token.substr(0, bar));

    auto it = params.find(name);
    if (it != params.end()) {
      out += it->second;
    } else if (bar != std::string::npos) {
      out += token.substr(bar + 1);
    } else {
      throw std::runtime_error("[xml_template] missing required parameter '" +
                               name + "'");
    }

    i = close + 1;
  }

  return out;
}

}  // namespace xml_template
