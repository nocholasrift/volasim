# Reads .vert/.frag files from SHADER_DIR and writes a C++ header to OUTPUT
# with each shader as an inline const char* inside namespace shaders.
#
# Usage (from add_custom_command):
#   cmake -DSHADER_DIR=... -DOUTPUT=... -P embed_shaders.cmake

file(GLOB SHADER_FILES "${SHADER_DIR}/*.vert" "${SHADER_DIR}/*.frag")
list(SORT SHADER_FILES)

set(SEMICOLON ";")
set(HEADER "// Auto-generated from definitions/shaders/ — do not edit.\n")
string(APPEND HEADER "#pragma once\n\n")
string(APPEND HEADER "namespace shaders {\n\n")

foreach(SHADER_FILE ${SHADER_FILES})
    file(READ "${SHADER_FILE}" CONTENT)
    get_filename_component(FILE_NAME "${SHADER_FILE}" NAME)
    string(REPLACE "." "_" VAR_NAME "${FILE_NAME}")
    string(APPEND HEADER "inline const char* ${VAR_NAME} = R\"glsl(\n${CONTENT})glsl\"${SEMICOLON}\n\n")
endforeach()

string(APPEND HEADER "} // namespace shaders\n")

file(WRITE "${OUTPUT}" "${HEADER}")
