#ifndef ENTITY_ID_H
#define ENTITY_ID_H

#include <cstdint>

// Split from entity.h so that code keyed by entity identity — the pose buffer,
// above all — does not have to pull in the scene graph and its GL dependencies.
using EntityID = std::uint32_t;

#endif
