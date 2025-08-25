#ifndef PHYSICSINTERFACE_H
#define PHYSICSINTERFACE_H

// You can use Jolt.h in your precompiled header to speed up compilation.
#include <Jolt/Jolt.h>

// Jolt includes
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <iostream>
#include <memory>
#include <unordered_map>

#include <volasim/event/event_listener.h>
#include <volasim/simulation/dynamic_object.h>

class DynamicObject;

// Disable common warnings triggered by Jolt, you can use JPH_SUPPRESS_WARNING_PUSH / JPH_SUPPRESS_WARNING_POP to store and restore the warning state
JPH_SUPPRESS_WARNINGS

// Callback for traces, connect this to your own trace function if you have one
static void TraceImpl(const char* inFMT, ...) {
  // Format the message
  // va_list list;
  //  JPH::literals::va_start(list, inFMT);
  // char buffer[1024];
  // vsnprintf(buffer, sizeof(buffer), inFMT, list);
  //  JPH::literals::va_end(list);

  // Print to the TTY
  std::cout << "" << std::endl;
}

#ifdef JPH_ENABLE_ASSERTS

// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(const char* inExpression, const char* inMessage,
                             const char* inFile, uint inLine) {
  // Print to the TTY
  std::cout << inFile << ":" << inLine << ": (" << inExpression << ") "
            << (inMessage != nullptr ? inMessage : "") << std::endl;

  // Breakpoint
  return true;
};

#endif

// Layer that objects can be in, determines which other objects it can collide with
// Typically you at least want to have 1 layer for moving bodies and 1 layer for static bodies, but you can have more
// layers if you want. E.g. you could have a layer for high detail collision (which is not used by the physics simulation
// but only if you do collision testing).
namespace Layers {
static constexpr JPH::ObjectLayer NON_MOVING = 0;
static constexpr JPH::ObjectLayer MOVING = 1;
static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
};  // namespace Layers

/// Class that determines if two object layers can collide
class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter {
 public:
  virtual bool ShouldCollide(JPH::ObjectLayer inObject1,
                             JPH::ObjectLayer inObject2) const override {
    switch (inObject1) {
      case Layers::NON_MOVING:
        return inObject2 ==
               Layers::MOVING;  // Non moving only collides with moving
      case Layers::MOVING:
        return true;  // Moving collides with everything
      default:
        // JPH::JPH_ASSERT(false);
        return false;
    }
  }
};

// Each broadphase layer results in a separate bounding volume tree in the broad phase. You at least want to have
// a layer for non-moving and moving objects to avoid having to update a tree full of static objects every frame.
// You can have a 1-on-1 mapping between object layers and broadphase layers (like in this case) but if you have
// many object layers you'll be creating many broad phase trees, which is not efficient. If you want to fine tune
// your broadphase layers define JPH_TRACK_BROADPHASE_STATS and look at the stats reported on the TTY.
namespace BroadPhaseLayers {
static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
static constexpr JPH::BroadPhaseLayer MOVING(1);
static constexpr uint NUM_LAYERS(2);
};  // namespace BroadPhaseLayers

// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
 public:
  BPLayerInterfaceImpl() {
    // Create a mapping table from object to broad phase layer
    mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
    mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
  }

  virtual uint GetNumBroadPhaseLayers() const override {
    return BroadPhaseLayers::NUM_LAYERS;
  }

  virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(
      JPH::ObjectLayer inLayer) const override {
    // JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
    return mObjectToBroadPhase[inLayer];
  }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
  virtual const char* GetBroadPhaseLayerName(
      JPH::BroadPhaseLayer inLayer) const override {
    switch ((JPH::BroadPhaseLayer::Type)inLayer) {
      case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING:
        return "NON_MOVING";
      case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:
        return "MOVING";
      default:
        JPH_ASSERT(false);
        return "INVALID";
    }
  }
#endif  // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED
 private:
  JPH::BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

/// Class that determines if an object layer can collide with a broadphase layer
class ObjectVsBroadPhaseLayerFilterImpl
    : public JPH::ObjectVsBroadPhaseLayerFilter {
 public:
  virtual bool ShouldCollide(JPH::ObjectLayer inLayer1,
                             JPH::BroadPhaseLayer inLayer2) const override {
    switch (inLayer1) {
      case Layers::NON_MOVING:
        return inLayer2 == BroadPhaseLayers::MOVING;
      case Layers::MOVING:
        return true;
      default:
        JPH_ASSERT(false);
        return false;
    }
  }
};

// An example contact listener
class MyContactListener : public JPH::ContactListener {
 public:
  // See: ContactListener
  virtual JPH::ValidateResult OnContactValidate(
      const JPH::Body& inBody1, const JPH::Body& inBody2,
      JPH::RVec3Arg inBaseOffset,
      const JPH::CollideShapeResult& inCollisionResult) override {
    // std::cout << "Contact validate callback" << std::endl;

    // Allows you to ignore a contact before it is created (using layers to not make objects collide is cheaper!)
    return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
  }

  virtual void OnContactAdded(const JPH::Body& inBody1,
                              const JPH::Body& inBody2,
                              const JPH::ContactManifold& inManifold,
                              JPH::ContactSettings& ioSettings) override {
    std::cout << "A contact was added" << std::endl;
    is_colliding_ = true;
  }

  virtual void OnContactPersisted(const JPH::Body& inBody1,
                                  const JPH::Body& inBody2,
                                  const JPH::ContactManifold& inManifold,
                                  JPH::ContactSettings& ioSettings) override {
    // std::cout << "A contact was persisted" << std::endl;
    // exit(0);
  }

  virtual void OnContactRemoved(
      const JPH::SubShapeIDPair& inSubShapePair) override {
    std::cout << "A contact was removed" << std::endl;
    is_colliding_ = false;
  }

  bool is_colliding_ = false;
};

// An example activation listener
class MyBodyActivationListener : public JPH::BodyActivationListener {
 public:
  virtual void OnBodyActivated(const JPH::BodyID& inBodyID,
                               JPH::uint64 inBodyUserData) override {
    // std::cout << "A body got activated" << std::endl;
  }

  virtual void OnBodyDeactivated(const JPH::BodyID& inBodyID,
                                 JPH::uint64 inBodyUserData) override {
    // std::cout << "A body went to sleep " << std::endl;
  }
};

struct PhysicsBinding {
  DynamicObject* dynamic_obj = nullptr;
  JPH::BodyID body_id;

  bool isDynamic() const { return dynamic_obj != nullptr; }
  bool isValid() const { return !body_id.IsInvalid(); }

  JPH::ObjectLayer getLayer() const {
    return isDynamic() ? Layers::MOVING : Layers::NON_MOVING;
  }

  JPH::EMotionType getMotionType() const {
    return isDynamic() ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static;
  }
};

class PhysicsInterface : public EventListener {
 public:
  // singleton pattern
  static PhysicsInterface& getInstance() {
    static PhysicsInterface instance;
    return instance;
  }

  PhysicsInterface();
  ~PhysicsInterface();

  virtual void handleEvent(Event* e) override;

  void update(double dt);
  void preRegister(DisplayObject* display_obj, DynamicObject* dynamic_obj);

  std::vector<DynamicObject*> getDynamicObjects() {
    std::vector<DynamicObject*> dyna_objs;

    for (auto& [disp_obj, binding] : disp_to_dyna_) {
      if (binding.isDynamic()) {
        dyna_objs.push_back(binding.dynamic_obj);
      }
    }

    return dyna_objs;
  }

 private:
  std::unique_ptr<JPH::PhysicsSystem> physics_system_;

  std::unordered_map<DisplayObject*, PhysicsBinding> disp_to_dyna_;

  MyContactListener contact_listener_;
  BPLayerInterfaceImpl broad_phase_layer_interface_;
  MyBodyActivationListener body_activation_listener_;
  ObjectLayerPairFilterImpl object_vs_object_layer_filter_;
  ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter_;

  std::unique_ptr<JPH::TempAllocatorImpl> temp_allocator_;
  std::unique_ptr<JPH::JobSystemThreadPool> job_system_;
};

#endif
