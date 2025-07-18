#include <volasim/vehicles/drone.h>
#include <volasim/simulation/dynamic_object.h>
#include <volasim/simulation/physics_interface.h>
#include <volasim/simulation/shape_renderable.h>

PhysicsInterface::PhysicsInterface() {
  JPH::RegisterDefaultAllocator();

  // Install trace and assert callbacks
  JPH::Trace = TraceImpl;

  JPH::Factory::sInstance = new JPH::Factory();

  // Register all physics types with the factory and install their collision handlers with the CollisionDispatch class.
  // If you have your own custom shape types you probably need to register their handlers with the CollisionDispatch before calling this function.
  // If you implement your own default material (PhysicsMaterial::sDefault) make sure to initialize it before this function or else this function will create one for you.
  JPH::RegisterTypes();

  // We need a temp allocator for temporary allocations during the physics update. We're
  // pre-allocating 10 MB to avoid having to do allocations during the physics update.
  // B.t.w. 10 MB is way too much for this example but it is a typical value you can use.
  // If you don't want to pre-allocate you can also use TempAllocatorMalloc to fall back to
  // malloc / free.
  temp_allocator_ = std::make_unique<JPH::TempAllocatorImpl>(10 * 1024 * 1024);

  // We need a job system that will execute physics jobs on multiple threads. Typically
  // you would implement the JobSystem interface yourself and let Jolt Physics run on top
  // of your own job scheduler. JobSystemThreadPool is an example implementation.
  job_system_ = std::make_unique<JPH::JobSystemThreadPool>(
      JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers,
      JPH::thread::hardware_concurrency() - 1);

  // This is the max amount of rigid bodies that you can add to the physics system. If you try to add more you'll get an error.
  // Note: This value is low because this is a simple test. For a real project use something in the order of 65536.
  const uint cMaxBodies = 1024;

  // This determines how many mutexes to allocate to protect rigid bodies from concurrent access. Set it to 0 for the default settings.
  const uint cNumBodyMutexes = 0;

  // This is the max amount of body pairs that can be queued at any time (the broad phase will detect overlapping
  // body pairs based on their bounding boxes and will insert them into a queue for the narrowphase). If you make this buffer
  // too small the queue will fill up and the broad phase jobs will start to do narrow phase work. This is slightly less efficient.
  // Note: This value is low because this is a simple test. For a real project use something in the order of 65536.
  const uint cMaxBodyPairs = 1024;

  // This is the maximum size of the contact constraint buffer. If more contacts (collisions between bodies) are detected than this
  // number then these contacts will be ignored and bodies will start interpenetrating / fall through the world.
  // Note: This value is low because this is a simple test. For a real project use something in the order of 10240.
  const uint cMaxContactConstraints = 1024;

  // Now we can create the actual physics system.
  physics_system_ = std::make_unique<JPH::PhysicsSystem>();
  // PhysicsSystem physics_system;
  physics_system_->Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs,
                        cMaxContactConstraints, broad_phase_layer_interface_,
                        object_vs_broadphase_layer_filter_,
                        object_vs_object_layer_filter_);

  // A body activation listener gets notified when bodies activate and go to sleep
  // Note that this is called from a job so whatever you do here needs to be thread safe.
  // Registering one is entirely optional.
  physics_system_->SetBodyActivationListener(&body_activation_listener_);

  // A contact listener gets notified when bodies (are about to) collide, and when they separate again.
  // Note that this is called from a job so whatever you do here needs to be thread safe.
  // Registering one is entirely optional.
  physics_system_->SetContactListener(&contact_listener_);

  // The main way to interact with the bodies in the physics system is through the body interface. There is a locking and a non-locking
  // variant of this. We're going to use the locking version (even though we're not planning to access bodies from multiple threads)
  JPH::BodyInterface& body_interface = physics_system_->GetBodyInterface();
}

PhysicsInterface::~PhysicsInterface() {

  delete JPH::Factory::sInstance;
  JPH::Factory::sInstance = nullptr;
}

void PhysicsInterface::update(double dt) {
  JPH::BodyInterface& body_interface = physics_system_->GetBodyInterface();

  // 1. Apply movement to non-static objects
  for (auto it = str_to_body_id_.begin(); it != str_to_body_id_.end(); ++it) {

    if (body_interface.GetMotionType(it->second) !=
        JPH::EMotionType::Static) {
      // DynamicDisplayWrapper<13, 4>* obj =
      //     dynamic_cast<DynamicDisplayWrapper<13, 4>*>(display_obj_lookup_[it->first]);
      DisplayObject* obj = display_obj_lookup_[it->first];
      glm::vec3 pos = obj->getPosition();
      glm::quat rot = obj->getRotation();

      JPH::RVec3 jolt_p(pos[0], pos[1], pos[2]);
      JPH::Quat jolt_r(rot[1], rot[2], rot[3], rot[0]);
      body_interface.MoveKinematic(it->second, jolt_p, jolt_r, dt);

      // std::cout << "? " << display_obj_lookup_[it->first] << std::endl;
      // std::cout << "? " << obj << std::endl;
      // glm::vec3 tv = obj->getDynaObj().getVelocity();
      // glm::quat wv = obj->getDynaObj().getBodyRates();
      // std::cout << "??" << std::endl;
      // body_interface.SetLinearAndAngularVelocity(it->second, 
      //     JPH::Vec3(tv[0], tv[1], tv[2]), 
      //     JPH::Vec3(wv[0], wv[1], wv[2]));
    }
  }

  // 2. Step the physics system
  physics_system_->Update(dt, 1, temp_allocator_.get(), job_system_.get());

  // 3. Update DOs with the true physical system positions (in case of collision)
  for (const auto& [str_id, body_id] : str_to_body_id_) {

    if (body_interface.GetMotionType(body_id) != JPH::EMotionType::Static) {
      DisplayObject* obj = display_obj_lookup_[str_id];

      JPH::RVec3 jolt_p;
      JPH::Quat jolt_r;
      body_interface.GetPositionAndRotation(body_id, jolt_p, jolt_r);

      glm::vec3 pos(jolt_p[0], jolt_p[1], jolt_p[2]);
      glm::quat rot(jolt_r.GetW(), jolt_r.GetX(), jolt_r.GetY(), jolt_r.GetZ());

      obj->setTranslation(pos);
      obj->setRotation(rot);
    }
  }
}

// TODO: Relax requirement that must be renderable b4 adding to display tree
void PhysicsInterface::handleEvent(Event* e) {
  using namespace JPH::literals;

  DisplayObject* object = static_cast<DisplayEvent*>(e)->getAddedObject();
  if (e->getType() == "OBJ_ADD") {
    if (str_to_body_id_.find(object->getID()) == str_to_body_id_.end() &&
        object->isRenderable()) {

      JPH::BodyID body_id;
      JPH::BodyCreationSettings shape_settings;
      JPH::BodyInterface& body_interface = physics_system_->GetBodyInterface();

      const ShapeRenderable& renderable =
          static_cast<const ShapeRenderable&>(object->getRenderable());

      glm::vec3 pos = object->getPosition();
      glm::quat ori = object->getRotation();
      ShapeMetadata shape_meta = renderable.getShapeMeta();

      switch (renderable.getType()) {

        case ShapeType::kSphere:
          std::cout << "[physics interface] handling add sphere event"
                    << std::endl;
          break;

        case ShapeType::kCube:
          std::cout << "[physics interface] handling add cube event"
                    << std::endl;

          shape_settings = JPH::BodyCreationSettings(
              new JPH::BoxShape(JPH::Vec3(shape_meta.scale, shape_meta.scale,
                                          shape_meta.scale)),
              JPH::RVec3(pos[0], pos[1], pos[2]),
              JPH::Quat(ori[1], ori[2], ori[3], ori[0]),
              JPH::EMotionType::Dynamic, Layers::MOVING);
          body_id = body_interface.CreateAndAddBody(shape_settings,
                                                    JPH::EActivation::Activate);
          body_interface.SetGravityFactor(body_id, 0.);
          break;

        case ShapeType::kCylinder:
          break;

        case ShapeType::kPlane:
          std::cout << "[physics interface] handling add plane event"
                    << std::endl;

          shape_settings = JPH::BodyCreationSettings(
              new JPH::BoxShape(JPH::Vec3(50, 50, 1.0)),
              JPH::RVec3(pos[0], pos[1], pos[2]-0.5),
              JPH::Quat(ori[1], ori[2], ori[3], ori[0]),
              JPH::EMotionType::Static, Layers::NON_MOVING);
          body_id = body_interface.CreateAndAddBody(shape_settings,
                                                    JPH::EActivation::Activate);
          break;
      }

      std::pair<std::string, JPH::BodyID> obj_map(object->getID(), body_id);
      str_to_body_id_.insert(obj_map);

      std::pair<std::string, DisplayObject*> id_obj_pair(object->getID(),
                                                         object);
      display_obj_lookup_.insert(id_obj_pair);
    }

  } else if (e->getType() == "OBJ_RM") {
    std::cout << "[physics interface] handling rm event" << std::endl;
  }
}
