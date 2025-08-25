#include <volasim/simulation/dynamic_object.h>
#include <volasim/simulation/physics_interface.h>
#include <volasim/simulation/shape_renderable.h>
#include <volasim/vehicles/drone.h>

#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

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
  physics_system_->SetGravity(JPH::Vec3(0., 0., -9.81));
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

  for (const auto& [disp_obj, binding] : disp_to_dyna_)
    delete binding.dynamic_obj;
}

void PhysicsInterface::update(double dt) {
  JPH::BodyInterface& body_interface = physics_system_->GetBodyInterface();

  // std::cout << "applying update at " << dt << " seconds\n";

  // 1. Apply movement to non-static objects
  for (const auto& [disp_obj, binding] : disp_to_dyna_) {

    if (binding.getMotionType() != JPH::EMotionType::Static) {
      /*Eigen::Vector4d u =*/
      /*    Eigen::Vector4d(1.0, 1.0, 1.01, 1.01) * 4.34 * 9.81 / 4.;*/

      /*binding.dynamic_obj->setInput(u);*/
      // binding.dynamic_obj->update(dt);

      JPH::RVec3 jolt_p = body_interface.GetPosition(binding.body_id);
      // JPH::Quat jolt_r;
      // body_interface.GetPositionAndRotation(binding.body_id, jolt_p, jolt_r);

      // std::cout << "setting rotation in interface as: " << jolt_r.GetX() << " "
      //           << jolt_r.GetY() << " " << jolt_r.GetZ() << " " << jolt_r.GetW()
      //           << std::endl;

      // glm::vec3 vel = binding.dynamic_obj->getVelocity();
      // glm::vec3 rot = binding.dynamic_obj->getBodyRates();
      // JPH::RVec3 jolt_v(vel[0], vel[1], vel[2]);
      // JPH::RVec3 jolt_w(rot[0], rot[1], rot[2]);
      // body_interface.SetLinearAndAngularVelocity(binding.body_id, jolt_v,
      //                                            jolt_w);
      Eigen::Vector3d force, rpy;
      binding.dynamic_obj->getForceAndTorque(force, rpy);

      JPH::Vec3 j_rpy(rpy[0], rpy[1], rpy[2]);
      JPH::Vec3 j_force(force[0], force[1], force[2]);
      body_interface.AddTorque(binding.body_id, j_rpy);
      body_interface.AddForce(binding.body_id, j_force, jolt_p);
    }
  }

  // 2. Step the physics system
  physics_system_->Update(dt, 1, temp_allocator_.get(), job_system_.get());

  // 3. Update DOs with the true physical system positions (in case of collision)
  for (const auto& [disp_obj, binding] : disp_to_dyna_) {

    if (binding.getMotionType() != JPH::EMotionType::Static) {

      JPH::RVec3 jolt_p;
      JPH::Quat jolt_r;
      body_interface.GetPositionAndRotation(binding.body_id, jolt_p, jolt_r);

      JPH::RVec3 jolt_v;
      JPH::RVec3 jolt_w;
      body_interface.GetLinearAndAngularVelocity(binding.body_id, jolt_v,
                                                 jolt_w);

      glm::vec3 pos(jolt_p[0], jolt_p[1], jolt_p[2]);
      glm::quat rot(jolt_r.GetW(), jolt_r.GetX(), jolt_r.GetY(), jolt_r.GetZ());

      // std::cout << "position: " << jolt_p[0] << " " << jolt_p[1] << " "
      //           << jolt_p[2] << "\n";

      binding.dynamic_obj->setTranslation(pos);
      binding.dynamic_obj->setRotation(rot);

      glm::vec3 vel(jolt_v[0], jolt_v[1], jolt_v[2]);
      glm::vec3 rpy(jolt_w[0], jolt_w[1], jolt_w[2]);

      binding.dynamic_obj->setVelocity(vel);
      binding.dynamic_obj->setAngularVelocity(rpy);

      disp_obj->setTranslation(pos);
      disp_obj->setRotation(rot);
    }
  }
}

void PhysicsInterface::preRegister(DisplayObject* display_obj,
                                   DynamicObject* dynamic_obj) {
  if (disp_to_dyna_.find(display_obj) == disp_to_dyna_.end()) {
    PhysicsBinding binding;
    binding.dynamic_obj = dynamic_obj;
    disp_to_dyna_.insert({display_obj, binding});
  }
}

// TODO: Relax requirement that must be renderable b4 adding to display tree
void PhysicsInterface::handleEvent(Event* e) {
  using namespace JPH::literals;

  DisplayObject* object = static_cast<DisplayEvent*>(e)->getAddedObject();
  if (e->getType() == "OBJ_ADD") {

    bool is_set = false;
    if (disp_to_dyna_.find(object) != disp_to_dyna_.end())
      is_set = disp_to_dyna_[object].isValid();
    else
      disp_to_dyna_.insert({object, PhysicsBinding()});

    if (!is_set && object->isRenderable()) {

      PhysicsBinding& binding = disp_to_dyna_[object];

      JPH::BodyID body_id;
      JPH::BodyCreationSettings shape_settings;
      JPH::BodyInterface& body_interface = physics_system_->GetBodyInterface();

      const ShapeRenderable& renderable =
          static_cast<const ShapeRenderable&>(object->getRenderable());

      glm::vec3 pos = object->getTranslation();
      glm::quat ori = object->getRotation();
      /*std::cout << "rotation dispobj: " << ori[0] << " " << ori[1] << " " << ori[2] << " " << ori[3] << "\n";*/
      ShapeMetadata shape_meta = renderable.getShapeMeta();

      JPH::ObjectLayer obj_layer =
          binding.isDynamic() ? Layers::MOVING : Layers::NON_MOVING;

      switch (renderable.getType()) {

        case ShapeType::kSphere:
          std::cout << "[physics interface] handling add sphere event"
                    << std::endl;
          break;

        case ShapeType::kCube:
          // std::cout << "[physics interface] handling add cube event"
          //           << std::endl;

          shape_settings = JPH::BodyCreationSettings(
              new JPH::BoxShape(JPH::Vec3(shape_meta.size / 2,
                                          shape_meta.size / 2,
                                          shape_meta.size / 2)),
              JPH::RVec3(pos[0], pos[1], pos[2]),
              JPH::Quat(ori[0], ori[1], ori[2], ori[3]),
              binding.getMotionType(), binding.getLayer());
          break;

        case ShapeType::kCylinder: {
          JPH::Quat align_rot = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 90.f);
          JPH::Quat renderable_ori(ori[1], ori[2], ori[3], ori[0]);
          shape_settings = JPH::BodyCreationSettings(
              new JPH::CylinderShape(shape_meta.height / 2., shape_meta.radius),
              JPH::RVec3(pos[0], pos[1], pos[2] + shape_meta.height / 2),
              // JPH::Quat(ori[1], ori[2], ori[3], ori[0]),
              align_rot * renderable_ori, binding.getMotionType(),
              binding.getLayer());

          break;
        }

        case ShapeType::kPlane: {
          // std::cout << "[physics interface] handling add plane event"
          //           << std::endl;

          // in case max is somehow smaller than min
          double half_width = fabs(shape_meta.x_max - shape_meta.x_min) / 2.;
          double half_height = fabs(shape_meta.y_max - shape_meta.y_min) / 2.;

          shape_settings = JPH::BodyCreationSettings(
              new JPH::BoxShape(JPH::Vec3(half_width, half_height, .5)),
              JPH::RVec3(pos[0], pos[1], pos[2] - 0.5),
              JPH::Quat(ori[1], ori[2], ori[3], ori[0]),
              binding.getMotionType(), binding.getLayer());
          break;
        }
      }

      if (binding.isDynamic()) {
        shape_settings.mOverrideMassProperties =
            JPH::EOverrideMassProperties::MassAndInertiaProvided;
        JPH::MassProperties mass_properties;
        mass_properties.mMass = binding.dynamic_obj->getMass();

        Eigen::Matrix3d inertia_mat = binding.dynamic_obj->getInertia();

        mass_properties.mInertia = JPH::Mat44::sIdentity();
        mass_properties.mInertia(0, 0) = inertia_mat(0, 0);
        mass_properties.mInertia(1, 1) = inertia_mat(1, 1);
        mass_properties.mInertia(2, 2) = inertia_mat(2, 2);

        shape_settings.mMassPropertiesOverride = mass_properties;
        // shape_settings.mGravityFactor = 0.f;
      }

      body_id = body_interface.CreateAndAddBody(shape_settings,
                                                JPH::EActivation::Activate);

      binding.body_id = body_id;
    }

  } else if (e->getType() == "OBJ_RM") {
  }
}
