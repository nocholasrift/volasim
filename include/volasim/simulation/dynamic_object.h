#ifndef DYNAMICOBJECT_H
#define DYNAMICOBJECT_H

#include <memory>
#include <regex>
#include <volasim/solvers/runge_kutta.h>
#include <volasim/simulation/display_object_container.h>

template <int N, int M>
class DynamicObject{
  public: 
    DynamicObject(double dt) : dt_(dt){}
    virtual ~DynamicObject(){}

    // dynamics function
    virtual void update(const Eigen::Vector<double, M>& u, double dt) = 0;
    virtual Eigen::Vector<double, N> dynamics(const Eigen::Vector<double, N>& x,
                                              const Eigen::Vector<double, M>& u) = 0;

    virtual void setTranslation(const glm::vec3& tran) = 0;
    virtual void setRotation(const glm::quat& rot) = 0;

    virtual glm::vec3 getTranslation() = 0;
    virtual glm::quat getRotation() = 0;

    virtual glm::vec3 getVelocity() = 0;
    virtual glm::vec3 getBodyRates() = 0;

  protected:
    std::unique_ptr<amrl::RungeKutta<N, M>> solver_;
    double dt_;
};

template<int N, int M>
class DynamicDisplayWrapper : public DisplayObjectContainer {
  public:
    DynamicDisplayWrapper(std::unique_ptr<DynamicObject<N, M> >& obj) : obj_(std::move(obj)) {}

    DynamicDisplayWrapper(std::string id, std::unique_ptr<DynamicObject<N, M> >& obj) : DisplayObjectContainer(id) {
        obj_ = std::move(obj);
        DisplayObjectContainer::setTranslation(obj_->getTranslation());
        DisplayObjectContainer::setRotation(obj_->getRotation());
    }

    virtual ~DynamicDisplayWrapper(){}

    virtual void setRenderable(ShapeType type, const ShapeMetadata& meta) override{
      DisplayObjectContainer::setRenderable(type, meta);
    }

    void update(const Eigen::Vector<double, M>& u, double dt){
      DisplayObjectContainer::update();

      obj_->update(u, dt);
      DisplayObjectContainer::setTranslation(obj_->getTranslation());
      DisplayObjectContainer::setRotation(obj_->getRotation());
    }

    virtual void draw() override{
      DisplayObjectContainer::draw();
    }

    DynamicObject<N,M>& getDynaObj(){ return *obj_; }

  protected:
    std::unique_ptr<DynamicObject<N,M> > obj_;
};

#endif
