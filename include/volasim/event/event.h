#ifndef EVENT_H
#define EVENT_H

#include <string>
#include <utility>

#include <volasim/event/event_dispatcher.h>
#include <volasim/simulation/entity.h>

class EventDispatcher;

class Event {

 public:
  Event(std::string type, EventDispatcher* source)
      : event_type_(std::move(type)), source_(source) {}

  std::string getType() { return event_type_; }

  EventDispatcher* getSource() { return source_; }

 private:
  std::string      event_type_;
  EventDispatcher* source_;
};

class DisplayEvent : public Event {
 public:
  DisplayEvent(std::string type, EventDispatcher* source, Entity* entity)
      : Event(std::move(type), source), added_entity_(entity) {}

  Entity* getAddedObject() { return this->added_entity_; }

 private:
  Entity* added_entity_;
};

#endif
