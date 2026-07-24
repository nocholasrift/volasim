#ifndef EVENTDISPATCHER_H
#define EVENTDISPATCHER_H

#include <string>
#include <unordered_map>
#include <vector>

// #include <volasim/event/event.h>
// #include <volasim/event/event_listener.h>

class EventListener;
class Event;

class EventDispatcher {

 public:
  // singleton pattern
  static EventDispatcher& getInstance() {
    static EventDispatcher instance;
    return instance;
  }

  EventDispatcher() = default;
  ~EventDispatcher() = default;

  EventDispatcher(const EventDispatcher&)            = delete;
  EventDispatcher& operator=(const EventDispatcher&) = delete;
  EventDispatcher(EventDispatcher&&)                 = delete;
  EventDispatcher& operator=(EventDispatcher&&)      = delete;

  void addEventListener(EventListener* l, const std::string& eventType);
  void removeEventListener(EventListener* l, const std::string& eventType);
  bool hasEventListener(EventListener* l, const std::string& eventType);
  void dispatchEvent(Event* e);

 private:
  /* List of listeners */
  std::unordered_map<std::string, std::vector<EventListener*>> listeners;
};

#endif
