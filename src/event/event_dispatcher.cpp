#include <volasim/event/event_dispatcher.h>
#include <algorithm>
#include <iostream>
#include <tuple>
#include <utility>

#include <volasim/event/event.h>
#include <volasim/event/event_listener.h>

void EventDispatcher::addEventListener(EventListener*     l,
                                       const std::string& eventType) {
  // if(!(*listeners)[eventType])
  // 	(*listeners)[eventType] = new std::vector<EventListener*>;
  if (listeners.find(eventType) == listeners.end()) {
    auto tuple = std::pair<std::string, std::vector<EventListener*>>(
        eventType, std::vector<EventListener*>());
    listeners.insert(tuple);
  }

  if (!hasEventListener(l, eventType)) {
    listeners[eventType].push_back(l);
  }
}

void EventDispatcher::removeEventListener(EventListener*     l,
                                          const std::string& eventType) {
  if (listeners.find(eventType) != listeners.end()) {
    std::vector<EventListener*>& vec = listeners[eventType];
    for (auto it = vec.begin(); it != vec.end(); ++it) {
      if (*it == l) {
        // delete *it;
        auto* loc = *it;
        vec.erase(it);
        if (find(vec.begin(), vec.end(), loc) != listeners[eventType].end())
          std::cerr << "LISTER WAS NOT PROPERLY REMOVED FROM EVENTLIST\n";
        return;
      }
    }
  }
}

bool EventDispatcher::hasEventListener(EventListener*     l,
                                       const std::string& eventType) {
  auto it = listeners.find(eventType);
  if (it == listeners.end())
    return false;

  const auto& vec = it->second;
  return std::find(vec.begin(), vec.end(), l) != vec.end();
}

void EventDispatcher::dispatchEvent(Event* e) {
  if (listeners.count(e->getType()) == 0) {
    std::cerr << "No listener with event type\n";
    return;
  }

  std::vector<EventListener*>& l = listeners[e->getType()];
  for (EventListener* listener : l) {
    listener->handleEvent(e);
  }
}
