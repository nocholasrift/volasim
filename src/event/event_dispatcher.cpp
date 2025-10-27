#include <volasim/event/event_dispatcher.h>
#include <algorithm>
#include <iostream>
#include <tuple>
#include <utility>

EventDispatcher::EventDispatcher() {}

EventDispatcher::~EventDispatcher() {
  //TODO: Find out if map destructor automatically frees container memory.
  for (auto it = listeners.begin(); it != listeners.end(); ++it) {
    std::vector<EventListener*> l = it->second;

    for (EventListener* listener : l) {
      // cerr << "addr of stuff: " << it->first << " " << listener << endl;
      if (listener) {
        //delete listener;
        listener = NULL;
      }
    }
    l.clear();
  }
}

void EventDispatcher::addEventListener(EventListener* l,
                                       std::string eventType) {
  // if(!(*listeners)[eventType])
  // 	(*listeners)[eventType] = new std::vector<EventListener*>;
  if (listeners.find(eventType) == listeners.end()) {
    auto tuple = std::pair<std::string, std::vector<EventListener*>>(
        eventType, std::vector<EventListener*>());
    listeners.insert(tuple);
  }

  auto vec = listeners[eventType];
  auto it = vec.begin();
  if ((it = find(vec.begin(), vec.end(), l)) == vec.end())
    listeners[eventType].push_back(l);
}

void EventDispatcher::removeEventListener(EventListener* l,
                                          std::string eventType) {
  if (listeners.find(eventType) != listeners.end()) {
    std::vector<EventListener*>& vec = listeners[eventType];
    for (auto it = vec.begin(); it != vec.end(); ++it) {
      if (*it == l) {
        // delete *it;
        auto loc = *it;
        vec.erase(it);
        if (find(vec.begin(), vec.end(), loc) != listeners[eventType].end())
          std::cerr << "LISTER WAS NOT PROPERLY REMOVED FROM EVENTLIST\n";
        return;
      }
    }
  }
}

bool EventDispatcher::hasEventListener(EventListener* l,
                                       std::string eventType) {
  auto it = listeners.find(eventType);
  return it != listeners.end();
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
