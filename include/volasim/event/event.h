#ifndef EVENT_H
#define EVENT_H

#include <string>

#include <volasim/event/event_dispatcher.h>
#include <volasim/simulation/display_object.h>

class EventDispatcher;

class Event{

public:

	Event(std::string type, EventDispatcher* source){
    event_type_ = type;
    source_ = source;
  }

  std::string getType(){
    return event_type_;
  }

	EventDispatcher* getSource(){
    return source_;
  }
	
private:
  std::string event_type_ = "";
	EventDispatcher* source_;
};

class DisplayEvent : public Event{
public:
	DisplayEvent(std::string type, EventDispatcher* source, DisplayObject* do1) : Event(type, source){
		this->added_object_ = do1;
	}

	~DisplayEvent(){
		
	}

	DisplayObject* getAddedObject(){
		return this->added_object_;
	}
private:
	DisplayObject* added_object_;
};

#endif

