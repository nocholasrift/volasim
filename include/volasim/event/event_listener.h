#ifndef EVENTLISTENER_H
#define EVENTLISTENER_H

#include <volasim/event/event.h>

class Event;
class EventDispatcher;

class EventListener {

public:

	virtual void handleEvent(Event* e) = 0;

private:	
};

#endif

