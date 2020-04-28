#include "learning_gem5/modifying-extending/hello_object.hh"
#include "debug/Hello.hh"

//#include <iostream>

HelloObject::HelloObject(HelloObjectParams *params) :
    SimObject(params), event([this]{processEvent();}, name()),
    latency(100), timesLeft(10)
{
    //std::cout << "Hello World! From a SimObject!" << std::endl;
    DPRINTF(Hello, "Created the hello object\n");
}

HelloObject*
HelloObjectParams::create()
{
    return new HelloObject(this);
}

void
HelloObject::processEvent()
{
    timesLeft--;
    DPRINTF(Hello, "Hello world! Processing the event! %d left\n", timesLeft);

    if (timesLeft <= 0) {
        DPRINTF(Hello, "Done firing!\n");
    }
    else {
        schedule(event, curTick() + latency);
    }
}

void HelloObject::startup()
{
    schedule(event, 100);
}
