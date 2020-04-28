#include "learning_gem5/modifying-extending/hello_object.hh"
#include "debug/Hello.hh"

HelloObject::HelloObject(HelloObjectParams *params) :
    SimObject(params),
    event([this]{processEvent();}, name()),
    //event(*this),
    goodbye(params->goodbye_object),
    myName(params->name),
    latency(params->time_to_wait),
    timesLeft(params->number_of_fires)
{
    //std::cout << "Hello World! From a SimObject!" << std::endl;
    DPRINTF(Hello, "Created the hello object with the name %s\n", myName);
    panic_if(!goodbye, "HelloObject must have a non-null GoodbyeObject");
}

void HelloObject::startup()
{
    schedule(event, latency);
}

void
HelloObject::processEvent()
{
    timesLeft--;
    DPRINTF(Hello, "Hello world! Processing the event! %d left\n", timesLeft);

    if (timesLeft <= 0) {
        DPRINTF(Hello, "Done firing!\n");
        goodbye->sayGoodbye(myName);
    }
    else {
        schedule(event, curTick() + latency);
    }
}

HelloObject*
HelloObjectParams::create()
{
    return new HelloObject(this);
}
