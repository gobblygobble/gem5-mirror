#ifndef __LEARNING_GEM5_HELLO_OBJECT_HH__
#define __LEARNING_GEM5_HELLO_OBJECT_HH__

#include "params/HelloObject.hh"
#include "sim/sim_object.hh"

class HelloObject : public SimObject
{
    private:
        // function we want to execute every time the event fires
        void processEvent();
        // Event instasnce. EventFunctionWrapper allows us to execute any function
        EventFunctionWrapper event;
        // latency between calling events (in ticks)
        Tick latency;
        // number of times to fire
        int timesLeft;

    public:
        HelloObject(HelloObjectParams *p);
        // part of initialization
        // called after all SimObjects constructed
        void startup();
};

#endif // __LEARNING_GEM5_HELLO_OBJECT_HH__

