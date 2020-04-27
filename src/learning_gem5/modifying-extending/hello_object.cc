#include "learning_gem5/modifying-extending/hello_object.hh"
#include "debug/Hello.hh"

//#include <iostream>

HelloObject::HelloObject(HelloObjectParams *params) :
    SimObject(params)
{
    //std::cout << "Hello World! From a SimObject!" << std::endl;
    DPRINTF(Hello, "Created the hello object\n");
}

HelloObject*
HelloObjectParams::create()
{
    return new HelloObject(this);
}

