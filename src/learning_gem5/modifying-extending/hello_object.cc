#include "learning_gem5/modifying-extending/hello_object.hh"

#include <iostream>

HelloObject::HelloObject(HelloObjectParams *params) :
    SimObject(params)
{
    std::cout << "Hello World! From a SimObject!" << std::endl;
}

HelloObject*
HelloObjectParams::create()
{
    return new HelloObject(this);
}

