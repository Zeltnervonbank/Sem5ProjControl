#ifndef WAYPOINTCONTROLLER_H
#define WAYPOINTCONTROLLER_H

#include <fl/Headers.h>
#include "globals.h"


struct waypointControlOutput
{
    float direction;
    float speed;
};

class waypointController
{
public:
    waypointController();
    virtual ~waypointController() = default;
    virtual void buildController();
    virtual ControlOutput getControlOutput();

protected:

    fl::Engine*          waypointEngine;
    fl::InputVariable*   waypointDirection;
    fl::InputVariable*   waypointDistance;
    fl::OutputVariable*  SteerDirection;
    fl::OutputVariable*  Speed;
    float center;
};


#endif // WAYPOINTCONTROLLER_H
