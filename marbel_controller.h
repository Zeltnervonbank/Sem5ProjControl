#ifndef MARBEL_CONTROLLER_H
#define MARBEL_CONTROLLER_H

#include <fl/Headers.h>
#include "camera.h"


struct ControlOutput
{
    float direction;
    float speed;
};

class marbel_Controller
{
public:
    marbel_Controller();
    virtual ~marbel_Controller() = default;
    virtual void buildController();
    virtual ControlOutput getControlOutput(float cent, float dist);
protected:

    fl::Engine*          marbleEngine;
    fl::InputVariable*   marbleDirection;
    fl::InputVariable*   marbleDistance;
    fl::OutputVariable*  SteerDirection;
    fl::OutputVariable*  Speed;
    float center;
};


#endif // MARBEL_CONTROLLER_H
