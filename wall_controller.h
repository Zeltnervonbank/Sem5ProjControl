#ifndef WALL_CONTROLLER_H
#define WALL_CONTROLLER_H

#include <fl/Headers.h>
#include "camera.h"

struct WallControlOutput
{
    float direction;
    float speed;
};


class wall_Controller
{
public:
    wall_Controller();
    virtual ~wall_Controller() = default;
    virtual void buildController();
    virtual WallControlOutput getControlOutput(float dirr, float dist);
protected:
    fl::Engine*          wall_Engine;
    fl::InputVariable*   wall_Direction;
    fl::InputVariable*   wall_Distance;
    fl::OutputVariable*  steer_Direction;
    fl::OutputVariable*  speed;
};


#endif // WALL_CONTROLLER_H
