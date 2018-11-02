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
    virtual ControlOutput getControlOutput(int cent);
protected:
    //LaserScanner*        m_pcLaserScanner;

    fl::Engine*          m_pcFLEngine;
    fl::InputVariable*   m_pflObstacleDirection;
    fl::InputVariable*   m_pflObstacleDistance;
    fl::OutputVariable*  m_pflSteerDirection;
    fl::OutputVariable*  m_pflSpeed;
    float center;
};


#endif // MARBEL_CONTROLLER_H
