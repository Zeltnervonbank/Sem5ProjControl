#ifndef WAYPOINTCONTROLLER_H
#define WAYPOINTCONTROLLER_H

#include <fl/Headers.h>



struct waypointControlOutput
{
    float direction;
    float speed;
};

class waypointController
{
public:
    struct Waypoint
    {
        double x;
        double y;
    };
    static std::queue<Waypoint> waypoints;
    static Waypoint CurrentWaypoint;

    waypointController();
    virtual ~waypointController() = default;
    virtual void buildController();
    virtual ControlOutput getControlOutput();
    static double GetDistanceToWaypoint();

protected:

    fl::Engine*          waypointEngine;
    fl::InputVariable*   waypointDirection;
    fl::InputVariable*   waypointDistance;
    fl::OutputVariable*  SteerDirection;
    fl::OutputVariable*  Speed;
    float center;
};


#endif // WAYPOINTCONTROLLER_H
