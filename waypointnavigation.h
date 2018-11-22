#ifndef WAYPOINTNAVIGATION_H
#define WAYPOINTNAVIGATION_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <queue>

class WaypointNavigation
{    
public:
    struct Waypoint
    {
        double x;
        double y;
    };

    WaypointNavigation();
    static std::queue<Waypoint> waypoints;

};

#endif // WAYPOINTNAVIGATION_H