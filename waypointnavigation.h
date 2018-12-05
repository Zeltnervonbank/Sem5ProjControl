#ifndef WAYPOINTNAVIGATION_H
#define WAYPOINTNAVIGATION_H

#include <queue>
#include <iostream>

#include "globals.h"
#include "movement.h"

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
    static Waypoint CurrentWaypoint;
    static void NavigateToNextWaypoint();


    static void MoveTowardWaypoint();
    static double GetDistanceToWaypoint();
};

#endif // WAYPOINTNAVIGATION_H
