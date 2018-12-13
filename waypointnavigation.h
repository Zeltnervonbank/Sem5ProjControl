#ifndef WAYPOINTNAVIGATION_H
#define WAYPOINTNAVIGATION_H

#include <queue>
#include <iostream>

#include "globals.h"
#include "movement.h"

class WaypointNavigation
{
public:
    WaypointNavigation();

    static void NavigateToNextWaypoint();
    static void MoveTowardWaypoint();
};

#endif // WAYPOINTNAVIGATION_H
