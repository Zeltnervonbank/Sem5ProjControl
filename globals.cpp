#include "globals.h"

double Globals::GetDistanceToWaypoint()
{
    double xDisplacement = abs(LastPosition.posX - CurrentWaypoint.x);
    double yDisplacement = abs(LastPosition.posY - CurrentWaypoint.y);

    // Uses pythagorean theorem to determine absolute distance to waypoint
    return sqrt(pow(xDisplacement, 2) + pow(yDisplacement, 2));
}

void Globals::ClearWaypointQueue()
{
    std::queue<Waypoint> empty;
    std::swap(waypoints, empty);
}

void Globals::ClearDestinationQueue()
{
    std::queue<Waypoint> empty;
    std::swap(destinationQueue, empty);
}
