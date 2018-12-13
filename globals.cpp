#include "globals.h"

double Globals::GetDistanceToWaypoint()
{
    double xDisplacement = abs(lastPosition.posX - currentWaypoint.x);
    double yDisplacement = abs(lastPosition.posY - currentWaypoint.y);

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
    std::queue<Destination> empty;
    std::swap(destinationQueue, empty);
}

void Globals::NextDestination()
{
    previousDestination = currentDestination;
    currentDestination = destinationQueue.front();
    destinationQueue.pop();
}

void Globals::NextWaypoint()
{
    currentWaypoint = waypoints.front();
    waypoints.pop();
}
