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
    if(waypoints.size() > 0)
    {
        currentWaypoint = waypoints.front();
        waypoints.pop();
    }
}

double Globals::GetRobotYaw()
{
    return fmod(2.0 * atan2(lastPosition.rotW, lastPosition.rotZ) + 3.0 * M_PI, 2.0 * M_PI);
}

double Globals::AngleBetweenVectors(double angleA, double magnitudeA, double angleB, double magnitudeB)
{
    // Get dot product of vectors
    double dot = cos(angleA) * cos(angleB) + sin(angleA) * sin(angleA);

    // Get cross product of vectors
    double cross = cos(angleA) * sin(angleB) - cos(angleA) * sin(angleB);

    // Calculate angle between vectors
    return cross < 0 ? -acos(dot / (magnitudeA * magnitudeB)) : acos(dot / (magnitudeA * magnitudeB));
}
