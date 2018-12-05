#include "waypointnavigation.h"

WaypointNavigation::WaypointNavigation()
{

}

void WaypointNavigation::NavigateToNextWaypoint()
{
    // If the robot is close to the active waypoint and there are more waypoints in queue
    if(GetDistanceToWaypoint() < 0.1 && waypoints.size() > 0)
    {
        // Get the next waypoint from queue, and stop robot
        CurrentWaypoint = waypoints.front();
        waypoints.pop();
        Movement::Move(0.0, 0.0);
    }

    // Stop the robot if close to waypoint, and there are no more waypoints in queue
    else if (GetDistanceToWaypoint() < 0.1 && waypoints.size() == 0)
    {
        Movement::Move(0.0, 0.0);
    }

    // If far from waypoint, move towards it
    else
    {
        MoveTowardWaypoint();
    }
}

void WaypointNavigation::MoveTowardWaypoint()
{
    /// Determine direction to next waypoint
    // Get the latest position
    RobotPosition position = Globals::LastPosition;

    // Calculate the yaw of the robot
    double yaw = fmod(2.0 * atan2(position.rotW, position.rotZ) + 3.0 * M_PI, 2.0 * M_PI);

    // Get X and Y components of the robot's yaw vector
    double yawX = cos(yaw);
    double yawY = sin(yaw);

    // Get displacement of waypoint in comparison to robot position
    double xDisplacement = CurrentWaypoint.x - position.posX;
    double yDisplacement = position.posY - CurrentWaypoint.y;

    // Get dot product of vectors
    double dot = yawX * xDisplacement + yawY * yDisplacement;

    // Get cross product of vectors
    double cross = yawX * yDisplacement - yawY * xDisplacement;

    // Get the distance to the waypoint (magnitude of second vector)
    double distance = GetDistanceToWaypoint();

    // Calculate angle between vectors
    double difference = cross < 0 ? -acos(dot / distance) : acos(dot / distance);

    // Print some data
    std::cout << "Rotation offset: " << difference << " Distance: " << GetDistanceToWaypoint() << " Cross: " << cross << std::endl;

    /// Output movement
    // Set rotation depending on sign of angle to waypoint
    double rotation = difference > 0 ? 1.0 : -1.0;

    // If almost pointing at waypoint, reduce speed to limit overshoot
    rotation = abs(difference) < 0.3 ? rotation * 0.3 : rotation;

    // Set speed if not very close to waypoint and pointing in correct direction, else stop
    double speed = GetDistanceToWaypoint() > 0.1 && abs(difference) < 0.1 ? 1.2 : 0;

    // Send movement
    Movement::Move(speed, rotation);
}

double WaypointNavigation::GetDistanceToWaypoint()
{
    double xDisplacement = abs(Globals::LastPosition.posX - CurrentWaypoint.x);
    double yDisplacement = abs(Globals::LastPosition.posY - CurrentWaypoint.y);

    // Uses pythagorean theorem to determine absolute distance to waypoint
    return sqrt(pow(xDisplacement, 2) + pow(yDisplacement, 2));
}
