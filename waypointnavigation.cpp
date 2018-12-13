#include "waypointnavigation.h"

WaypointNavigation::WaypointNavigation()
{

}

void WaypointNavigation::NavigateToNextWaypoint()
{
    // If the robot is close to the active waypoint and there are more waypoints in queue
    if(Globals::GetDistanceToWaypoint() < 0.1 && Globals::waypoints.size() > 0)
    {
        // Get the next waypoint from queue, and stop robot
        Globals::currentWaypoint = Globals::waypoints.front();
        Globals::waypoints.pop();
        Movement::Move(0.0, 0.0);
    }

    // Stop the robot if close to waypoint, and there are no more waypoints in queue
    else if (Globals::GetDistanceToWaypoint() < 0.05 && Globals::waypoints.size() == 0)
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
    double xDisplacement = Globals::currentWaypoint.x - position.posX;
    double yDisplacement = position.posY - Globals::currentWaypoint.y;

    // Get dot product of vectors
    double dot = yawX * xDisplacement + yawY * yDisplacement;

    // Get cross product of vectors
    double cross = yawX * yDisplacement - yawY * xDisplacement;

    // Get the distance to the waypoint (magnitude of second vector)
    double distance = Globals::GetDistanceToWaypoint();

    // Calculate angle between vectors
    double difference = cross < 0 ? -acos(dot / distance) : acos(dot / distance);


    std::cout << "\033[2J\033[1;1H";
    // Print some data
    std::cout << "Current waypoint: " << Globals::currentWaypoint.x << ", " << Globals::currentWaypoint.y << std::endl;
    std::cout << "Current position: " << position.posX << ", " << position.posY << std::endl;
    std::cout << "Rotation offset: " << difference << " Distance: " << Globals::GetDistanceToWaypoint() << " Cross: " << cross << std::endl;
    std::cout << "Remaining waypoints: " << Globals::waypoints.size() << std::endl;

    try
    {
    /// Output movement
    // Set rotation depending on sign of angle to waypoint
    double rotation = (double) difference;

    // Set speed if not very close to waypoint and pointing in correct direction, else stop
    //double speed = Globals::GetDistanceToWaypoint() > 0.05 && abs(difference) < 0.1 ? 1.2 : 0;

    // Send movement
    //Movement::Move(speed, rotation);
    }
    catch(std::exception e)
    {
        std::cout << e.what() << std::endl;
        Movement::Move(0.0, 0.0);
    }
}
