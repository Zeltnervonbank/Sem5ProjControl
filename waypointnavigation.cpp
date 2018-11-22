#include "waypointnavigation.h"

WaypointNavigation::WaypointNavigation()
{

}

void WaypointNavigation::NavigateToNextWaypoint()
{
    if(GetDistanceToWaypoint() < 0.1 && waypoints.size() > 0)
    {
        CurrentWaypoint = waypoints.front();
        waypoints.pop();
        Movement::Move(0.0, 0.0);
    }
    else
    {
        MoveTowardWaypoint();
    }
}

const double maxSpeed = 0.4;
const double maxRotation = 1.2;

void WaypointNavigation::MoveTowardWaypoint()
{
    RobotPosition position = Globals::LastPosition;

    double yaw = fmod(2.0 * atan2(position.rotW, position.rotZ) + 3.0 * M_PI, 2.0 * M_PI);

    double angleToWaypoint = atan2(position.posX - CurrentWaypoint.x, position.posY - CurrentWaypoint.y) + 1.0/2.0 * M_PI;

    double difference = angleToWaypoint - yaw;
    std::cout << "Rotation offset: " << difference << " Distance: " << GetDistanceToWaypoint() << std::endl;

    double speed = 0;
    double rotation = 0;
    if(difference > 0.01)
    {
        rotation = 0.6;
    }
    else if(difference < -0.01)
    {
        rotation = -0.6;
    }

    if(GetDistanceToWaypoint() > 0.01 && abs(difference) < 0.1)
    {
        speed = 1.2;
    }

    Movement::Move(speed, rotation);
}

double WaypointNavigation::GetDistanceToWaypoint()
{
    double xDisplacement = abs(Globals::LastPosition.posX - CurrentWaypoint.x);
    double yDisplacement = abs(Globals::LastPosition.posY - CurrentWaypoint.y);
    return sqrt(pow(xDisplacement, 2) + pow(yDisplacement, 2));
}
