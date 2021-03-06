#ifndef GAZEBOGLOBALS_H
#define GAZEBOGLOBALS_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include "datatypes.h"
#include "vector"



class Globals
{
public:
    static gazebo::transport::NodePtr node;
    static gazebo::transport::PublisherPtr movementPublisher;
    static boost::mutex mutex;

    // Movement related
    static RobotPosition lastPosition;
    static std::queue<Waypoint> waypoints;
    static Waypoint currentWaypoint;
    static Destination currentDestination;
    static Destination previousDestination;

    static std::vector<std::vector<int>> destinations;
    static std::queue<Destination> destinationQueue;

    // Methods
    static double GetDistanceToWaypoint();

    static void ClearWaypointQueue();
    static void ClearDestinationQueue();

    static void NextDestination();
    static void NextWaypoint();

    static double GetRobotYaw();
    static double AngleBetweenVectors(double angleA, double magnitudeA, double angleB, double magnitudeB);
};

#endif // GAZEBOGLOBALS_H
