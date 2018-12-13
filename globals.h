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
    static RobotPosition LastPosition;

    static std::queue<Waypoint> waypoints;
    static Waypoint CurrentWaypoint;

    static std::vector<std::vector<int>> destinations;
    static std::queue<Waypoint> destinationQueue;

    static double GetDistanceToWaypoint();
    static void ClearWaypointQueue();
    static void ClearDestinationQueue();
};

#endif // GAZEBOGLOBALS_H
