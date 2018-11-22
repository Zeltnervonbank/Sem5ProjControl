#ifndef GAZEBOGLOBALS_H
#define GAZEBOGLOBALS_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include "datatypes.h"

class Globals
{
public:
    static gazebo::transport::NodePtr node;
    static gazebo::transport::PublisherPtr movementPublisher;
    static boost::mutex mutex;
    static RobotPosition LastPosition;
};

#endif // GAZEBOGLOBALS_H
