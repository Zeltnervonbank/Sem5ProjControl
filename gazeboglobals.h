#ifndef GAZEBOGLOBALS_H
#define GAZEBOGLOBALS_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

class GazeboGlobals
{
public:
    static gazebo::transport::NodePtr node(new gazebo::transport::Node());
};

#endif // GAZEBOGLOBALS_H
