#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>
#include <queue>

#include "camera.h"
#include "marbel_controller.h"
#include "datatypes.h"
#include "lidar.h"
#include "mapping.h"
#include "waypointnavigation.h"
#include "movement.h"
#include "globals.h"


// Initialise Gazebo node and publishers
gazebo::transport::NodePtr Globals::node(new gazebo::transport::Node());
gazebo::transport::PublisherPtr Globals::movementPublisher;

// Initialise static variables in classes:
// Globals
boost::mutex Globals::mutex;
RobotPosition Globals::LastPosition;

// Lidar
bool lidar::marblesPresent = false;
std::vector<LidarMarble> lidar::detectedMarbles;
LidarMarble lidar::nearestMarble;
std::vector<LidarRay> lidar::lidarRays;
LidarRay lidar::nearestPoint;

// Mapping
int mapping::map[MAP_SIDE_LENGTH][MAP_SIDE_LENGTH] = {};
cv::Mat mapping::img = cv::Mat(MAP_SIDE_LENGTH, MAP_SIDE_LENGTH, CV_8U);
bool mapping::mappingEnabled = false;

// Waypoint navigation
std::queue<WaypointNavigation::Waypoint> WaypointNavigation::waypoints;
WaypointNavigation::Waypoint WaypointNavigation::CurrentWaypoint = {.x = 0.0, .y = 0.0};

void statCallback(ConstWorldStatisticsPtr &_msg)
{
      (void)_msg;
      // Dump the message contents to stdout.
      //std::cout << _msg->DebugString();
      //std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg)
{
    for (int i = 0; i < _msg->pose_size(); i++)
    {
        if (_msg->pose(i).name() == "pioneer2dx")
        {
            RobotPosition pos =
            {
                _msg->pose(i).position().x(),
                _msg->pose(i).position().y(),
                _msg->pose(i).orientation().w(),
                _msg->pose(i).orientation().x(),
                _msg->pose(i).orientation().y(),
                _msg->pose(i).orientation().z()                
            };

            Globals::LastPosition = pos;
            mapping::UpdateMap(pos);
        }
    }
}

// Modified in other branch, won't remove from here for now.
void cameraCallback(ConstImageStampedPtr &msg)
{
    std::size_t width = msg->image().width();
    std::size_t height = msg->image().height();
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

    Globals::mutex.lock();
    cv::imshow("camera", im);
    Globals::mutex.unlock();
}

int main(int _argc, char **_argv)
{
    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    // Initialise our node for communication
    Globals::node->Init();

    // Listen to Gazebo topics
    gazebo::transport::SubscriberPtr statSubscriber = Globals::node->Subscribe("~/world_stats", statCallback);

    gazebo::transport::SubscriberPtr poseSubscriber = Globals::node->Subscribe("~/pose/info", poseCallback);

    gazebo::transport::SubscriberPtr lidarSubscriber = Globals::node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidar::lidarCallback);

    gazebo::transport::SubscriberPtr cameraSubscriber = Globals::node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);

    // Publish to the robot vel_cmd topic
    Globals::movementPublisher = Globals::node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

    // Publish a reset of the world
    gazebo::transport::PublisherPtr worldPublisher = Globals::node->Advertise<gazebo::msgs::WorldControl>("~/world_control");

    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);

    // Set static variables in classes
    mapping::img.setTo(0);

    WaypointNavigation::Waypoint p1 = {.x = -16.0, .y = 0.0};
    WaypointNavigation::Waypoint p2 = {.x = -16.0, .y = -10.0};
    WaypointNavigation::Waypoint p3 = {.x = -16.0, .y = 2.0};
    WaypointNavigation::Waypoint p4 = {.x = 0.0, .y = 0.0};


    WaypointNavigation::waypoints.push(p1);
    WaypointNavigation::waypoints.push(p2);
    WaypointNavigation::waypoints.push(p3);
    WaypointNavigation::waypoints.push(p4);

    // Loop
    while (true)
    {
        // Insert slight delay between frames
        gazebo::common::Time::MSleep(10);

        // Handle keyboard input and quit if esc key is pressed
        /*if(Movement::HandleKeyboardInput() == -1)
        {
            break;
        }

        // Apparently this has to be here, or opencv windows break ¯\_(ツ)_/¯
        cv::waitKey(1);
        }*/

        //WaypointNavigation::NavigateToNextWaypoint();

        //std::cout << WaypointNavigation::GetDistanceToWaypoint() << std::endl;
    }

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
