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
#include "pathing.h"
#include "movement.h"


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

// Pathfinding
int pathing::grid[ROW][COL] = {};
cv::Mat pathing::image = cv::imread("/home/andreas/Desktop/floor_plan.png", CV_LOAD_IMAGE_COLOR);

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

std::vector<int> convertToPixelCoords(std::vector<int> pair)
{
    std::vector<int> retPair;

    retPair.push_back(pair[0] * 4 + 200);
    retPair.push_back(200 - pair[1] * 4);

    return retPair;
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

    cv::Mat image = cv::imread("/home/andreas/Desktop/floor_plan.png", CV_8U);
    cv::imshow("test", image);

    cv::Mat testimg = cv::Mat(COL, ROW, CV_8U);
    testimg.setTo(0);

    for(int y = 0; y < 400; y++)
    {
        for(int x = 0; x < 400; x++)
        {
            auto i = image.at<uchar>(x, y);
            if(i == 255)
            {
                std::cout << "1";
                pathing::grid[y][x] = 1;

            }
            else
            {
                std::cout << "0";
                pathing::grid[y][x] = 0;
            }
            testimg.at<char>(x, y) = i;

        }
        std::cout << std::endl;
    }

    cv::imshow("orientationtest", testimg);

    //cv::waitKey(0);

    /*
    WaypointNavigation::Waypoint p1 = {.x = -16.0, .y = 0.0};
    WaypointNavigation::Waypoint p2 = {.x = -16.0, .y = -10.0};
    WaypointNavigation::Waypoint p3 = {.x = -16.0, .y = 2.0};
    WaypointNavigation::Waypoint p4 = {.x = 0.0, .y = 0.0};


    WaypointNavigation::waypoints.push(p1);
    WaypointNavigation::waypoints.push(p2);
    WaypointNavigation::waypoints.push(p3);
    WaypointNavigation::waypoints.push(p4);*/

    // Source is the middle point
    std::vector<int> src = convertToPixelCoords({0, 0});

    std::vector<std::vector<int>> dest = {{-36, 22}, {-25, 22}, {-26, 11}, {-36, 10}, {-13, 11}, {-13, 22}, {7, 21}, {7, 10}, {-36, -1}, {-36, -23}, {-20, -22}};
    std::vector<std::vector<int>> pixelDest;

    for(size_t i = 0 ; i < dest.size(); i++)
    {
        pixelDest.push_back(convertToPixelCoords(dest[i]));
    }
    //pathing::aStarSearch(std::make_pair(200, 200), std::make_pair(76, 256));

    pathing::aStarmulti(src, pixelDest);

    // Loop
    while (true)
    {
        try
        {
        // Insert slight delay between frames
        gazebo::common::Time::MSleep(10);

        // Handle keyboard input and quit if esc key is pressed
        if(Movement::HandleKeyboardInput() == -1)
        {
            break;
        }

        WaypointNavigation::NavigateToNextWaypoint();

        //std::cout << WaypointNavigation::GetDistanceToWaypoint() << std::endl;

        // Apparently this has to be here, or opencv windows break ¯\_(ツ)_/¯
        //cv::waitKey(1);
        }
        catch(std::exception e)
        {
            std::cout << e.what() << std::endl;
        }

    }

    /* Description of the Grid-
    1--> The cell is not blocked
    0--> The cell is blocked */


   // Pair src1 = {39,59};
   // vector<Pair> dest1 = { {7,9}, {9,24}, {9,41}, {9,68}, {11,93}, {11,111}, {24,9}, {24,24}, {24,41}, {26,68}, {39,8}, {39,34}, {39,93}, {39,111}, {57,52}, {57,111}, {62,8}, {62,34}, {75,52}, {75,79}, {69,79}, {69,111} };

    // Destination is the left-most top-most corner
    // vector<vector<int>> dest = { {5,5} };
   // vector<vector<int>> dest = { {50,70},{60,80},{75,110},{45,75} };
   // Pair dest1 = std::make_pair(60,70);
    //vector<int> dest1 = {39,59};


  //aStarSearch(grid, src, dest);
  //cout << endl;
   // path.aStarSearch(grid,src,dest1);

   /*cv::namedWindow("scaled", CV_WINDOW_AUTOSIZE);

   cv::imshow("scaled", *mypoint);*/
   //cv::waitKey(0);


    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
