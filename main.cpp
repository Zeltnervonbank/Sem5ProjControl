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
#include "wall_controller.h"
#include "qlearning.h"
#include <chrono>
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
bool Camera::marbleClose = false;
std::vector<LidarMarble> lidar::detectedMarbles;
LidarMarble lidar::nearestMarble;
std::vector<LidarRay> lidar::lidarRays;
LidarRay lidar::nearestPoint;

// Marble collection
int marblePoint = 0;
int marblesCollected = 0;
auto start = std::chrono::steady_clock::now();
auto tempC = start;
int cent;
int radius;

// Mapping
int mapping::map[MAP_SIDE_LENGTH][MAP_SIDE_LENGTH] = {};
cv::Mat mapping::img = cv::Mat(MAP_SIDE_LENGTH, MAP_SIDE_LENGTH, CV_8U);
bool mapping::mappingEnabled = false;
static boost::mutex mutex;

// Waypoint navigation
std::queue<WaypointNavigation::Waypoint> WaypointNavigation::waypoints;
WaypointNavigation::Waypoint WaypointNavigation::CurrentWaypoint = {.x = 0.0, .y = 0.0};

// Pathfinding
int pathing::grid[ROW][COL] = {};
cv::Mat pathing::image = cv::imread("/home/andreas/Desktop/floor_plan.png", CV_LOAD_IMAGE_COLOR);

// Callbacks
void statCallback(ConstWorldStatisticsPtr &_msg)
{
      (void)_msg;
      // Dump the message contents to stdout.
      //std::cout << _msg->DebugString();
      //std::cout << std::flush;
}

void contactCallback(ConstContactSensorPtr &_msg)
{
    auto startC=std::chrono::steady_clock::now();
    if(_msg->ByteSize()>300){
        std::chrono::duration<double> diffC = startC-tempC;
        //std::cout << diffC.count() << std::endl;
        if(diffC.count()>=0.20){
            marblePoint+=100;
            marblesCollected++;
            std::cout << "Marble point:                          " << marblePoint << std::endl;
            std::cout << "Marbles collected:                          " << marblesCollected << std::endl;
        }
        tempC=startC;
     }
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

    gazebo::transport::SubscriberPtr cameraSubscriber = node->Subscribe("~/pioneer2dx/camera/link/camera/image", Camera::cameraCallback);

    gazebo::transport::SubscriberPtr marble_clone_0 = node->Subscribe("~/marble_clone_0/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_1 = node->Subscribe("~/marble_clone_1/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_2 = node->Subscribe("~/marble_clone_2/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_3 = node->Subscribe("~/marble_clone_3/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_4 = node->Subscribe("~/marble_clone_4/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_5 = node->Subscribe("~/marble_clone_5/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_6 = node->Subscribe("~/marble_clone_6/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_7 = node->Subscribe("~/marble_clone_7/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_8 = node->Subscribe("~/marble_clone_8/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_9 = node->Subscribe("~/marble_clone_9/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_10 = node->Subscribe("~/marble_clone_10/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_11 = node->Subscribe("~/marble_clone_11/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_12 = node->Subscribe("~/marble_clone_12/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_13 = node->Subscribe("~/marble_clone_13/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_14 = node->Subscribe("~/marble_clone_14/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_15 = node->Subscribe("~/marble_clone_15/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_16 = node->Subscribe("~/marble_clone_16/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_17 = node->Subscribe("~/marble_clone_17/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_18 = node->Subscribe("~/marble_clone_18/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_19 = node->Subscribe("~/marble_clone_19/marble/link/marble_contact/contacts", contactCallback);

    // Publish to the robot vel_cmd topic
    Globals::movementPublisher = Globals::node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

    // Publish a reset of the world
    gazebo::transport::PublisherPtr worldPublisher = Globals::node->Advertise<gazebo::msgs::WorldControl>("~/world_control");

    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);

    // Main code

    // TODO: Move
    const int key_c = 99;
    const int key_v = 118;

    // Make marble controller (Marble collection)- TODO: Rename
    marbel_Controller marble;
    marble.buildController();

    // Make wall controller (obstacle avoidance)
    wall_Controller wall;
    wall.buildController();

    // Instantiate Q learning
    Qlearning qlear;
    qlear.initialize();
    qlear.chooseAction(0, 0, 1);

    // Values to simulate runs - TODO: Refactor
    int visited=0;
    int runs=0;

    // Set random seed - TODO: Refactor
    srand(time(NULL));

    // Convert image file to A* grid
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
        }
        std::cout << std::endl;
    }


    // Loop
    while (true)
    {
        if(marblesCollected == 20)
        {
            std::cout << "ya done son" << std::endl;
        }

        gazebo::common::Time::MSleep(10);


        mutex.lock();
        int key = cv::waitKey(1);
        mutex.unlock();


    if(Camera::marbleClose)
    {
        dir= 0.0;
        speed = 1.0;
    }

    else if(lidar::marblesPresent && lidar::nearestMarble.distance < 1000)
    {
        dir = marble.getControlOutput(
                    lidar::nearestMarble.angle,
                    lidar::nearestMarble.distance
                    ).direction;

        speed = marble.getControlOutput(
                    lidar::nearestMarble.angle,
                    lidar::nearestMarble.distance
                    ).speed;
    }
    else if(lidar::nearestPoint.range < 1 && abs(lidar::nearestPoint.angle) <= 1.56)
    {
        dir = wall.getControlOutput(
                    lidar::nearestPoint.angle,
                    lidar::nearestPoint.range
                    ).direction;

        speed = wall.getControlOutput(
                    lidar::nearestPoint.angle,
                    lidar::nearestPoint.range
                    ).speed;
    }
    else
    {
        // TODO: Implement A* pathfollowing here
        dir = 0.0;
        speed = 0.7;
    }

    // Source is the middle point
    std::vector<int> src = convertToPixelCoords({0, 0});

    std::vector<std::vector<int>> dest = {{-36, 22}, {-25, 22}, {-26, 11}, {-36, 10}, {-13, 11}, {-13, 22}, {7, 21}, {7, 10}, {-36, -1}, {-36, -23}, {-20, -22}};
    std::vector<std::vector<int>> pixelDest;

    for(size_t i = 0 ; i < dest.size(); i++)
    {
        pixelDest.push_back(convertToPixelCoords(dest[i]));
    }

    pathing::aStarmulti(src, pixelDest);

    if ((key == key_up) && (speed <= 1.2f))
      speed += 0.05;
    else if ((key == key_down) && (speed >= -1.2f))
     speed -= 0.05;
    else if ((key == key_right) && (dir <= 0.4f))
      dir += 0.05;
    else if ((key == key_left) && (dir >= -0.4f))
      dir -= 0.05;
    else if (key == key_c)
    {
        if(visited >= 3)
        {
            qlear.run();
            runs++;
            visited = 0;
            std::cout << "runs: " << runs << std::endl;
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = end - start;

        qlear.chooseAction(qlear.currentState, marblePoint, diff.count());
        marblePoint = 0;
        visited++;
        qlear.printR();
    }
    else if (key == key_v)
    {
        qlear.printroute();
    }

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

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
