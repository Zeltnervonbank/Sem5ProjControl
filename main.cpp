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
#include "waypointcontroller.h"


// Initialise Gazebo node and publishers
gazebo::transport::NodePtr Globals::node(new gazebo::transport::Node());
gazebo::transport::PublisherPtr Globals::movementPublisher;

// Initialise static variables in classes:
// Globals
boost::mutex Globals::mutex;
RobotPosition Globals::lastPosition;
std::queue<Waypoint> Globals::waypoints;
Waypoint Globals::currentWaypoint = {.x = 0.0, .y = 0.0, .isDestination = false};
Destination Globals::currentDestination;
Destination Globals::previousDestination;
std::vector<std::vector<int>> Globals::destinations;
std::queue<Destination> Globals::destinationQueue;

// Lidar
bool Lidar::marblesPresent = false;
bool Camera::marbleClose = false;
std::vector<LidarMarble> Lidar::detectedMarbles;
LidarMarble Lidar::nearestMarble;
std::vector<LidarRay> Lidar::lidarRays;
LidarRay Lidar::nearestPoint;

// Mapping
int mapping::map[MAP_SIDE_LENGTH][MAP_SIDE_LENGTH] = {};
cv::Mat mapping::img = cv::Mat(MAP_SIDE_LENGTH, MAP_SIDE_LENGTH, CV_8U);
bool mapping::mappingEnabled = false;
static boost::mutex mutex;

// Pathfinding
int pathing::grid[ROW][COL] = {};
cv::Mat pathing::image = cv::imread("../Sem5ProjControl/floor_plan.png", CV_LOAD_IMAGE_COLOR);

// Movement
Qlearning Movement::qLearn;
marbel_Controller Movement::marbleController;
wall_Controller Movement::wallController;
waypointController Movement::wayController;

int Movement::visited;
int Movement::runs;
int Movement::marblePoint = 0;

std::chrono::_V2::steady_clock::time_point Movement::start = std::chrono::steady_clock::now();
bool Movement::testMode;
bool Movement::allowPassiveSlowing;
bool Movement::printKeyPresses;
bool Movement::enableAutomaticMovement;

// Marble collection
int marblesCollected = 0;
auto tempC = Movement::start;
int cent;
int radius;

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
    auto startC = std::chrono::steady_clock::now();

    if(_msg->ByteSize() > 300)
    {
        std::chrono::duration<double> diffC = startC - tempC;

        if(diffC.count() >= 0.20)
        {
            Movement::marblePoint += 100;
            marblesCollected++;
            std::cout << "Marble point:                          " << Movement::marblePoint << std::endl;
            std::cout << "Marbles collected:                          " << marblesCollected << std::endl;

            pathing::CreatePathToCurrentDestination();
        }

        tempC = startC;
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

            Globals::lastPosition = pos;
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

void LoadImageIntoAStarGrid(const char* path)
{
    cv::Mat image = cv::imread(path, CV_8U);

    // Convert image filee to A* grid
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
}

void SetDestinations()
{
    Globals::destinations = {{-36, 22}, {-25, 22}, {-26, 11}, {-36, 10}, {-13, 11}, {-13, 22}, {7, 21}, {7, 10}, {-36, -1}, {-36, -23}, {-20, -22}};
}

void AddDestinationToQueue(int index)
{
    Destination d = {.x = (double)Globals::destinations[index][0], .y = (double)Globals::destinations[index][1], .index = index};
    Globals::destinationQueue.push(d);
}

void AddAllToDestinationQueue()
{
    for (size_t i = 0; i < Globals::destinations.size(); i++)
    {
        AddDestinationToQueue(i);
    }
}

void RandomOrderAddAllDestinationsToQueue()
{
    // Make new vector for indexes and add each index to it
    std::vector<int> shuffledIndexes;
    for(size_t i = 0; i < Globals::destinations.size(); i++)
    {
        shuffledIndexes.push_back(i);
    }

    // Shuffle indexes
    std::random_shuffle (shuffledIndexes.begin(), shuffledIndexes.end());

    // Add each destination to queue
    for(size_t i = 0; i < shuffledIndexes.size(); i++)
    {
        AddDestinationToQueue(shuffledIndexes[i]);
    }
}

void SeedWaypointsWithAStar()
{
    // Source is the middle point
    std::vector<int> src = convertToPixelCoords({0, 0});

    // List of points we want to go to
    std::vector<std::vector<int>> dest = {{-36, 22}, {-25, 22}, {-26, 11}, {-36, 10}, {-13, 11}, {-13, 22}, {7, 21}, {7, 10}, {-36, -1}, {-36, -23}, {-20, -22}};

    // Output points
    std::vector<std::vector<int>> pixelDest;

    // I wish C++ had LINQ ;_;
    for(size_t i = 0 ; i < dest.size(); i++)
    {
        pixelDest.push_back(convertToPixelCoords(dest[i]));
    }

    pathing::AStarMultiSearch(src, pixelDest);
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

    gazebo::transport::SubscriberPtr lidarSubscriber = Globals::node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", Lidar::LidarCallback);

    gazebo::transport::SubscriberPtr cameraSubscriber = Globals::node->Subscribe("~/pioneer2dx/camera/link/camera/image", Camera::CameraCallback);

    gazebo::transport::SubscriberPtr marble_clone_0 = Globals::node->Subscribe("~/marble_clone_0/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_1 = Globals::node->Subscribe("~/marble_clone_1/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_2 = Globals::node->Subscribe("~/marble_clone_2/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_3 = Globals::node->Subscribe("~/marble_clone_3/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_4 = Globals::node->Subscribe("~/marble_clone_4/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_5 = Globals::node->Subscribe("~/marble_clone_5/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_6 = Globals::node->Subscribe("~/marble_clone_6/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_7 = Globals::node->Subscribe("~/marble_clone_7/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_8 = Globals::node->Subscribe("~/marble_clone_8/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_9 = Globals::node->Subscribe("~/marble_clone_9/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_10 = Globals::node->Subscribe("~/marble_clone_10/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_11 = Globals::node->Subscribe("~/marble_clone_11/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_12 = Globals::node->Subscribe("~/marble_clone_12/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_13 = Globals::node->Subscribe("~/marble_clone_13/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_14 = Globals::node->Subscribe("~/marble_clone_14/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_15 = Globals::node->Subscribe("~/marble_clone_15/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_16 = Globals::node->Subscribe("~/marble_clone_16/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_17 = Globals::node->Subscribe("~/marble_clone_17/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_18 = Globals::node->Subscribe("~/marble_clone_18/marble/link/marble_contact/contacts", contactCallback);
    gazebo::transport::SubscriberPtr marble_clone_19 = Globals::node->Subscribe("~/marble_clone_19/marble/link/marble_contact/contacts", contactCallback);

    // Publish to the robot vel_cmd topic
    Globals::movementPublisher = Globals::node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

    // Publish a reset of the world
    gazebo::transport::PublisherPtr worldPublisher = Globals::node->Advertise<gazebo::msgs::WorldControl>("~/world_control");

    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);

    /// Main code
    // Prepare destinations
    SetDestinations();
    RandomOrderAddAllDestinationsToQueue();

    // Initialise Q learning system
    Movement::qLearn.Initialize();
    Movement::qLearn.ChooseAction(0, 0, 1);
    Movement::qLearn.PrintRoute();


    // Prepare A* grid
    LoadImageIntoAStarGrid("../Sem5ProjControl/floor_plan.png");
    Globals::NextDestination();
    pathing::CreatePathToCurrentDestination();

    //SeedWaypointsWithAStar();
    /*Waypoint p = {.x = -10.0, .y = 0.0};
    Globals::CurrentWaypoint = p;*/


    // Build fuzzy controllers
    Movement::marbleController.buildController();
    Movement::wallController.buildController();
    Movement::wayController.buildController();

    // Apply settings for Movement class
    Movement::allowPassiveSlowing = true;
    Movement::printKeyPresses = false;
    Movement::testMode = false;
    Movement::enableAutomaticMovement = true;

    // Set random seed - TODO: Refactor
    srand(time(NULL));

    // Loop
    while (true)
    {
        // Insert slight delay between frames
        //try
        //{
            gazebo::common::Time::MSleep(10);
        //}
        //catch(std::exception e)
/*        {
            std::cout << "An error occurred:\n" << e.what() << std::endl;
        }
*/
        // If all marbles have been collected
        if(marblesCollected == 20)
        {
            std::cout << "ya done son" << std::endl;
        }

        // Handle movement and quit if esc key is pressed
        if(Movement::HandleMovement() == -1)
        {
            break;
        }
    }

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
