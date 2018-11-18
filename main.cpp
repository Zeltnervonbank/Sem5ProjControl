#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>
#include "camera.h"
#include "marbel_controller.h"
#include "datatypes.h"
#include "lidar.h"
#include "mapping.h"

bool lidar::marblesPresent = false;
std::vector<LidarMarble> lidar::detectedMarbles;
LidarMarble lidar::nearestMarble;
std::vector<LidarRay> lidar::lidarRays;
LidarRay lidar::nearestPoint;

int mapping::map[MAP_SIDE_LENGTH][MAP_SIDE_LENGTH] = {};
cv::Mat mapping::img = cv::Mat(MAP_SIDE_LENGTH, MAP_SIDE_LENGTH, CV_8U);


static boost::mutex mutex;
    int cent;
    float dir =0.0;

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
    //std::cout << std::flush;


    for (int i = 0; i < _msg->pose_size(); i++)
    {
        if (_msg->pose(i).name() == "pioneer2dx")
        {
            RobotPosition pos = {
                _msg->pose(i).position().x(),
                _msg->pose(i).position().y(),
                _msg->pose(i).orientation().w(),
                _msg->pose(i).orientation().x(),
                _msg->pose(i).orientation().y(),
                _msg->pose(i).orientation().z()                
            };

            mapping::UpdateMap(pos);


      /*std::cout << std::setprecision(2) << std::fixed << std::setw(6)
               << _msg->pose(i).position().x() << std::setw(6)
               << _msg->pose(i).position().y() << std::setw(6)
               << _msg->pose(i).position().z() << std::endl
                << _msg->pose(i).orientation().w() << std::setw(6)
                << _msg->pose(i).orientation().x() << std::setw(6)
                << _msg->pose(i).orientation().y() << std::setw(6)
                << _msg->pose(i).orientation().z() << std::endl;*/
        }
    }
}

void cameraCallback(ConstImageStampedPtr &msg) {
    std::size_t width = msg->image().width();
    std::size_t height = msg->image().height();
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));
    Camera cam;
    //MarbleLocation mLoc = cam.getMarbelCenter(im);

    //marbel_Controller fuzzy;
    //dir=fuzzy.buildController(mLoc.center);


    mutex.lock();
    cv::imshow("camera", im);
    mutex.unlock();
}

int main(int _argc, char **_argv) {
  //lidar::doSomething();
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo topics
  gazebo::transport::SubscriberPtr statSubscriber =
      node->Subscribe("~/world_stats", statCallback);

  gazebo::transport::SubscriberPtr poseSubscriber =
      node->Subscribe("~/pose/info", poseCallback); 

  gazebo::transport::SubscriberPtr lidarSubscriber =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidar::lidarCallback);

  gazebo::transport::SubscriberPtr cameraSubscriber =
  node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);


  // Publish to the robot vel_cmd topic
  gazebo::transport::PublisherPtr movementPublisher =
      node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

  // Publish a reset of the world
  gazebo::transport::PublisherPtr worldPublisher =
      node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  gazebo::msgs::WorldControl controlMessage;
  controlMessage.mutable_reset()->set_all(true);
  worldPublisher->WaitForConnection();
  worldPublisher->Publish(controlMessage);

  const int key_left = 81;
  const int key_up = 82;
  const int key_down = 84;
  const int key_right = 83;
  const int key_esc = 27;
  const int key_s = 115;
  const int key_l = 108;
  marbel_Controller fuzzy;

  float speed = 0.0;

    mapping::img.setTo(0);
    // Loop
  while (true) {
    //std::cout << cent << std::endl;
    gazebo::common::Time::MSleep(10);

    // Display lidar info
    /*std::cout << "Marbles have been detected: " << lidar::marblesPresent << std::endl;
    std::cout << "Range to nearest detected point: " << lidar::nearestPoint.range << std::endl;
    std::cout << "Total number of detected marbles: " << lidar::detectedMarbles.size() << std::endl;
    std::cout << "Total number of rays: " << lidar::lidarRays.size() << std::endl;*/
    //std::cout << mapping::map[1][1] << std::endl;

    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();


    if (key == key_esc)
      break;

    //dir=fuzzy.buildController(cent);

    /*if (cent==0){
        speed=0.5;
  }
    else if(cent>=150 && cent<=170 && cent != 0){
        speed= 0.5;
  }*/
//    else if (cent > 170)
//        dir =0.15;
//    else if (cent < 150 && cent > 0)
//        dir =-0.15;
    // Print key pressed
    if(key != 255)
    {
        std::cout << key << std::endl;
    }

    if ((key == key_up) && (speed <= 1.2f))
      speed += 0.05;
    else if ((key == key_down) && (speed >= -1.2f))
     speed -= 0.05;
    else if ((key == key_right) && (dir <= 0.4f))
      dir += 0.05;
    else if ((key == key_left) && (dir >= -0.4f))
      dir -= 0.05;
    else if(key == key_s)
    {
        mapping::SaveMapToDisk();
    }
    else if(key == key_l)
    {
        mapping::LoadMapFromDisk();
    }
    else
    {
        speed *= 0.99f;
        dir *= 0.99f;
    }
//    else {
      // slow down
      //      speed *= 0.1;
      //      dir *= 0.1;
    //}

    // Generate a pose
    ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
