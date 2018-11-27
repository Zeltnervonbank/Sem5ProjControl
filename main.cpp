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
#include "wall_controller.h"
#include "qlearning.h"

bool lidar::marblesPresent = false;
bool Camera::marbelClose = false;
std::vector<LidarMarble> lidar::detectedMarbles;
LidarMarble lidar::nearestMarble;
std::vector<LidarRay> lidar::lidarRays;
LidarRay lidar::nearestPoint;

static boost::mutex mutex;
    int cent;
    int radius;

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {

      //std::cout << std::setprecision(2) << std::fixed << std::setw(6)
               //<< _msg->pose(i).position().x() << std::setw(6)
               //<< _msg->pose(i).position().y() << std::setw(6)
               //<< _msg->pose(i).position().z() << std::endl;
               // << _msg->pose(i).orientation().w() << std::setw(6)
               // << _msg->pose(i).orientation().x() << std::setw(6)
               // << _msg->pose(i).orientation().y() << std::setw(6)
               // << _msg->pose(i).orientation().z() << std::endl;
    }
  }
}

// Now that Camera Class works, this section can be removed.
/*void cameraCallback(ConstImageStampedPtr &msg) {
    std::size_t width = msg->image().width();
    std::size_t height = msg->image().height();
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));    
    MarbleLocation mLoc = Camera::getMarbelCenter(im);
    radius=mLoc.radius;
    cent=mLoc.center;

    //std::cout << "rad:" << radius << std::endl;
    //std::cout << "cent:" << mLoc.center << std::endl;
    marbel_Controller fuzzy;
    //dir=fuzzy.buildController(mLoc.center);

    mutex.lock();
    cv::imshow("camera", im);
    mutex.unlock();
}*/

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
      node->Subscribe("~/pioneer2dx/camera/link/camera/image", Camera::cameraCallback);


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
  const int key_c = 99;
  const int key_v = 118;
  /*marbel_Controller fuzzy;
  fuzzy.buildController();
  wall_Controller fuzz;
  fuzz.buildController();*/

  int reward=0;
  int iterations=0;
  int visited=0;
  int runs=0;
  Qlearning qlear;
  //qlear.initialize();
  qlear.chooseAction(0,0,1);
  //std::cout << "her2" << std::endl;

  float speed=0;
  float dir;


    // Loop
  while (true) {
    //std::cout << cent << std::endl;
    gazebo::common::Time::MSleep(10);

    /* Display lidar info
    std::cout << "Marbles have been detected: " << lidar::marblesPresent << std::endl;
    std::cout << "angle to nearest detected point: " << lidar::nearestPoint.angle << std::endl;
    std::cout << "distance to nearest detected point: " << lidar::nearestPoint.range << std::endl;
    std::cout << "Total number of detected marbles: " << lidar::detectedMarbles.size() << std::endl;
    std::cout << "Total number of rays: " << lidar::lidarRays.size() << std::endl;*/

    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();


    if (key == key_esc)
      break;
/*
    std::cout << "close?" << Camera::marbelClose << std::endl;

    if(Camera::marbelClose){
        dir= 0.0;
        speed = 1.0;
    }
    else if(lidar::marblesPresent==1 && lidar::nearestMarble.distance<1000){
    dir= fuzzy.getControlOutput(lidar::nearestMarble.angle,lidar::nearestMarble.distance).direction;
    speed = fuzzy.getControlOutput(lidar::nearestMarble.angle,lidar::nearestMarble.distance).speed;
  }
    else if(lidar::nearestPoint.range<1 && lidar::nearestPoint.angle<=1.56 && lidar::nearestPoint.angle>=-1.56 ){
        dir=fuzz.getControlOutput(lidar::nearestPoint.angle,lidar::nearestPoint.range).direction;
        speed=fuzz.getControlOutput(lidar::nearestPoint.angle,lidar::nearestPoint.range).speed;
    }
    else{
        dir= 0.0;
        speed = 0.7;
    }

    //if(radius>36){
    //    dir=0;
    //    speed=0;
    //    for(int i=0; i<100;i++){

     //   }
  //}
  */
    //std::cout << "key" << key << std::endl;

    //dir=0.5; Højre er positiv retning
    //dir=-0.5; Venstre er negativ retning

    //else if(cent>=150 && cent<=170 && cent != 0){
  //      speed= 0.5;
 // }
//    else if (cent > 170)
//        dir =0.15;
//    else if (cent < 150 && cent > 0)
//        dir =-0.15;


    if ((key == key_up) && (speed <= 1.2f))
      speed += 0.05;
    else if ((key == key_down) && (speed >= -1.2f))
     speed -= 0.05;
    else if ((key == key_right) && (dir <= 0.4f))
      dir += 0.05;
    else if ((key == key_left) && (dir >= -0.4f))
      dir -= 0.05;
    else if (key == key_c){
        if(visited>=3){
            qlear.run();
            runs++;
            visited=0;
            std::cout << "runs: " << runs << std::endl;
        }
        reward= (rand()%100)+1;
        std::cout << "reward rand" << reward << std::endl;
        qlear.chooseAction(qlear.currentState,reward,iterations);
        iterations=0;
        visited++;
        qlear.printR();
    }
    else if (key == key_v){
        qlear.printroute();
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
    iterations++;
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
