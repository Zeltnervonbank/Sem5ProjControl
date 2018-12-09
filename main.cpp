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
#include <chrono>

bool lidar::marblesPresent = false;
bool Camera::marbelClose = false;
std::vector<LidarMarble> lidar::detectedMarbles;
LidarMarble lidar::nearestMarble;
std::vector<LidarRay> lidar::lidarRays;
LidarRay lidar::nearestPoint;
int marblePoint=0;
int marblesCollected=0;
auto start=std::chrono::steady_clock::now();
auto tempC=start;

static boost::mutex mutex;
    int cent;
    int radius;

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
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


  gazebo::transport::SubscriberPtr marble_clone_0 = node->Subscribe("~/marble_clone_0/marble/link/marble_contact/contacts", contactCallback);
  gazebo::transport::SubscriberPtr marble_clone_1 = node->Subscribe("~/marble_clone_1/marble/link/marble_contact/contacts", contactCallback);
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
  gazebo::transport::SubscriberPtr marble_clone_2 = node->Subscribe("~/marble_clone_2/marble/link/marble_contact/contacts", contactCallback);
  gazebo::transport::SubscriberPtr marble_clone_3 = node->Subscribe("~/marble_clone_3/marble/link/marble_contact/contacts", contactCallback);
  gazebo::transport::SubscriberPtr marble_clone_4 = node->Subscribe("~/marble_clone_4/marble/link/marble_contact/contacts", contactCallback);
  gazebo::transport::SubscriberPtr marble_clone_5 = node->Subscribe("~/marble_clone_5/marble/link/marble_contact/contacts", contactCallback);
  gazebo::transport::SubscriberPtr marble_clone_6 = node->Subscribe("~/marble_clone_6/marble/link/marble_contact/contacts", contactCallback);
  gazebo::transport::SubscriberPtr marble_clone_7 = node->Subscribe("~/marble_clone_7/marble/link/marble_contact/contacts", contactCallback);
  gazebo::transport::SubscriberPtr marble_clone_8 = node->Subscribe("~/marble_clone_8/marble/link/marble_contact/contacts", contactCallback);
  gazebo::transport::SubscriberPtr marble_clone_9 = node->Subscribe("~/marble_clone_9/marble/link/marble_contact/contacts", contactCallback);



  const int key_left = 81;
  const int key_up = 82;
  const int key_down = 84;
  const int key_right = 83;
  const int key_esc = 27;
  const int key_c = 99;
  const int key_v = 118;

  marbel_Controller marble;
  marble.buildController();
  wall_Controller wall;
  wall.buildController();

  int visited=0;
  int runs=0;
  Qlearning qlear;
  qlear.initialize();
  qlear.chooseAction(0,0,1);
  //std::cout << "her2" << std::endl;

srand(time(NULL));

  float speed=0;
  float dir=0;


    // Loop
  while (true) {

      if(marblesCollected==20){
          std::cout << "ya done son" << std::endl;
      }
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

    //std::cout << gazebo::common::Time::GetWallTime()-startTime << std::endl;

    if (key == key_esc)
      break;

    //std::cout << "close?" << Camera::marbelClose << std::endl;

    if(Camera::marbelClose){
        dir= 0.0;
        speed = 1.0;
    }
    else if(lidar::marblesPresent==1 && lidar::nearestMarble.distance<1000){
    dir= marble.getControlOutput(lidar::nearestMarble.angle,lidar::nearestMarble.distance).direction;
    speed = marble.getControlOutput(lidar::nearestMarble.angle,lidar::nearestMarble.distance).speed;
  }
    else if(lidar::nearestPoint.range<1 && lidar::nearestPoint.angle<=1.56 && lidar::nearestPoint.angle>=-1.56 ){
        dir=wall.getControlOutput(lidar::nearestPoint.angle,lidar::nearestPoint.range).direction;
        speed=wall.getControlOutput(lidar::nearestPoint.angle,lidar::nearestPoint.range).speed;
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
        //reward= (rand()%100)+1;
        //std::cout << "reward rand" << reward << std::endl;
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = end-start;
        //std::cout << diff.count() << std::endl;
        qlear.chooseAction(qlear.currentState,marblePoint,diff.count());
        marblePoint=0;
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
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
