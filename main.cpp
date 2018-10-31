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

void cameraCallback(ConstImageStampedPtr &msg) {
    std::size_t width = msg->image().width();
    std::size_t height = msg->image().height();
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));
    Camera cam;
    MarbleLocation mLoc = cam.getMarbelCenter(im);

    marbel_Controller fuzzy;
    dir=fuzzy.buildController(mLoc.center);


    mutex.lock();
    cv::imshow("camera", im);
    mutex.unlock();
}

float GetCollinearity(cv::Point2f points[3])
{
    cv::Point2f A = points[0];
    cv::Point2f B = points[1];
    cv::Point2f C = points[2];

    float area = A.x * (B.y - C.y) + B.x * (C.y - A.y) + C.x * (A.y - B.y);

    return area;
}

void lidarCallback(ConstLaserScanStampedPtr &msg) {

  //  std::cout << ">> " << msg->DebugString() << std::endl;
  float angle_min = float(msg->scan().angle_min());
  //  double angle_max = msg->scan().angle_max();
  float angle_increment = float(msg->scan().angle_step());

  float range_min = float(msg->scan().range_min());
  float range_max = float(msg->scan().range_max());

  int sec = msg->time().sec();
  int nsec = msg->time().nsec();

  int nranges = msg->scan().ranges_size();
  int nintensities = msg->scan().intensities_size();

  assert(nranges == nintensities);

  int width = 600;
  int height = 600;
  float px_per_m = 200 / range_max;

  cv::Mat im(height, width, CV_8UC3);
  im.setTo(0);

  std::vector<cv::Point2f> detectedPoints;

  struct ScanSegment
  {
      std::vector<cv::Point2f> points;
      int type;
  };

  for (int i = 0; i < nranges; i++) {
    float angle = angle_min + i * angle_increment;
    float range = std::min(float(msg->scan().ranges(i)), range_max);
    //    double intensity = msg->scan().intensities(i);

    // Get start point
    cv::Point2f startpt(300.5f + range_min * px_per_m * std::cos(angle),
                        300.5f - range_min * px_per_m * std::sin(angle));

    // Get end point
    cv::Point2f endpt(300.5f + range * px_per_m * std::cos(angle),
                      300.5f - range * px_per_m * std::sin(angle));

    detectedPoints.push_back(endpt);

    // Create line from start to end
    //cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1, cv::LINE_AA, 4);

    //    std::cout << angle << " " << range << " " << intensity << std::endl;
  }

  std::vector<ScanSegment> segments;
#define STRAIGHT 0
#define CURVED 1

  ScanSegment currentSegment;
  int segmentType = 0;

  for(uint i = 0; i < detectedPoints.size(); i++)
  {
      // Break out before out of bounds exception
      if(i == detectedPoints.size() - 1)
      {
          break;
      }

      // Get three points needed for collinearity calculation
      cv::Point2f points [3] = {detectedPoints[i], detectedPoints[i+1], detectedPoints[i+2]};

      // Determine collinearity of points
      float collinearity = GetCollinearity(points);

      //std::cout << collinearity << " ";

      // If linear draw line from first point to second point

      currentSegment.points.push_back(detectedPoints[i]);

      if(collinearity < 0.01f && segmentType == CURVED)
      {
          //currentSegment.points.push_back(detectedPoints[i + 1]);
          currentSegment.type = CURVED;
          segments.push_back(currentSegment);

          segmentType = STRAIGHT;

          currentSegment.points.clear();
          //cv::line(im, points[0] * 16, points[1] * 16, cv::Scalar(255, 255, 0, 255), 1, cv::LINE_AA, 4);
      }

      // If not linear, draw different colour line
      else if(collinearity > 0.01f && segmentType == STRAIGHT)
      {
          currentSegment.points.push_back(detectedPoints[i + 1]);
          currentSegment.type = STRAIGHT;
          segments.push_back(currentSegment);

          segmentType = CURVED;

          currentSegment.points.clear();
          //cv::line(im, points[0] * 16, points[1] * 16, cv::Scalar(0, 0, 255, 255), 1, cv::LINE_AA, 4);
      }
  }

  for(uint i = 0; i < segments.size(); i++)
  {
      ScanSegment segment = segments[i];
      for(uint j = 0; j < segment.points.size() - 1; j++)
      {
          if(segment.type == STRAIGHT)
          {
              cv::line(im, segment.points[j] * 16, segment.points[j + 1] * 16, cv::Scalar(0, 0, 255, 255), 1, cv::LINE_AA, 4);
          }
          else if(segment.type == CURVED)
          {
              cv::line(im, segment.points[j] * 16, segment.points[j + 1] * 16, cv::Scalar(0, 255, 255, 255), 1, cv::LINE_AA, 4);
          }
      }
  }

  cv::circle(im, cv::Point(300, 300), 2, cv::Scalar(0, 0, 255));
  cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
              cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(255, 0, 0));

  mutex.lock();
  cv::imshow("lidar", im);
  mutex.unlock();
}

int main(int _argc, char **_argv) {
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
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

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
  marbel_Controller fuzzy;

  float speed = 0.0;


    // Loop
  while (true) {
    std::cout << cent << std::endl;
    gazebo::common::Time::MSleep(10);



    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();


    if (key == key_esc)
      break;

    dir=fuzzy.buildController(cent);

    if (cent==0){
        speed=0.5;
  }
    else if(cent>=150 && cent<=170 && cent != 0){
        speed= 0.5;
  }
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
