#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "globals.h"
#include "mapping.h"

class Movement
{
public:
    Movement();
    static void PublishPose(ignition::math::Pose3d pose);
    static int HandleKeyboardInput(bool printKey = false);
};

#endif // MOVEMENT_H
