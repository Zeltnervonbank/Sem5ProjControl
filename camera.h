#ifndef CAMERA_H
#define CAMERA_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>
#include "datatypes.h"
#include <stdlib.h>

#include "marbel_controller.h"
#include "globals.h"


class Camera
{
public:
    Camera();
    static bool marbleClose;

    static MarbleLocation GetMarbleCenter(cv::Mat im);
    static void CameraCallback(ConstImageStampedPtr &msg);
};

#endif // CAMERA_H
