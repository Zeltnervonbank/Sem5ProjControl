#ifndef CAMERA_H
#define CAMERA_H
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>

#include "marbel_controller.h"


class Camera
{
public:
    Camera();
    float getMarbelCenter(cv::Mat im);
};

#endif // CAMERA_H
