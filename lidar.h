#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>

#ifndef LIDAR_H
#define LIDAR_H


class lidar
{
public:
    lidar();
    static void doSomething();
    float GetCollinearity(cv::Point2f points[3]);
};

#endif // LIDAR_H
