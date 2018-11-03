#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifndef DATATYPES_H
#define DATATYPES_H

struct MarbleLocation
{
    int center;
    int radius;
};

struct LidarMarble
{
    float distance;
    float angle;
    float radius;
};

struct LidarRay
{
    cv::Point2f startPoint;
    cv::Point2f endPoint;
    float range;
    bool isMaxRange;
    float angle;
};

#endif // DATATYPES_H
