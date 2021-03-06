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
    double distance;
    double angle;
    double radius;
};

struct LidarRay
{
    cv::Point2f startPoint;
    cv::Point2f endPoint;
    double range;
    bool isMaxRange;
    double angle;
};

struct RobotPosition
{
    double posX;
    double posY;

    double rotW;
    double rotX;
    double rotY;
    double rotZ;
};

struct Waypoint
{
    double x;
    double y;

    bool isDestination;
    bool isMarble;
};

struct Destination
{
    double x;
    double y;

    int index;
};

struct ControlOutput
{
    float direction;
    float speed;
};

#endif // DATATYPES_H
