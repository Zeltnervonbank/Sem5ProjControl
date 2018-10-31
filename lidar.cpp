#include "lidar.h"

lidar::lidar()
{

}

void doSomething()
{

}

float GetCollinearity(cv::Point2f points[3])
{
    cv::Point2f A = points[0];
    cv::Point2f B = points[1];
    cv::Point2f C = points[2];

    float area = A.x * (B.y - C.y) + B.x * (C.y - A.y) + C.x * (A.y - B.y);

    return area;
}
