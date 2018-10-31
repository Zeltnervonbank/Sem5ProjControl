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

#define STRAIGHT 0
#define CURVED 1


class lidar
{
public:
    lidar();
    static void lidarCallback(ConstLaserScanStampedPtr &msg);

private:
    struct ScanSegment
    {
        std::vector<cv::Point2f> points;
        int type;
    };

    static float GetCollinearity(cv::Point2f points[3]);
    static std::vector<ScanSegment> GetSegmentsOfScan(std::vector<cv::Point2f> points);

};

#endif // LIDAR_H
