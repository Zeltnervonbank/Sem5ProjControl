#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "datatypes.h"
#include <cmath>

#ifndef LIDAR_H
#define LIDAR_H

#define STRAIGHT 0
#define CURVED 1


class lidar
{
public:
    lidar();
    static void lidarCallback(ConstLaserScanStampedPtr &msg);
    static bool marblesPresent;
    static std::vector<LidarMarble> detectedMarbles;
    static LidarMarble nearestMarble;
    static std::vector<LidarRay> lidarRays;
    static LidarRay nearestPoint;

private:
    struct ScanSegment
    {
        std::vector<LidarRay> points;
        int type;
    };

    static float GetCollinearity(cv::Point2f points[3]);
    static std::vector<ScanSegment> GetSegmentsOfScan(std::vector<LidarRay> points);
    static std::vector<LidarRay> GetLidarPoints(ConstLaserScanStampedPtr &msg);
    static cv::Mat DisplayLidarPoints(cv::Mat im, std::vector<LidarRay> points, bool displayMaxRange);
    static cv::Mat DisplayScanSegments(cv::Mat im, std::vector<lidar::ScanSegment> segments);
    static std::vector<LidarMarble> ConvertCirclesToLidarMarbles(std::vector<cv::Vec3f> circles);
    static cv::Mat DisplayCircles(cv::Mat im, std::vector<cv::Vec3f> circles);
    static cv::Mat DisplayLines(cv::Mat im, std::vector<cv::Vec4i> lines);
};


#endif // LIDAR_H
