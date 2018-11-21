#ifndef MAPPING_H
#define MAPPING_H
#include "datatypes.h"
#include "lidar.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define MAP_SIDE_LENGTH 800

class mapping
{
public:
    mapping();

    static int map[MAP_SIDE_LENGTH][MAP_SIDE_LENGTH];

    static cv::Mat img;

    static void UpdateMap(RobotPosition position);
    static void SaveMapToDisk();
    static void LoadMapFromDisk();
};

#endif // MAPPING_H
