#ifndef MAPPING_H
#define MAPPING_H
#include "datatypes.h"
#include "lidar.h"
#include <vector>
#include <iostream>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class mapping
{
public:
    mapping();

    static int map[400][400];

    static cv::Mat img;

    static void UpdateMap(RobotPosition position);
};

#endif // MAPPING_H
