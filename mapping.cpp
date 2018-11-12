#include "mapping.h"

mapping::mapping()
{

}

void mapping::UpdateMap(RobotPosition position)
{
    std::vector<LidarRay> rays = lidar::lidarRays;

    if(rays.size() == 0)
    {
        return;
    }

    double x = (position.posX + 50.0) * 8.0;
    double y = (position.posY + 50.0) * 8.0;

    double rW = position.rotW;
    double rX = position.rotX;
    double rY = position.rotY;
    double rZ = position.rotZ;

    double roll = atan2(2.0 * rY * rW - 2.0 * rX * rZ, 2.0 * rY * rY - 2.0 * rZ * rZ);
    double pitch = atan2(2.0 * rX * rW - 2.0 * rY * rZ, 2.0 * rX * rX - 2.0 * rZ * rZ);
    double yaw = asin(2.0 * rX * rY + 2.0 * rZ * rW);

    std::cout
            //<< "X: " << x
            //<< " Y: " << y
            << " Roll: " << roll
            << " Pitch: " << pitch
            << " Yaw: " << yaw
            << std::endl;


    for(size_t i = 0; i < rays.size(); i++)
    {
        if(rays[i].isMaxRange)
        {
            continue;
        }

        int xPos = x + rays[i].range * cos(rays[i].angle) * 8.0;
        int yPos = y + rays[i].range * sin(rays[i].angle) * 8.0;

        img.at<uchar>(800 - yPos, xPos, 0) = 255;
    }

    cv::imshow("Map", img);
}
