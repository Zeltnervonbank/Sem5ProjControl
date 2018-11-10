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

    float x = (position.posX + 50.0f) * 4.0;
    float y = (position.posY + 50.0f) * 4.0;

    std::cout << "X: " << x << " Y: " << y << std::endl;

    for(size_t i = 0; i < rays.size(); i++)
    {
        if(rays[i].isMaxRange)
        {
            continue;
        }

        int xPos = x + rays[i].range * cos(rays[i].angle) * 4.0;
        int yPos = y + rays[i].range * sin(rays[i].angle) * 4.0;

        img.at<uchar>(yPos, xPos, 0) = 255;
    }

    cv::imshow("Map", img);
}
