#include "mapping.h"
#define PI 3.14159

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

    double x = (position.posX + 50.0) * MAP_SIDE_LENGTH / 100;
    double y = (position.posY + 50.0) * MAP_SIDE_LENGTH / 100;

    double yaw = fmod(2.0 * atan2(position.rotW, position.rotZ) + 3.0 * M_PI, 2.0 * M_PI);

    for(size_t i = 0; i < rays.size(); i++)
    {
        if(rays[i].isMaxRange)
        {
            continue;
        }

        int xPos = x + rays[i].range * cos(rays[i].angle - yaw) * MAP_SIDE_LENGTH / 100;
        int yPos = y + rays[i].range * sin(rays[i].angle - yaw) * MAP_SIDE_LENGTH / 100;


        map[xPos][yPos] = 1;
        img.at<uchar>(MAP_SIDE_LENGTH - yPos, xPos, 0) = 255;
    }

    cv::imshow("Map", img);
}

void mapping::SaveMapToDisk()
{
    const char *path="/home/andreas/Desktop/map.txt";
    std::cout << "Attempting to save map" << std::endl;
    std::ofstream mapStream(path);
    if(mapStream.is_open())
    {
        for(int x = 0; x < MAP_SIDE_LENGTH; x++)
        {
            for(int y = 0; y < MAP_SIDE_LENGTH; y++)
            {
                mapStream << map[y][x];
            }

            mapStream << "\n";
        }

        mapStream.close();
    }
    else
    {
        std::cout << "Could not open stream" << std::endl;
    }
    std::cout << "Finished saving attempt" << std::endl;
}
