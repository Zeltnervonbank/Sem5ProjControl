#include "mapping.h"
#define PI 3.14159

mapping::mapping()
{

}

// Updates map based on a given position
void mapping::UpdateMap(RobotPosition position)
{
    if(!mappingEnabled)
    {
        return;
    }
    // Make copy static rays vector TODO: Add lock
    std::vector<LidarRay> rays = Lidar::lidarRays;

    // Return if no rays
    if(rays.size() == 0)
    {
        return;
    }

    // Convert coordinate to position in positive area
    double x = (position.posX + 50.0) * MAP_SIDE_LENGTH / 100;
    double y = (position.posY + 50.0) * MAP_SIDE_LENGTH / 100;

    // Get the yaw angle from orientation Quaternion
    double yaw = fmod(2.0 * atan2(position.rotW, position.rotZ) + 3.0 * M_PI, 2.0 * M_PI);

    for(size_t i = 0; i < rays.size(); i++)
    {
        // Ignore rays with max length
        if(rays[i].isMaxRange)
        {
            continue;
        }

        // Get x and y position of the ray's endpoint, taking rotation into account
        int xPos = x + rays[i].range * cos(rays[i].angle - yaw) * MAP_SIDE_LENGTH / 100;
        int yPos = y + rays[i].range * sin(rays[i].angle - yaw) * MAP_SIDE_LENGTH / 100;

        // Write that point to array
        map[xPos][yPos] = 1;

        // Set that pixel in image to white
        img.at<uchar>(MAP_SIDE_LENGTH - yPos, xPos, 0) = 255;
    }

    cv::imshow("Map", img);
}

// Saves created map to disk
void mapping::SaveMapToDisk()
{
    if(!mappingEnabled)
    {
        return;
    }

    // Set output path TODO: Use path that won't just work for me
    const char *path="/home/andreas/Desktop/map.txt";
    std::cout << "Attempting to save map" << std::endl;

    // Open stream to path
    std::ofstream mapStream(path);

    // If that worked
    if(mapStream.is_open())
    {
        // Go through each pixel
        for(int x = 0; x < MAP_SIDE_LENGTH; x++)
        {
            for(int y = 0; y < MAP_SIDE_LENGTH; y++)
            {
                // Output that pixel's value
                mapStream << map[y][x];
            }

            // End lines
            mapStream << "\n";
        }

        mapStream.close();
    }
    // Report error
    else
    {
        std::cout << "Could not open stream" << std::endl;
    }
    std::cout << "Finished saving attempt" << std::endl;
}

void mapping::LoadMapFromDisk()
{
    if(!mappingEnabled)
    {
        return;
    }

    std::ifstream stream ("/home/andreas/Desktop/map.txt");

    if(stream.is_open())
    {
        for(int y = 0; y < MAP_SIDE_LENGTH; y++)
        {
            int x = 0;
            int val = 0;
            while(val != -38)
            {
                val = stream.get() - '0';
                if(val != -38)
                {
                    std::cout << val;
                }
                x++;
            }
        }

        stream.close();
    }
}
