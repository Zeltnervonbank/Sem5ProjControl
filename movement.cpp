#include "movement.h"

#define KEY_LEFT 81
#define KEY_UP 82
#define KEY_DOWN 84
#define KEY_RIGHT 83
#define KEY_ESC 27
#define KEY_S 115
#define KEY_L 108

Movement::Movement()
{

}

void Movement::PublishPose(ignition::math::Pose3d pose)
{
    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);

    // And publish it
    Globals::movementPublisher->Publish(msg);
}

// Vars for keyboard input
double dir = 0.0;
double speed = 0.0;

int Movement::HandleKeyboardInput(bool printKey)
{
    Globals::mutex.lock();
    int key = cv::waitKey(1);
    Globals::mutex.unlock();


    if (key == KEY_ESC)
        return -1;

    // Print key pressed
    if(key != 255 && printKey)
    {
        std::cout << key << std::endl;
    }

    if ((key == KEY_UP) && (speed <= 1.2))
        speed += 0.05;
    else if ((key == KEY_DOWN) && (speed >= -1.2))
        speed -= 0.05;
    else if ((key == KEY_RIGHT) && (dir <= 0.4))
        dir += 0.05;
    else if ((key == KEY_LEFT) && (dir >= -0.4))
        dir -= 0.05;
    else if(key == KEY_S)
    {
        mapping::SaveMapToDisk();
    }
    else if(key == KEY_L)
    {
        mapping::LoadMapFromDisk();
    }
    else
    {
        speed *= 0.99;
        dir *= 0.99;
    }

    // Generate a pose
    ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

    Movement::PublishPose(pose);

    return 0;
}
