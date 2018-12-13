#include "movement.h"

#define KEY_LEFT 81
#define KEY_UP 82
#define KEY_DOWN 84
#define KEY_RIGHT 83
#define KEY_ESC 27
#define KEY_S 115
#define KEY_L 108
#define KEY_C 99
#define KEY_V 118

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

// Vars for movement
double dir = 0.0;
double speed = 0.0;

// Move with internally set values
void Movement::Move()
{
    // Generate a pose
    ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

    // Publish that pose
    PublishPose(pose);
}

// Move with given values
void Movement::Move(double speed, double rotation)
{
    ignition::math::Pose3d pose(speed, 0, 0, 0, 0, rotation);
    PublishPose(pose);
}

int Movement::HandleMovement()
{
    // Checks keyboard first
    if(HandleKeyboardInput() == -1)
    {
        return -1;
    }

    // If a marble is close, move toward it
    if(Camera::marbleClose)
    {
        dir = 0.0;
        speed = 1.0;
    }

    // If marbles are visible, and within a certain range, move toward it
    else if(lidar::marblesPresent && lidar::nearestMarble.distance < 1000 && lidar::nearestMarble.angle<4)
    {
        std::cout << "                  marble" << std::endl;

        dir = marbleController.getControlOutput(
                    lidar::nearestMarble.angle,
                    lidar::nearestMarble.distance
                    ).direction;

        speed = marbleController.getControlOutput(
                    lidar::nearestMarble.angle,
                    lidar::nearestMarble.distance
                    ).speed;
    }
    // If we're about to move into an obstacle, don't
    else if(lidar::nearestPoint.range < 0.5 && abs(lidar::nearestPoint.angle) <= 1.56)
    {
        std::cout << "                  wall" << std::endl;
        dir = wallController.getControlOutput(
                    lidar::nearestPoint.angle,
                    lidar::nearestPoint.range
                    ).direction;

        speed = wallController.getControlOutput(
                    lidar::nearestPoint.angle,
                    lidar::nearestPoint.range
                    ).speed;
    }

    // If no marbles are visible, use A* to move to next waypoint
    else if(Globals::GetDistanceToWaypoint() < 0.1 && Globals::waypoints.size() > 0)
    {
        // Get the next waypoint from queue, and stop robot
        Globals::CurrentWaypoint = Globals::waypoints.front();
        Globals::waypoints.pop();
        //Movement::Move(0.0, 0.0);
    }
else{
    std::cout << "                  path" << std::endl;
        dir = wayController.getControlOutput().direction;
        speed = wayController.getControlOutput().speed;
    }

/*
    else{
        dir = 0.0;
        speed = 1.2;
        }
*/
        // TODO: Implement A* pathfollowing here

    std::cout << "dirr" << dir << "speed:" << speed << std::endl;
    Move();
}


int Movement::HandleKeyboardInput()
{
    Globals::mutex.lock();
    int key = cv::waitKey(1);
    Globals::mutex.unlock();

    // Break out if esc pressed
    if (key == KEY_ESC)
    {
        return -1;
    }

    // If enabled, print the pressed key
    if(key != 255 && printKeyPresses)
    {
        std::cout << key << std::endl;
    }

    if ((key == KEY_UP) && (speed <= 1.2))        
    {
        speed += 0.05;
    }
    else if ((key == KEY_DOWN) && (speed >= -1.2))
    {
        speed -= 0.05;
    }
    else if ((key == KEY_RIGHT) && (dir <= 0.4))
    {
        dir += 0.05;
    }
    else if ((key == KEY_LEFT) && (dir >= -0.4))
    {
        dir -= 0.05;
    }
    else if(key == KEY_S)
    {
        mapping::SaveMapToDisk();
    }
    else if(key == KEY_L)
    {
        mapping::LoadMapFromDisk();
    }

    // Testing feature
    else if(key == KEY_C && !testMode)
    {
        if(visited >= 3)
        {
            qLearn.run();
            runs++;
            visited = 0;
            std::cout << "runs: " << runs << std::endl;
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = end - start;

        qLearn.chooseAction(qLearn.currentState, marblePoint, diff.count());
        qLearn.printR();

        marblePoint = 0;
        visited++;
    }
    else if(key == KEY_V && !testMode)
    {
        qLearn.printroute();
    }

    // Slow down if set to do so
    else if(allowPassiveSlowing)
    {
        speed *= 0.99;
        dir *= 0.99;
    }    

    return 0;
}
