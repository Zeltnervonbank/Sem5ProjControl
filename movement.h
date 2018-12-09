#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>

#include "globals.h"
#include "mapping.h"
#include "qlearning.h"
#include "marbel_controller.h"
#include "wall_controller.h"

class Movement
{
public:
    Movement();

    static Qlearning qLearn;
    // Make marble controller (Marble collection)- TODO: Rename
    static marbel_Controller marbleController;
    static wall_Controller wallController;


    static int runs;
    static int visited;
    static int marblePoint;
    static std::chrono::_V2::steady_clock::time_point start;

    static bool testMode;
    static bool allowPassiveSlowing;
    static bool printKeyPresses;

    static void PublishPose(ignition::math::Pose3d pose);
    static void Move();
    static void Move(double speed, double rotation);
    static int HandleMovement();
    static int HandleKeyboardInput();
};

#endif // MOVEMENT_H
