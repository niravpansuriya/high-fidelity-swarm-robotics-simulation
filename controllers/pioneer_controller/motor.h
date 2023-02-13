#ifndef MOTOR_HEADER
#define MOTOR_HEADER

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Compass.hpp>
#include <cmath>
#include <cstring>
#include <vector>
#include "defs.h"
#include "positionController.h"

using namespace webots;
using namespace std;

class MotorController
{
private:
    Robot *robot;
    Motor *leftMotor, *rightMotor;
    PositionController *positionController;

public:
    MotorController(Robot *robot);
    void stopMotor();
    void motorMoveForward(int speed = 0);
    void motorRotateRight(int speed = 0);
    void motorRotateLeft(int speed = 0);
    void motorRotateLeftInDegree(double degrees);
    void moveToDestination(const double destinationCoordinate[2]);
    void turnTowardDestination(const double destinationCoordinate[2]);
};

#endif