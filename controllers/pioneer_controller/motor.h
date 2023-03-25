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
    void motorRotateRightInDegree(double degrees);
    int getLeftMotorSpeed();
    int getRightMotorSpeed();
    vector<int> getCurrentState();
    void setCurrentSet(vector<int> &state);
    void setSpeed(double left, double right, double a = 1);
    Motor* getLeftMotorReference();
    Motor* getRightMotorReference();
};

#endif