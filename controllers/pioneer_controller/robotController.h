#ifndef ROBOT_CONTROLLER_HEADER
#define ROBOT_CONTROLLER_HEADER

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <cstring>
#include <vector>
#include "defs.h"
#include "motor.h"
#include "positionController.h"
#include "sensors.h"
using namespace webots;
using namespace std;


class RobotController
{
private:
    Robot *robot;
    MotorController *motorController;
    PositionController *positionController;
    SensorsController *sensorController;

    string mode; // explore or transport

    vector<vector<double>> path;
    llu currentPathIndex;

public:
    RobotController(Robot *robot);
    void moveToRandomLocation();
    void addLocationToVisitedPath(vector<double> location);
    void followPath(bool forward = true);
    llu getNearestPointIndexForPathSmoothing(bool forward = true);

    bool isDestinationReachable(const vector<double> dest);
    bool gotoDestination();
    string getMode();
    void replacePath(vector<vector<double>> path);
    // void broadcastPath();
};

#endif