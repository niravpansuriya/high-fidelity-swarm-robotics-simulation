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
#include "communication.h"
#include "TrackerApp.h"
#include "FileManager.h"
using namespace webots;
using namespace std;

class RobotController
{
private:
    Robot *robot;
    MotorController *motorController;
    PositionController *positionController;
    SensorsController *sensorController;
    Communication *communication;
    FileManager *fileManager;
    string robotId;
    string mode; // explore or transport

    // Display *display;

    vector<vector<double>> path;
    llu currentPathIndex;
    int pathSmoothingRadius;

    TrackerApp *trackerApp;

public:
    RobotController(Robot *robot, string robotId);
    void moveToRandomLocation();
    void addLocationToVisitedPath(vector<double> location);
    void followPath(bool forward = true);
    llu getNearestPointIndexForPathSmoothing(bool forward = true);
    string getRobotId();
    bool isDestinationReachable(const vector<double> dest);
    bool gotoDestination();
    string getMode();
    void updateMode(string mode);
    void updatePath(vector<vector<double>> &receivedPath);
    string *getRobotModeReference();
    int getNearestPathIndex(vector<vector<double>> &path);
    vector<vector<double>> &getPath();
    void moveToDestination(const vector<double> destinationCoordinate);
    void turnTowardDestination(const vector<double> destinationCoordinate);
    int middleware();
    void exploreEnvironment();
    // void displayEstimate();
};

#endif