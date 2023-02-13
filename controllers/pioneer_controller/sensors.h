#ifndef SENSORS_HEADER
#define SENSORS_HEADER

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <cstring>
#include <vector>
#include "defs.h"
#include "robotsUtils.h"

using namespace webots;
using namespace std;

class SensorsController
{
private:
    Robot *robot;
    // string distanceSensorsName[NUMBER_OF_DISTANCE_SENSORS] = {"ps0", "sops1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
    string distanceSensorsName[NUMBER_OF_DISTANCE_SENSORS] = {"so0", "so1", "so2", "so3", "so4", "so5", "so6", "so7",
                                                              "so8", "so9", "so10", "so11", "so12", "so13", "so14", "so15"};
    DistanceSensor *distanceSensors[NUMBER_OF_DISTANCE_SENSORS];

public:
    SensorsController(Robot *robot);
    void initSensors();
    vector<bool> getSensorsCondition();
    bool isSomethingInFront();
    bool isSomethingInFrontLeft();
    bool isSomethingInFrontRight();
};

#endif