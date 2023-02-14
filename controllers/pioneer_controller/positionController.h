#ifndef POSITION_CONTROLLER_HEADER
#define POSITION_CONTROLLER_HEADER

#include <cmath>
#include <cstring>
#include <webots/Robot.hpp>
#include <webots/Compass.hpp>
#include <webots/gps.hpp>

#include "defs.h"
#include "robotsUtils.h"
#include "cartesian.h"

using namespace webots;
using namespace std;

class PositionController
{
private:
    Robot *robot;
    Compass *compass;
    GPS *gps;

public:
    PositionController(Robot *robot);

    /*
    To get robot coordinate by using gps.
    */
    vector<double> getRobotCoordinates();
    double getCompassReadingInDegrees();
};

#endif