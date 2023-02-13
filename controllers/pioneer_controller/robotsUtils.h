#ifndef ROBOT_UTILS_HEADER
#define ROBOT_UTILS_HEADER

#include <webots/Robot.hpp>
#include "defs.h"

using namespace webots;
using namespace std;

int getRobotTimestep(Robot *robot);
double fModulo(double n, double m);

#endif