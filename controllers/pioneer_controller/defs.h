#include <limits>
#include <cmath>
#include <vector>
#include <utility>
#include "robotsUtils.h"

#define llu long long
#define INFINITE std::numeric_limits<double>::infinity()
#define PI 3.14


#define MAX_SPEED 6.28
// #define ROBOT_ANGULAR_SPEED_IN_DEGREES 283.588111888
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 359.8175
#define NUMBER_OF_DISTANCE_SENSORS 16
#define SENSOR_VALUE_DETECTION_THRESHOLD 140


/* 
 * in real world, there must be gps noise so the coordinate is not accurate
 * so to check if two coordinate are equal, we cannot check with formula: coordinate1==coordinate2
 * we must use a threshold accuracy
 * */
#define COORDINATE_MATCHING_ACCURACY 0.02 //in meter

#define THETA_MATCHING_ACCURACY 1 //in degrees

#define TOO_CLOSE_DISTANCE 0.2f

#define MAX_BOT_TURN_RADIAN 0.25
#define MAX_STEP_TIME 40
#define ROBOT_SPEED 6

#define PATH_SMOOTHING_RADIUS 5

#define DESTINATION_X -18.2903
#define DESTINATION_Y -7.7024