#include <limits>
#include <cmath>

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
