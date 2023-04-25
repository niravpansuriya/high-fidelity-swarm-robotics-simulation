#ifndef DEFS_HEADER
#define DEFS_HEADER

// class Communication;
// class MotorController;

#include <limits>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <utility>
#include <cstring>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <fstream>
#include <webots/Display.hpp>


using namespace std;
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
#define COORDINATE_MATCHING_ACCURACY 0.02 // in meter

#define THETA_MATCHING_ACCURACY 1 // in degrees

#define TOO_CLOSE_DISTANCE 0.5f

#define DISTANCE_COOR_STORING 0.3
#define DISTANCE_NEW_DIRECTION 0.3 

#define PATH_RADIUS_THRESHOLD 0.25 // 1 -> working great

#define MAX_BOT_TURN_RADIAN 0.25
#define MAX_STEP_TIME 100
#define ROBOT_SPEED 6

#define PATH_SMOOTHING_RADIUS 4 // 4 -> working great
#define MIN_PATH_SMOOTHING_RADIUS 0
#define DESTINATION_X 4.96412
#define DESTINATION_Y -12.5933
// #define DESTINATION_X -26.5271
// #define DESTINATION_Y -10.8252

#define COMM_WAITING_TIME 600
#define ESTIMATED_COMM_TIME 10

#define STUCK_WAITING_TIME 60

#define MAX_QUEUE_SIZE 100

// extern Communication *globalCommunication;
// extern MotorController *globalMotorController;
#endif
