#include "defs.h"

using namespace std;

/*
 * in real world, there must be gps noise so the coordinate is not accurate
 * so to check if two coordinate are equal, we cannot check with formula: coordinate1==coordinate2
 * we must use a threshold accuracy
 * */
bool cartesianIsCoordinateEqual(const double coordinate1[2], const double coordinate2[2]);

/*
 * in real world, there must be compass noise so the heading is not accurate
 * so to check if two heading are equal, we cannot check with formula: heading1==heading2
 * we must use a threshold accuracy
 * */
bool cartesianIsAngleEqual(double angle1, double angle2);
