#include "defs.h"

using namespace std;

/*
 * in real world, there must be gps noise so the coordinate is not accurate
 * so to check if two coordinate are equal, we cannot check with formula: coordinate1==coordinate2
 * we must use a threshold accuracy
 * */
bool cartesianIsCoordinateEqual(const vector<double> coordinate1, const vector<double> coordinate2);

/*
 * in real world, there must be compass noise so the heading is not accurate
 * so to check if two heading are equal, we cannot check with formula: heading1==heading2
 * we must use a threshold accuracy
 * */
bool cartesianIsAngleEqual(double angle1, double angle2);

double convertRadToDegree(double rad);
double convertDegreeToRad(double degree);
double getDistance(vector<double> p1, vector<double> p2);

vector<vector<double>> getPerimeterCoords(double x, double y, double rad);

vector<pair<vector<double>, llu>> getPerimeterCoordsWithPathCoords(vector<vector<double>> &robotPath,
                                                                   vector<vector<double>> &perimeterCoords,
                                                                   llu currentPathIndex,
                                                                   bool forward = true);