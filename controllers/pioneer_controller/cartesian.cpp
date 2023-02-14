#include "cartesian.h"

bool cartesianIsCoordinateEqual(const vector<double> coordinate1, const vector<double> coordinate2)
{
    if (fabs(coordinate1[0] - coordinate2[0]) < COORDINATE_MATCHING_ACCURACY &&
        fabs(coordinate1[1] - coordinate2[1]) < COORDINATE_MATCHING_ACCURACY)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool cartesianIsAngleEqual( double angle1,  double angle2)
{
    if (fabs(angle1 - angle2) < THETA_MATCHING_ACCURACY)
    {
        return true;
    }
    else
    {
        return false;
    }
}

double convertRadToDegree(double rad){
    return rad*(180/PI);
}

double convertDegreeToRad(double degree){
    return degree*(PI/180);
}