#include "cartesian.h"

bool cartesianIsCoordinateEqual(const double coordinate1[2], const double coordinate2[2])
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
