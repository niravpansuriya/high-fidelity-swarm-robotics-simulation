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

bool cartesianIsAngleEqual(double angle1, double angle2)
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

double getDistance(vector<double> p1, vector<double> p2)
{
    return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2));
}

double getPathLength(vector<vector<double>> &path)
{
    double distance = 0;

    for (int i = 1; i < path.size(); i++)
    {
        distance += getDistance(path[i - 1], path[i]);
    }

    return distance;
}

double convertRadToDegree(double rad)
{
    return rad * (180 / PI);
}

double convertDegreeToRad(double degree)
{
    return degree * (PI / 180);
}

vector<vector<double>> getPerimeterCoords(double x, double y, double rad)
{
    vector<vector<double>> perimeterPoints =
        {
            {x, y - rad},
            {x + rad, y},
            {x, y + rad},
            {x - rad, y}};

    double xTemp = rad;
    double yTemp = 0;
    double p = 1 - rad;

    while (xTemp > yTemp)
    {
        yTemp += 1;
        if (p <= 0)
            p = p + 2 * yTemp + 1;
        else
        {
            xTemp -= 1;
            p = p + 2 * yTemp - 2 * xTemp + 1;
        }

        if (xTemp < yTemp)
            break;

        perimeterPoints.push_back({xTemp + x, yTemp + y});
        perimeterPoints.push_back({-xTemp + x, yTemp + y});
        perimeterPoints.push_back({xTemp + x, -yTemp + y});
        perimeterPoints.push_back({-xTemp + x, -yTemp + y});

        if (x != y)
        {
            perimeterPoints.push_back({yTemp + x, xTemp + y});
            perimeterPoints.push_back({-yTemp + x, xTemp + y});
            perimeterPoints.push_back({yTemp + x, -xTemp + y});
            perimeterPoints.push_back({-yTemp + x, -xTemp + y});
        }
    }

    return perimeterPoints;
}

vector<pair<vector<double>, llu>> getPerimeterCoordsWithPathCoords(vector<vector<double>> &robotPath,
                                                                   vector<vector<double>> &perimeterCoords,
                                                                   llu currentPathIndex,
                                                                   bool forward)
{
    vector<pair<vector<double>, llu>> v;

    // for each perimeter coords
    for (vector<double> pC : perimeterCoords)
    {
        if (forward)
        {
            for (llu i = currentPathIndex + 1; i < robotPath.size(); i++)
            {
                vector<double> rpc = robotPath[i];
                if (cartesianIsCoordinateEqual(rpc, pC))
                {
                    pair<vector<double>, llu> t(pC, i);
                    v.push_back(t);
                    break;
                }
            }
        }
        else
        {
            for (int i = currentPathIndex - 1; i >= 0; i--)
            {
                vector<double> rpc = robotPath[i];
                if (cartesianIsCoordinateEqual(rpc, pC))
                {
                    pair<vector<double>, int> t(pC, i);
                    v.push_back(t);
                    break;
                }
            }
        }
    }

    return v;
}
