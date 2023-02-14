#include "positionController.h"

PositionController::PositionController(Robot *robot)
{
    int robotTimestep = getRobotTimestep(robot);

    this->gps = robot->getGPS("gps");
    this->gps->enable(robotTimestep);

    this->compass = robot->getCompass("compass");
    this->compass->enable(robotTimestep);
}

double PositionController::getCompassReadingInDegrees()
{
    const double *readings = compass->getValues();

    double rad = atan2(readings[0], readings[1]);
    double bearing = ((rad - PI / 2) / PI) * 180.0;
    if (bearing > 180)
        bearing = 360 - bearing;
    if (bearing < -180)
        bearing = 360 + bearing;
    return bearing;
}

vector<double> PositionController::getRobotCoordinates()
{
    const double *coordinates = this->gps->getValues();
    vector<double> _2fValues = {coordinates[0], -1 * coordinates[2]};
    return _2fValues;
}