#include <sensors.h>

SensorsController::SensorsController(Robot *robot)
{
    this->robot = robot;
}

void SensorsController::initSensors()
{
    int timeStep = getRobotTimestep(robot);
    for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
    {
        this->distanceSensors[i] = this->robot->getDistanceSensor(this->distanceSensorsName[i]);
        this->distanceSensors[i]->enable(timeStep);
    }
}

vector<bool> SensorsController::getSensorsCondition()
{
    vector<bool> sensorsConditions;
    // cout<<endl;
    for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
    {
        double val = getDistanceSensorReading(i);
            // cout << i << " " << val << endl;

        if (val > 0 && val < TOO_CLOSE_DISTANCE)
            sensorsConditions.push_back(true);
        else
            sensorsConditions.push_back(false);
    }
    // cout<<endl;

    return sensorsConditions;
}

double SensorsController::getDistanceSensorReading(int i)
{
    double val = distanceSensors[i]->getValue();
    return val < 0 ? 99 : val;
}

bool SensorsController::isSomethingInFront()
{
    vector<bool> distanceSensorsCondition = getSensorsCondition();
    return distanceSensorsCondition[3] || distanceSensorsCondition[4];
}

bool SensorsController::isSomethingInFrontLeft()
{
    vector<bool> distanceSensorsCondition = getSensorsCondition();
    return distanceSensorsCondition[1] || distanceSensorsCondition[3] || distanceSensorsCondition[2];
}

bool SensorsController::isSomethingInFrontRight()
{
    vector<bool> distanceSensorsCondition = getSensorsCondition();
    return distanceSensorsCondition[4] || distanceSensorsCondition[6] || distanceSensorsCondition[5];
}

double SensorsController::getDistanceAtFrontRight()
{
    double distance = min(distanceSensors[4]->getValue(), min(distanceSensors[5]->getValue(), distanceSensors[6]->getValue()));
    return distance < 0 ? 99 : distance;
}

double SensorsController::getDistanceAtFrontLeft()
{
    double distance = min(distanceSensors[1]->getValue(), min(distanceSensors[2]->getValue(), distanceSensors[3]->getValue()));
    return distance < 0 ? 99 : distance;
}

double SensorsController::getDistanceAtFront()
{
    double distance = min(distanceSensors[3]->getValue(), distanceSensors[4]->getValue());
    ;
    return distance < 0 ? 99 : distance;
}

bool SensorsController::isSomethingInLeft()
{
    vector<bool> distanceSensorsCondition = getSensorsCondition();
    return distanceSensorsCondition[0];
}

bool SensorsController::isSomethingInRight()
{
    vector<bool> distanceSensorsCondition = getSensorsCondition();
    return distanceSensorsCondition[7];
}