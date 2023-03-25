#include <motor.h>

MotorController::MotorController(Robot *robot)
{
    this->robot = robot;
    this->leftMotor = robot->getMotor("left wheel");
    this->rightMotor = robot->getMotor("right wheel");
    this->leftMotor->setPosition(INFINITE);
    this->rightMotor->setPosition(INFINITE);
    this->leftMotor->setVelocity(0.0);
    this->rightMotor->setVelocity(0.0);

    this->positionController = new PositionController(robot);
}

int MotorController::getLeftMotorSpeed()
{
    return leftMotor->getVelocity();
}
int MotorController::getRightMotorSpeed()
{
    return rightMotor->getVelocity();
}

vector<int> MotorController::getCurrentState()
{
    return {getLeftMotorSpeed(), getRightMotorSpeed()};
}
void MotorController::setCurrentSet(vector<int> &state)
{
    leftMotor->setVelocity(state[0]);
    rightMotor->setVelocity(state[0]);
}

void MotorController::stopMotor()
{
    // this->leftMotor->setVelocity(0.0);
    // this->rightMotor->setVelocity(0.0);
    setSpeed(0, 0);
}

void MotorController::motorRotateRight(int speed)
{
    this->leftMotor->setVelocity(speed ? speed : MAX_SPEED);
    this->rightMotor->setVelocity(-1 * (speed ? speed : MAX_SPEED));
    // setSpeed(speed ? speed : MAX_SPEED, -1 * (speed ? speed : MAX_SPEED));
    // setSpeed(speed ? speed : MAX_SPEED, -1 * (speed ? speed : MAX_SPEED));
}

void MotorController::motorMoveForward(int speed)
{
    // this->leftMotor->setVelocity(speed ? speed : MAX_SPEED);
    // this->rightMotor->setVelocity(speed ? speed : MAX_SPEED);
    setSpeed(speed ? speed : MAX_SPEED, speed ? speed : MAX_SPEED);
}

void MotorController::motorRotateLeft(int speed)
{
    this->leftMotor->setVelocity(-1 * (speed ? speed : MAX_SPEED));
    this->rightMotor->setVelocity(speed ? speed : MAX_SPEED);
    // setSpeed(-1 * (speed ? speed : MAX_SPEED), speed ? speed : MAX_SPEED);
}

Motor *MotorController::getLeftMotorReference()
{
    return leftMotor;
}
Motor *MotorController::getRightMotorReference()
{
    return rightMotor;
}

void MotorController::setSpeed(double left, double right, double a)
{
    int timeStep = getRobotTimestep(robot);
    double cls = getLeftMotorSpeed();
    double crs = getRightMotorSpeed();
    while (robot->step(timeStep) != -1)
    {
        if (cls == left && crs == right)
            return;

        if (cls < left)
        {
            cls = left - cls > a ? cls + a : left;
            // cls++;
            leftMotor->setVelocity(cls);
        }
        else if (cls > left)
        {
            cls = cls - left > a ? cls - a : left;
            // cls--;
            leftMotor->setVelocity(cls);
        }
        if (crs > right)
        {
            crs = crs - right > a ? crs - a : right;
            // crs--;
            rightMotor->setVelocity(crs);
        }
        else if (crs < right)
        {
            // crs++;
            crs = right - crs > a ? crs + a : right;
            rightMotor->setVelocity(crs);
        }
    }
}
void MotorController::motorRotateLeftInDegree(double degrees)
{
    int timeStep = (int)robot->getBasicTimeStep();

    // this->motorRotateLeft(0);
    // this->motorRotateRight(1);
    setSpeed(0,1);
    double initAngle = positionController->getCompassReadingInDegrees();
    if (initAngle < 0)
        initAngle += 360;

    double desiredAngle = fModulo((initAngle + degrees), 360);

    double currentAngle = positionController->getCompassReadingInDegrees();
    if (currentAngle < 0)
        currentAngle += 360;

    do  
    {

        this->robot->step(timeStep);
        currentAngle = positionController->getCompassReadingInDegrees();
        if (currentAngle < 0)
            currentAngle += 360;
    } while (!cartesianIsAngleEqual(currentAngle, desiredAngle));

    // this->stopMotor();
}

void MotorController::motorRotateRightInDegree(double degrees)
{
    int timeStep = (int)robot->getBasicTimeStep();

    // this->motorRotateRight(0);
    // this->motorRotateLeft(1);
    setSpeed(1,0);

    double initAngle = positionController->getCompassReadingInDegrees();
    if (initAngle < 0)
        initAngle += 360;

    double desiredAngle = initAngle - degrees;
    while (desiredAngle < 0)
        desiredAngle += 360;

    double currentAngle = positionController->getCompassReadingInDegrees();
    if (currentAngle < 0)
        currentAngle += 360;

    do
    {
        this->robot->step(timeStep);
        currentAngle = positionController->getCompassReadingInDegrees();
        if (currentAngle < 0)
            currentAngle += 360;
    } while (!cartesianIsAngleEqual(currentAngle, desiredAngle));

    // this->stopMotor();
}
