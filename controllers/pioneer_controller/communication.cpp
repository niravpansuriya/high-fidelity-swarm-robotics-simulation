#include "communication.h"

Communication::Communication(Robot *robot, const string emitterName, const string receiverName)
{
    this->robot = robot;
    emitter = new Emitter(emitterName);
    receiver = new Receiver(receiverName);
    isReceiverEnabled = false;
    emitter->setRange(100);
    emitter->setChannel(-1);
}

void Communication::createPackage(vector<vector<double>> &v, double *res)
{
    int i = 0;
    for (auto vec : v)
    {
        res[i++] = vec[0];
        res[i++] = vec[1];
    }
}

vector<vector<double>> Communication::getDataFromPackage(double *v, int size)
{
    vector<vector<double>> res;

    int i = 0;
    while (i < size)
    {
        res.push_back({v[i++], v[i++]});
    }
    return res;
}

void Communication::enableReceiver()
{
    if (receiver)
    {
        receiver->enable(getRobotTimestep(robot));
        isReceiverEnabled = true;
        while (isReceiverEnabled)
        {
            if (receiver->getQueueLength())
            {
                double *package = (double *)receiver->getData();
                int size = receiver->getDataSize() / sizeof(double);

                // get path vector
                vector<vector<double>> receivedPath = getDataFromPackage(package, size);

                // set up the path
                // robotController->replacePath(receivedPath);

                // // follow path
                // robotController->followPath(true);
            }
        }
    }
}

void Communication::disableReceiver()
{
    if (receiver)
    {
        receiver->disable();
        isReceiverEnabled = false;
    }
}

void Communication::broadcastPath(vector<vector<double>> &path)
{
    // generate package based on path
    double package[path.size() * 2];
    createPackage(path, package);

    // broadcast package
    emitter->send(package, sizeof(double) * 2 * path.size());
}
