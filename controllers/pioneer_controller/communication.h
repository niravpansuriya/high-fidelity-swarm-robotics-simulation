#ifndef COMMUNICATION_HEADER
#define COMMUNICATION_HEADER

#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include "defs.h"

using namespace webots;
using namespace std;

class Communication
{
private:
    Robot *robot;
    Emitter *emitter;
    Receiver *receiver;
    bool isReceiverEnabled;

public:
    Communication(Robot *robot, const string emitterName, const string receiverName);

    static void createPackage(vector<vector<double>> &v, double *res);
    static vector<vector<double>> getDataFromPackage(double *v, int size);
    void enableReceiver();
    void disableReceiver();

    void broadcastPath(vector<vector<double>> &path);
};

#endif