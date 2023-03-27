#ifndef COMMUNICATION_HEADER
#define COMMUNICATION_HEADER

#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include "robotsUtils.h"
// #include "robotController.h"
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
    bool isCommunicating;
    unordered_map<string, double> priority;
    string partnerId;
    double commStartTime;
    string partnerMode;
    string *robotMode;
    string robotId;
    double lastBroadcastTime;
    double lastAcceptMessageSentTime;
    int ccc;

public:
    Communication(Robot *robot, const string emitterName, const string receiverName, string robotId, string *robotMode);

    string createPackage(vector<vector<double>> &path);
    static vector<vector<double>> getDataFromPackage(double *v, int size);
    void enableReceiver();
    void disableReceiver();

    void broadcastPath(vector<vector<double>> &path);
    int interceptReceiver(string robotId, string robotMode);
    bool shouldCommunicate(string id, string mode);
    bool isPartnerNear();

    string getMessageType(const string &message);
    string getPartnerId(const string &message);
    string getPartnerMode(const string &message);
    void sendMessage(string message);
    void handleReject();
    string getNextMessage();
    void broadcastWaitMessage();
    bool establishConnection();
    bool establishConnectionV2();
    bool establishConnectionV3();
    int getCommunicationChannel(const string &robotId, const string &partnerId);
    bool receivePath(vector<vector<double>> &partnerPath);
    void sendAcceptMessage();
    vector<vector<double>> getPathFromString(const string &path);
    void addPartnerInPriority(const string &partnerId);
    string getReceiverId(const string &message);
    int getQueueLength();
    void clearQueue();
    bool isMessageForMe(const string &message);
    // bool exchangePath(vector<vector<double>> &path, vector<vector<double>> &partnerPath);

};

#endif