#include "communication.h"

Communication::Communication(Robot *robot, const string emitterName, const string receiverName, string robotId, string *robotMode)
{
    this->robot = robot;
    emitter = new Emitter(emitterName);
    receiver = new Receiver(receiverName);
    isReceiverEnabled = false;
    isCommunicating = false;
    emitter->setRange(2);
    emitter->setChannel(0);
    receiver->setChannel(0);
    partnerId = "";
    this->robotMode = robotMode;
    this->robotId = robotId;
    string partnerMode;
    this->lastBroadcastTime = -1;
    lastAcceptMessageSentTime = -1;
    ccc = 0;
}

void Communication::addPartnerInPriority(const string &partnerId)
{
    priority[partnerId] = robot->getTime();
}

string Communication::getMessageType(const string &message)
{
    vector<string> splittedString = split(message, '_');
    return splittedString[0];
}

string Communication::getPartnerId(const string &message)
{
    vector<string> splittedString = split(message, '_');
    return splittedString[1];
}

string Communication::getReceiverId(const string &message)
{
    vector<string> splittedString = split(message, '_');
    return splittedString[2];
}

string Communication::getPartnerMode(const string &message)
{
    vector<string> splittedString = split(message, '_');
    return splittedString[2];
}

// string Communication::createPackage(vector<vector<double>> &path)
// {
//     string package = "PATH_1_";

//     for (int i = 0; i < path.size(); i++)
//     {
//         vector<double> coor = path[i];
//         package += to_string(coor[0]) + ",";
//         package += to_string(coor[1]);
//         if (i != path.size() - 1)
//             package += ',';
//     }

//     return package;
// }

vector<string> Communication::createPackage(vector<vector<double>> &path)
{
    int SIZE = 20;
    vector<string> chunks;

    string package = "PATH_1_";

    for (int i = 0; i < path.size(); i++)
    {
        if (i != 0 && i % SIZE == 0)
        {
            chunks.push_back(package);
            package = "PATH_" + to_string(i / SIZE + 1) + "_";
        }
        vector<double> coor = path[i];
        package += to_string(coor[0]) + ",";
        package += to_string(coor[1]);
        if (i != path.size() - 1)
            package += ',';
    }

    chunks.push_back(package);

    return chunks;
}

vector<vector<double>> Communication::getPathFromString(const string &path)
{
    // const string pathString = split(path, '_')[2];
    const string pathString = path;
    vector<string> pathArray = split(pathString, ',');

    vector<vector<double>> res;
    for (int i = 0; i < pathArray.size(); i += 2)
    {
        res.push_back({stod(pathArray[i]), stod(pathArray[i + 1])});
    }

    cout << res.size() << " " << res[res.size() - 1][0] << " " << res[res.size() - 1][1] << endl;
    return res;
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

void Communication::sendAcceptMessage()
{
    if (robot->getTime() - lastAcceptMessageSentTime >= 1)
    {
        sendMessage("ACCEPT_" + robotId);
        lastAcceptMessageSentTime = robot->getTime();
    }
}

void Communication::enableReceiver()
{
    if (receiver)
    {
        receiver->enable(getRobotTimestep(robot));
        isReceiverEnabled = true;
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

// void Communication::broadcastPath(vector<vector<double>> &path)
// {
//     // if robot's mode is explore, then do not broadcast the path
//     if (*robotMode == "explore")
//     {
//         return;
//     }

//     cout << robot->getName() << " broadcasting path" << endl;
//     // generate package based on path
//     string pathPackage = createPackage(path);
//     // string pathPackage = "PATH_1_Message Message Message";
//     cout << robot->getName() << " broadcasting path done" << endl;

//     sendMessage(pathPackage);

//     sendMessage("END_" + robotId);
//     // cout << robot->getName() << " Path sending done" << endl;
// }

bool Communication::broadcastPath(vector<vector<double>> &path)
{
    // if robot's mode is explore, then do not broadcast the path
    if (*robotMode == "explore")
    {
        return false;
    }

    cout << robot->getName() << " broadcasting path " <<path[path.size()-1][0]<<" "<<path[path.size()-1][1]<< endl;
    // generate package based on path
    vector<string> pathPackage = createPackage(path);
    // string pathPackage = "PATH_1_Message Message Message";

    for (string chunk : pathPackage)
    {
        sendMessage(chunk);
    }

    sendMessage("END_" + robotId);

    cout << robot->getName() << " broadcasting path done" << endl;

    return true;
    // cout << robot->getName() << " Path sending done" << endl;
}

bool Communication::shouldCommunicate(string id, string mode)
{
    // return false;
    
    if(robot->getTime() <= 900) return false;
      
    if (mode == "explore" && *robotMode == "explore")
    {
        return false;
    }

    if(mode == "explore")   return true;
    if(*robotMode == "explore") return true;
    return false;

    if (priority.count(id))
        return ((robot->getTime() - priority[id]) >= COMM_WAITING_TIME);
    else
        return true;
}

int Communication::interceptReceiver(string robotId, string robotMode)
{
    // get next package
    if (receiver)
    {
        if (receiver->getQueueLength())
        {
            string *dataStringPointer = (string *)(receiver->getData());
            const string dataString = *dataStringPointer;
            vector<string> data = split(dataString, '_');
            int size = receiver->getDataSize();
            receiver->nextPacket();
            if (isCommunicating)
            {
            }
            else
            {
                // check if this is wait signal
                if (data[0] == "wait")
                {
                    string id = data[1];
                    if (shouldCommunicate(id, "transport"))
                    {
                        priority[id] = robot->getTime();
                    }
                    else
                    {
                        // send reject signal
                    }
                }
            }
        }
    }

    return 0;
}

void Communication::sendMessage(string message)
{
    const char *messagePointer = message.c_str();
    emitter->send(messagePointer, message.length() * sizeof(char));
}

void Communication::broadcastWaitMessage()
{
    // if (isCommunicating)
    //     return;

    if (lastBroadcastTime == -1 || robot->getTime() - lastBroadcastTime >= 1)
    {
        lastBroadcastTime = robot->getTime();
    }
    else
        return;

    // emitter->setRange(1);
    // cout<<robot->getName()<<" sent broadcast message"<<endl;
    string message = "WAIT_" + robotId + "_" + (*robotMode);
    sendMessage(message);
}

// bool Communication::exchangePath(vector<vector<double>> &path, vector<vector<double>> &partnerPath)
// {
//     bool doesPartnerReceivedPath = *robotMode == "explore";
//     bool isPartnerSentPath = partnerMode == "explore";
//     double pathSharedTime = -1;
//     double startTime = robot->getTime();
//     int timestep = getRobotTimestep(robot);
//     while (robot->step(timestep) != -1)
//     {
//         if (robot->getTime() - startTime >= 8)
//         {
//             cout << robot->getTime() << " I have spent enough time in the room, not going to spend more time" << endl;
//             return;
//         }

//         if (!doesPartnerReceivedPath)
//         {
//             if (pathSharedTime == -1 || robot->getTime() - pathSharedTime >= 2)
//             {
//                 // send path
//                 broadcastPath(path);
//                 pathSharedTime = robot->getTime();
//             }
//         }

//         // receive the message
//         if(!receiver->getQueueLength())
//         string data = getNextMessage();

//     }
// }

bool Communication::receivePath(vector<vector<double>> &partnerPath)
{
    // if parther's mode is explore mode, then partner will not send the path
    if (partnerMode == "explore")
    {
        // cout << robot->getName() << " partner's mode is explore, so I am not assuming the path" << endl;
        priority[partnerId] = robot->getTime();
        handleReject();

        return false;
    }

    // cout << robot->getName() << " looking for path" << endl;
    double startTime = robot->getTime();
    int timestep = getRobotTimestep(robot);
    int count = 0;
    unordered_map<int, string> m;

    while (robot->step(timestep) != -1)
    {
        // cout << "innn" << endl;
        if (!receiver->getQueueLength())
        {

            // cout << "innn 1" << endl;

            continue;
        }

        if (robot->getTime() - startTime >= ESTIMATED_COMM_TIME)
        {
            // cout << "innn 2" << endl;

            handleReject();
            return false;
        }
        // get data
        string data = getNextMessage();
        // cout << robot->getName() << " received data: " << data << endl;

        if (getMessageType(data) == "PATH")
        {
            vector<string> splitted = split(data, '_');

            // get id
            int id = stoi(splitted[1]);
            m[id] = splitted[2];
            count++;
            // cout << "here" << endl;
            // cout << robot->getName() << "======================================================================================================= " << data << endl;
            // cout << "partner path innnn" << partnerPath.size() << " " << partnerPath[partnerPath.size() - 1][0] << " " << partnerPath[partnerPath.size() - 1][1] << endl;
        }
        else if (getMessageType(data) == "END")
        {
            // cout << "innn 3 " <<count<< endl;

            string pathString = "";
            for (int i = 1; i <= count; i++)
            {
                pathString += m[i];
            }
            // cout<<"pathString "<<pathString<<endl;
            partnerPath = getPathFromString(pathString);

            priority[partnerId] = robot->getTime();
            handleReject();

            return true;
        }
    }
}

string Communication::getNextMessage()
{
    if (receiver->getQueueLength())
    {
        char *dataPointer = (char *)receiver->getData();
        int size = receiver->getDataSize();

        string data;

        for (int i = 0; i < size / sizeof(char); i++)
        {
            data += *(dataPointer + i);
        }
        receiver->nextPacket();
        return data;
    }
    else
        return "";
}

bool Communication::isPartnerNear()
{
    if (isCommunicating)
        return false;

    if (receiver->getQueueLength() > MAX_QUEUE_SIZE)
        clearQueue();

    if (!receiver->getQueueLength())
    {
        return false;
    }
    if (receiver->getQueueLength() > 100)
        cout << robot->getName() << " " << receiver->getQueueLength() << " " << isCommunicating << endl;

    const string data = getNextMessage();
    // cout << robot->getName() << " received this message: " << data << endl;

    if (getMessageType(data) != "WAIT")
        return false;

    // cout << robot->getName() << " robot is checking if connection should be made or not" << endl;
    string partnerId = getPartnerId(data);
    string partnerMode = getPartnerMode(data);

    if (shouldCommunicate(partnerId, partnerMode))
    {
        // emitter->setRange(2);
        // cout << robot->getName() << " accepted the connection and sent accept message" << endl;
        // broadcastWaitMessage();
        // sendMessage("ACCEPT_" + robotId);
        this->partnerId = partnerId;
        this->partnerMode = partnerMode;
        // cout << robot->getName() << " set this as partnerId and Mode " << this->partnerId << " " << this->partnerMode << endl;
        return true;
    }
    else
    {
        // emitter->setRange(1);
        // cout << robot->getName() << " rejected the connection 1 " << partnerId << " " << partnerMode << endl;
        sendMessage("REJECT_" + robotId + "_" + partnerId);
        return false;
    }
}

int Communication::getQueueLength()
{
    return receiver->getQueueLength();
}
bool Communication::establishConnection()
{
    // clear queue
    clearQueue();

    // cout << robot->getName() << " establishing connection" << endl;
    // cout << robot->getName() << " partnerId " << partnerId << endl;

    commStartTime = robot->getTime();
    isCommunicating = true;

    int timestep = getRobotTimestep(robot);
    double lastAckTime = -1;
    double lastAcceptMessageSendTime = -1;
    bool isChannelTurned = false;
    while (robot->step(timestep) != -1)
    {
        if (receiver->getQueueLength() > MAX_QUEUE_SIZE)
            clearQueue();

        if (robot->getTime() - commStartTime > ESTIMATED_COMM_TIME)
        {
            // cout << robot->getName() << " says, I have done enough waiting, I am not going to wait more" << endl;
            // cout << robot->getName() << " xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << ++ccc << endl;
            handleReject();
            return false;
        }

        // cout<<robot->getName()<<" isChannelTurned "<<isChannelTurned<<endl;
        if (isChannelTurned)
        {
            if ((lastAckTime == -1 || robot->getTime() - lastAckTime >= 1))
            {
                lastAckTime = robot->getTime();
                sendMessage("ACK_" + robotId);
                // cout << robot->getName() << " sent ack message" << endl;
            }
        }
        else
        {
            if ((lastAcceptMessageSendTime == -1 || robot->getTime() - lastAcceptMessageSendTime >= 1))
            {
                lastAcceptMessageSendTime = robot->getTime();
                sendMessage("ACCEPT_" + robotId);
                // cout << robot->getName() << " sent accept message" << endl;
            }
            broadcastWaitMessage();
        }

        if (!receiver->getQueueLength())
            continue;

        const string data = getNextMessage();
        // cout << robot->getName() << " received data " << data << endl;

        cout << getMessageType(data) << endl;
        if (!isChannelTurned && getMessageType(data) == "ACCEPT")
        {

            string partnerId = getPartnerId(data);
            string receiverId = getReceiverId(data);
            // cout << robot->getName() << " received ACCEPT from  " << partnerId << " and for " << receiverId << endl;

            if (receiverId == this->robotId)
            {
                if (partnerId == this->partnerId)
                {
                    // turn on channel
                    int commChannel = getCommunicationChannel(robotId, partnerId);
                    clearQueue();

                    emitter->setChannel(commChannel);
                    receiver->setChannel(commChannel);
                    // cout << robot->getName() << " " << emitter->getChannel() << " " << receiver->getChannel() << " " << receiver->getQueueLength() << endl;
                    isChannelTurned = true;
                }
                else
                {
                    // cout << robot->getName() << " sent reject message 2 " << partnerId << " this->partnerId " << this->partnerId << endl;
                    sendMessage("REJECT_" + robotId);
                }
            }
        }
        else if (getMessageType(data) == "ACK")
        {
            string partnerId = getPartnerId(data);
            // cout << robot->getName() << "ACK received from =================================== " << partnerId << endl;
            if (partnerId == this->partnerId)
                return true;
        }
        else if (getMessageType(data) == "REJECT")
        {
            string partnerId = getPartnerId(data);
            string receiverId = getReceiverId(data);
            // cout << robot->getName() << " received REJECT from  " << partnerId << endl;

            if (receiverId == this->robotId)
            {
                if (partnerId == this->partnerId)
                {
                    handleReject();
                    return false;
                }
            }
        }
    }
}

bool Communication::establishConnectionV2()
{
    // clear queue
    // clearQueue();

    // cout << robot->getName() << " establishing connection" << endl;
    // cout << robot->getName() << " partnerId " << partnerId << endl;

    commStartTime = robot->getTime();
    isCommunicating = true;

    int timestep = getRobotTimestep(robot);
    int acceptCount = 0;
    bool isAcceptReceived = false;
    bool lastPathBroadcastTime = -1;
    double lastAckTime = -1;
    double lastAcceptMessageSendTime = -1;
    bool isChannelTurned = false;
    int ackCount = 0;
    while (robot->step(timestep) != -1)
    {
        if (receiver->getQueueLength() > MAX_QUEUE_SIZE)
            clearQueue();

        if (robot->getTime() - commStartTime > ESTIMATED_COMM_TIME)
        {
            cout << robot->getName() << " says, I have done enough waiting, I am not going to wait more" << endl;
            cout << robot->getName() << " xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << ++ccc << endl;
            handleReject();
            return false;
        }

        if (isChannelTurned)
        {
            cout << robot->getName() << " inside the room" << endl;

            if ((lastAckTime == -1 || robot->getTime() - lastAckTime >= 0.5))
            {
                lastAckTime = robot->getTime();
                sendMessage("ACK_" + robotId);
                cout << robot->getName() << " sent ack message" << emitter->getChannel() << " " << receiver->getChannel() << endl;
            }
        }
        else if (!isAcceptReceived)
        {
            cout << robot->getName() << " broadcast wait message" << emitter->getChannel() << " " << receiver->getChannel() << endl;

            broadcastWaitMessage();
        }

        if (acceptCount <= 8 && (lastAcceptMessageSendTime == -1 || robot->getTime() - lastAcceptMessageSendTime >= 0.5))
        {
            lastAcceptMessageSendTime = robot->getTime();
            sendMessage("ACCEPT_" + robotId + "_" + this->partnerId);
            cout << robot->getName() << " sent accept message " << emitter->getChannel() << " " << receiver->getChannel() << " " << acceptCount << endl;
            acceptCount++;
            if (acceptCount > 8)
            {
                if (!isAcceptReceived)
                {
                    cout << robot->getName() << " not going to wait because of accept message didn't receive " << endl;

                    handleReject();
                    return false;
                }
                isChannelTurned = true;
                int commChannel = getCommunicationChannel(robotId, partnerId);

                emitter->setChannel(commChannel);
                receiver->setChannel(commChannel);
                cout << robot->getName() << " channel is turned on " << acceptCount << endl;
                clearQueue();

                cout << robot->getName() << " " << emitter->getChannel() << " " << receiver->getChannel() << " " << receiver->getQueueLength() << endl;
            }
        }

        if (!receiver->getQueueLength())
            continue;

        const string data = getNextMessage();
        cout << robot->getName() << " received data " << data << emitter->getChannel() << " " << receiver->getChannel() << endl;

        cout << getMessageType(data) << endl;
        if (!isAcceptReceived && getMessageType(data) == "ACCEPT")
        {
            string partnerId = getPartnerId(data);
            string receiverId = getReceiverId(data);

            if (receiverId == this->robotId)
            {
                cout << robot->getName() << " received ACCEPT from  " << partnerId << " and for " << receiverId << " " << emitter->getChannel() << " " << receiver->getChannel() << endl;

                if (partnerId == this->partnerId)
                {

                    isAcceptReceived = true;
                }
                else
                {
                    cout << robot->getName() << " sent reject message to " << partnerId << emitter->getChannel() << " " << receiver->getChannel() << endl;
                    sendMessage("REJECT_" + robotId + "_" + partnerId);
                }
            }
        }
        else if (getMessageType(data) == "ACK")
        {
            string partnerId = getPartnerId(data);
            cout << robot->getName() << "ACK received from =================================== " << partnerId << emitter->getChannel() << " " << receiver->getChannel() << endl;
            if (partnerId == this->partnerId)
                return true;
        }
        else if (getMessageType(data) == "REJECT")
        {
            string partnerId = getPartnerId(data);
            string receiverId = getReceiverId(data);

            if (receiverId == this->robotId)
            {
                cout << robot->getName() << " received REJECT from  " << partnerId << " and for " << receiverId << " " << emitter->getChannel() << " " << receiver->getChannel() << endl;

                if (partnerId == this->partnerId)
                {
                    handleReject();
                    return false;
                }
            }
        }
    }
}

bool Communication::establishConnectionV3()
{
    bool isMajor = stoi(robotId) > stoi(partnerId);
    // cout << robot->getName() << " is major: " << isMajor << endl;
    int timestep = getRobotTimestep(robot);

    commStartTime = robot->getTime();
    isCommunicating = true;

    if (isMajor)
    {
        int ACCEPT_MESSAGE_LIMIT = 8;
        int acceptSendCount = 0;
        double lastAcceptMessageSendTime = -1;

        int ACK_MESSAGE_LIMIT = 8;
        int ackSendCount = 0;
        double lastAckSendCount = -1;

        while (robot->step(timestep) != -1)
        {
            if (receiver->getQueueLength() > MAX_QUEUE_SIZE)
                clearQueue();

            if (robot->getTime() - commStartTime > ESTIMATED_COMM_TIME)
            {
                cout << robot->getName() << " says, I have done enough waiting, I am not going to wait more" << endl;
                // cout << robot->getName() << " xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << ++ccc << endl;
                handleReject();
                return false;
            }

            // broadcast wait message
            broadcastWaitMessage();

            if (acceptSendCount > ACCEPT_MESSAGE_LIMIT)
            {
                handleReject();
                return false;
            }
            else
            {
                // send accept message at every half second for 8 times
                if (lastAcceptMessageSendTime == -1 || robot->getTime() - lastAcceptMessageSendTime >= 0.5)
                {
                    sendMessage("ACCEPT_" + robotId + "_" + this->partnerId);
                    acceptSendCount++;
                    lastAcceptMessageSendTime = robot->getTime();
                }
            }

            // receiver message
            if (!receiver->getQueueLength())
                continue;

            const string data = getNextMessage();

            if (getMessageType(data) == "ACCEPTACK" && isMessageForMe(data))
            {
                // go to chat room
                int commChannel = getCommunicationChannel(robotId, partnerId);

                emitter->setChannel(commChannel);
                receiver->setChannel(commChannel);
                clearQueue();
                // cout << robot->getName() << " is in room" << endl;
                break;
            }
            else if (getMessageType(data) == "ACCEPT" && isMessageForMe(data))
            {
                const string senderId = getPartnerId(data);
                sendMessage("REJECT_" + robotId + "_" + senderId);
            }
            else if (getMessageType(data) == "REJECT" && isMessageForMe(data))
            {

                handleReject();
                return false;
            }
        }

        // room part
        while (robot->step(timestep) != -1)
        {
            if (receiver->getQueueLength() > MAX_QUEUE_SIZE)
                clearQueue();

            if (robot->getTime() - commStartTime > ESTIMATED_COMM_TIME)
            {
                cout << robot->getName() << " says, I have done enough waiting, I am not going to wait more" << endl;
                // cout << robot->getName() << " xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << ++ccc << endl;
                handleReject();
                return false;
            }

            // send ack message
            if (ackSendCount > ACK_MESSAGE_LIMIT)
            {
                handleReject();
                return false;
            }
            else
            {
                // send accept message at every half second for 8 times
                if (lastAckSendCount == -1 || robot->getTime() - lastAckSendCount >= 0.5)
                {
                    sendMessage("ACK_" + robotId + "_" + this->partnerId);
                    ackSendCount++;
                    lastAckSendCount = robot->getTime();
                }
            }

            // receiver message
            if (!receiver->getQueueLength())
                continue;

            const string data = getNextMessage();

            if (getMessageType(data) == "ACK" && isMessageForMe(data))
            {
                // cout << robot->getName() << " Now, I can exchange the path" << endl;
                return true;
            }
        }
    }
    else
    {
        while (robot->step(timestep) != -1)
        {
            if (receiver->getQueueLength() > MAX_QUEUE_SIZE)
                clearQueue();

            if (robot->getTime() - commStartTime > ESTIMATED_COMM_TIME)
            {
                cout << robot->getName() << " says, I have done enough waiting, I am not going to wait more" << endl;
                // cout << robot->getName() << " xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << ++ccc << endl;
                handleReject();
                return false;
            }

            // broadcast wait message
            broadcastWaitMessage();

            // receive message
            if (!receiver->getQueueLength())
                continue;

            const string data = getNextMessage();

            if (getMessageType(data) == "ACCEPT" && isMessageForMe(data))
            {
                sendMessage("ACCEPTACK_" + robotId + "_" + partnerId);

                // go to chat room
                int commChannel = getCommunicationChannel(robotId, partnerId);

                emitter->setChannel(commChannel);
                receiver->setChannel(commChannel);
                clearQueue();
                // cout << robot->getName() << " is in room" << endl;

                break;
            }
        }
        while (robot->step(timestep) != -1)
        {
            if (receiver->getQueueLength() > MAX_QUEUE_SIZE)
                clearQueue();

            if (robot->getTime() - commStartTime > ESTIMATED_COMM_TIME)
            {
                cout << robot->getName() << " says, I have done enough waiting, I am not going to wait more" << endl;
                // cout << robot->getName() << " xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << ++ccc << endl;
                handleReject();
                return false;
            }

            // receiver message
            if (!receiver->getQueueLength())
                continue;

            const string data = getNextMessage();

            if (getMessageType(data) == "ACK" && isMessageForMe(data))
            {
                sendMessage("ACK_" + robotId + "_" + partnerId);
                // cout << robot->getName() << " Now, I can exchange the path" << endl;

                return true;
            }
        }
    }
}

string Communication::getPartnerId(){
    return partnerId;
}
bool Communication::isMessageForMe(const string &message)
{
    const string receiverId = getReceiverId(message);
    const string senderId = getPartnerId(message);

    return senderId == this->partnerId && receiverId == this->robotId;
}

string Communication::getPartnerMode(){
    return partnerMode;
}
int Communication::getCommunicationChannel(const string &robotId, const string &partnerId)
{

    if (robotId.compare(partnerId) < 0)
        return stoi(robotId + partnerId);
    else
        return stoi(partnerId + robotId);
}

void Communication::handleReject()
{
    // cout << robot->getName() << " handling rejection" << endl;

    isCommunicating = false;
    partnerId = "";
    partnerMode = "";
    emitter->setChannel(0);
    receiver->setChannel(0);
    // cout << robot->getName() << " stats: ===================" << endl;
    // cout << robot->getName() << "emitter channel: " << emitter->getChannel() << endl;
    // cout << robot->getName() << "receiver channel: " << receiver->getChannel() << endl;
    // cout << robot->getName() << "partner mode: " << this->partnerMode << endl;
    // cout << robot->getName() << "partner mode: " << this->partnerId << endl;
    // cout << robot->getName() << "isCommunicating: " << this->isCommunicating << endl;
    // cout << robot->getName() << "===================" << endl;
}

void Communication::clearQueue()
{
    // cout << robot->getName() << " clearing the queue =s=s==s=s=s=s=s=s=s=s=s=s=s=s=-s=s=s=s=s-=sdf-=sd-f=ds-f=sd-f=dsf-=sd-f=ds-f=sdf-=" << endl;
    while (receiver->getQueueLength())
        receiver->nextPacket();
}
