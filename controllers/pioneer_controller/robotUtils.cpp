#include "robotsUtils.h"

int getRobotTimestep(Robot *robot)
{
  return (int)robot->getBasicTimeStep();
}

double fModulo(double n, double m)
{
  while (n >= m)
    n -= m;
  return n;
}

double uniform(double min, double max)
{
  random_device rd;
  mt19937 gen(rd());
  uniform_real_distribution<> dist(min, max);

  return dist(gen);
}

void removeVectorElements(vector<vector<double>> &v, llu sIndex, llu eIndex)
{
  auto it = v.begin() + sIndex;
  while (sIndex < eIndex)
  {
    v.erase(it);
    sIndex++;
  }
}

vector<string> split(const string &s, const char seperator)
{
  vector<string> ans;

  string t = "";
  for (char c : s)
  {
    if (c == seperator)
    {
      if (t.length())
        ans.push_back(t);
      t = "";
    }
    else
    {
      t += c;
    }
  }
  if (t.length())
    ans.push_back(t);

  return ans;
}

string getRobotId(const string &robotName)
{
  vector<string> splittedName = split(robotName, '_');
  return splittedName[1];
}

// void Communication::handleCommunication()
// {
//     // broadcast wait message
//     broadcastWaitMessage();

//     if (isPartnerNear())
//     {
//         // get robot's current state
//         vector<double> robotStage = robot->getMotor

//         // stop robot
//         motorController->stopMotor();
//         if (comm->establishConnection())
//         {
//             cout << "connection establish successful" << endl;
//             vector<vector<double>> p = {{1, 2}, {10, 20}};
//             if (robotController->getMode() != "explore")
//                 comm->broadcastPath(p);
//             vector<vector<double>> receivedPath;

//             if (comm->receivePath(receivedPath))
//             {
//                 for (vector<double> c : receivedPath)
//                 {
//                     cout << c[0] << " " << c[1] << endl;
//                 }
//             }
//         }
//         else
//         {
//             cout << "connection establish not successful";
//         }
//     }
// }

void middleware(){

  // create the communication object
  // globalCommunication->broadcastWaitMessage();
}