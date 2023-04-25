#ifndef FILE_MANAGER_HEADER
#define FILE_MANAGER_HEADER

#include <cmath>
#include <cstring>
#include <webots/Robot.hpp>
#include <webots/Compass.hpp>
#include <webots/gps.hpp>

#include "defs.h"
#include "robotsUtils.h"
#include "cartesian.h"

using namespace webots;
using namespace std;

class FileManager
{
private:
    ofstream csvFile;
    string path;
    string fileName;

public:
    FileManager(const string &path, const string &fileName);
    void writeFile(const string &data);
    void closeFile();
};

#endif