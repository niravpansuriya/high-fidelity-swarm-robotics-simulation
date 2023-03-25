#ifndef TRACE_HEADER
#define TRACE_HEADER

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Display.hpp>
#include <cstring>
#include <vector>
#include "defs.h"

using namespace webots;
using namespace std;

class Trace
{
private:
    int width;
    int height;
    vector<vector<int>> actualPoints;
    vector<vector<int>> estimatePoints;
    int* gridImage;

public:
    Trace(int w, int h);
    ~Trace();
    void appendBoundaryPoint(int x, int y);
    void appendEstimatePoint(int x, int y);
    void draw(Display *aPen);
};

#endif