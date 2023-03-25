#ifndef TRACKER_APP_HEADER
#define TRACKER_APP_HEADER

#include "defs.h"
#include "Trace.h"

using namespace webots;
using namespace std;

class TrackerApp
{
private:
    static int const windowWidth = 400;
    static int const windowHeight = 400;

    Display *display;
    Trace *trace;

public:
    TrackerApp(Display *d);
    void addActualLocation(int x, int y);
    void addEstimatedLocation(int x, int y);
};

#endif