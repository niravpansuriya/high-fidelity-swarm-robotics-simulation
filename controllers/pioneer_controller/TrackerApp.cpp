#include "TrackerApp.h"

TrackerApp::TrackerApp(Display *d)
{
    display = d;
    trace = new Trace(windowWidth, windowHeight);
}

void TrackerApp::addActualLocation(int x, int y)
{
    cout<<x<<" "<<y<<endl;
    // cout<<(int)(x + 180) / 360 * 60 / 0.5<<" "<< (90 - y) / 180 * 60 / 0.5<<endl;
    // display->drawPixel((int)(x + 180) / 360 * 60 / 0.5, (90 - y) / 180 * 60 / 0.5);
    display->drawPixel(x, y);
    return;
    // for(int i=0;i<100;i++){
    //     for(int j=0;j<100;j++){
    //         // cout<<i<<" "<<j<<endl;
    //         display->drawPixel(i,j);
    //     }
    // }
    // return;
    // trace->appendBoundaryPoint(x + windowWidth / 4, y + 15 + windowHeight / 4);
    trace->appendBoundaryPoint(x, y);
    // cout<<"4"<<endl;
    // trace->draw(display);
}

void TrackerApp::addEstimatedLocation(int x, int y)
{
    // trace->appendEstimatePoint(x + windowWidth / 4, y + 15 + windowHeight / 4);
    trace->appendBoundaryPoint(x, y);
    trace->draw(display);
}