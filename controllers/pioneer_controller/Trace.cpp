#include "Trace.h"

Trace::Trace(int w, int h)
{

    width = w;
    height = h;
    gridImage = (int*)malloc(height*width*sizeof(int));
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
            gridImage[y * width + x] = 0x00000000;
}

Trace::~Trace(){
    free(gridImage);
}

void Trace::appendBoundaryPoint(int x, int y)
{
    actualPoints.push_back({x, y});
    cout<<"Here"<<endl;
    if (actualPoints.size() == 1)
        return;

    vector<int> prev = actualPoints[actualPoints.size() - 2];
    vector<int> current = actualPoints[actualPoints.size() - 1];
    for (int i = 0; i < 100; i++)
    {
        x = (int)((current[0] - prev[0]) * i / 100.0 + prev[0]);
        y = (int)((current[1] - prev[1]) * i / 100.0 + prev[1]);
        gridImage[(height / 2 - 1 - y) * width + x] = 0xFFFFFFFF;
    }
}

void Trace::appendEstimatePoint(int x, int y)
{
    estimatePoints.push_back({x, y});
    if (estimatePoints.size() == 1)
        return;

    vector<int> prev = estimatePoints[estimatePoints.size() - 2];
    vector<int> current = estimatePoints[estimatePoints.size() - 1];
    for (int i = 0; i < 100; i++)
    {
        x = (int)((current[0] - prev[0]) * i / 100.0 + prev[0]);
        y = (int)((current[1] - prev[1]) * i / 100.0 + prev[1]);
        if ((x >= 0) && (x < width) && (y >= 0) && (y < height / 2))
            gridImage[(height / 2 - 1 - y) * width + x] = 0xFFFF0000;
    }
}

void Trace::draw(Display *aPen)
{
    ImageRef *imageOfGrid1 = aPen->imageNew(width, height, gridImage, Display::ARGB);
    aPen->imagePaste(imageOfGrid1, 0, 0, false);
}