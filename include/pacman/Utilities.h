#ifndef UTILITIES_H
#define UTILITIES_H

#include <QCoreApplication>
#include <math.h>
#include <QPoint>

class Utilities
{

public:
    Utilities();
    int GetIndexRowFromCoord(QPoint coord, int mapHeight);
    int GetIndexColFromCoord(QPoint coord, int mapWidth);
    int ComputeDistanceBetweenPoints(QPoint a, QPoint b);
};

#endif
