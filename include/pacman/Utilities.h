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
    QPoint* GetCoordFromIndex(int blockWidth, int blockHeight, double ortho[4], int iRow, int iCol);
    int ComputeDistanceBetweenPoints(QPoint a, QPoint b);
};

#endif
