#ifndef UTILITIES_H
#define UTILITIES_H

#include <QCoreApplication>
#include <math.h>
#include <QPoint>

class Utilities
{

public:
    Utilities();
    void SetMapData(int blockWidth, int blockHeight, double ortho[4]);
    int GetIndexRowFromCoord(QPoint coord, int mapHeight);
    int GetIndexColFromCoord(QPoint coord, int mapWidth);
    QPoint* GetCoordFromIndex(int iRow, int iCol);
    int ComputeDistanceBetweenPoints(QPoint a, QPoint b);
    
private:
    int _blockWidth;
    int _blockHeight;
    double _ortho[4];
};

#endif
