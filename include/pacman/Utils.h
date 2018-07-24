#ifndef UTILITIES_H
#define UTILITIES_H

#include <QCoreApplication>
#include <math.h>
#include <QPoint>

class Utilities :
{

public:
    Utilities();
    int getIndexRowFromCoord(QPoint coord, int mapHeight);
    int getIndexColFromCoord(QPoint coord, int mapWidth);
};

#endif
