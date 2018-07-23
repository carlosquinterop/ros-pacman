#ifndef PACMAN_H
#define PACMAN_H

#include <iostream>
#include <QObject>
#include <QPoint>
#include <math.h>

using namespace std;

class Pacman: public QObject
{
    Q_OBJECT

public:
    enum class Action {Up, Down, Right, Left, None};
    Pacman(QPoint initialPosition, double initialOrientation, int aHeight, int aWidth);
        
    QPoint currentPosition;
    double orientation;
    Action action;
    QString fileName;
    int height;
    int width;
};

#endif