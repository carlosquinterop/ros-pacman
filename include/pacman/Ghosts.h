#ifndef GHOSTS_H
#define GHOSTS_H

#include <iostream>
#include <QObject>
#include <QPoint>
#include <math.h>

using namespace std;

class Ghosts: public QObject
{
    Q_OBJECT

public:
  enum class Mode {Chase, Scatter, Frightened};
    enum class Action {Up, Down, Right, Left, None};
    enum class Personality {Shadow, Speedy, Bashful, Pokey}; 
    Ghosts(QPoint initialPosition, Ghosts::Personality character, int aHeight, int aWidth);
    int getTexId();
    
    QPoint currentPosition;
    double orientation;
    Mode mode;
    QPoint targetPosition;
    Action action;
    Action previousAction;
    QString name;
    Personality character;
    QString upFileName, downFileName, rightFileName, leftFileName;
    int height;
    int width;
    int texId;
};

#endif