#ifndef GHOSTS_H
#define GHOSTS_H

#include <iostream>
#include <QObject>
#include <QPoint>

using namespace std;

class Ghosts: public QObject
{
    Q_OBJECT

public:
    Ghosts();
    enum class Mode {Chase, Scatter, Frightened};
    enum class Action {Up, Down, Right, Left, None};
    
private:
    QPoint currentPosition;
    double orientation;
    Mode mode;
    QPoint targetPosition;
    Action action;
    Action previousAction;
    QString name;
    QString personality;
    QString fileName;
    int height;
    int width;
};

#endif