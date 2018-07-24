#ifndef GHOSTS_H
#define GHOSTS_H

#include <iostream>
#include <QObject>
#include <QPoint>
#include <math.h>
#include "Utilities.h"
#include <QVector>

using namespace std;

class Ghosts: public QObject
{
    Q_OBJECT

public:
    enum class Mode {Chase, Scatter, Frightened};
    enum class Action {Up, Down, Right, Left, None};
    enum class Personality {Shadow, Speedy, Bashful, Pokey}; 
    Ghosts(QPoint initialPosition, Ghosts::Personality character, int aHeight, int aWidth, QPoint initialPacmanPosition, int cMapHeight, int cMapWidth, int *cObstacles);
    int GetTexId();
    void UpdateGhostPosition();
    void CalculateTargetPosition();
    int GetPossibleActions(QVector<Ghosts::Action> *possibleActions);
    void PrintPossibleActions(QVector<Ghosts::Action>* possibleActions);
    bool IsPossibleAction(Ghosts::Action anAction, QVector<Ghosts::Action>* possibleActions);

    QPoint currentPosition;
    double orientation;
    Mode mode;
    QPoint targetPosition;
    Action action;
    QString name;
    Personality character;
    int height;
    int width;
    int texId;
    Utilities utilities;
    
    QPoint pacmanPosition;
    int mapWidth, mapHeight;
    int *obstacles;
    
};

#endif