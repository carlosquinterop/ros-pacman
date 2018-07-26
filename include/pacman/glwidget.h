#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLTexture>
#include <QOpenGLWidget>
#include <GL/glu.h>
#include <QOpenGLFunctions>
#include <QCoreApplication>
#include <math.h>
#include <iostream>
#include <QPoint>
#include <QImage>
#include "pacman/Ghosts.h"
#include "pacman/Pacman.h"
#include <QTimer>

#include "Utilities.h"

using namespace std;

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent = 0);
    ~GLWidget();
    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;
    void LoadTexture (QImage* img);
    void LoadNewTexture (QImage* img);
    void DrawMap();
    void DrawPacman();
    void DrawGhosts();
    void DrawCookies();
    void DrawBonus();
    void DrawCircle(float x, float y, float radius, float red, float green, float blue);
    void TogglePlaying();
    void UpdatePacmanPosition(int i);
    
protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;
    
private slots:
    void UpdateSimulationSlot();
    void ReceiveMapDataGL(int blockWidht, int blockHeight, QImage* mapImage, int *mObstacles, QVector<int> *pPacman, QVector<int> *pGhosts, QVector<int> *pCookies, QVector<int> *pBonus);
    void SetPacmanCommand(int aPacmanCommand);
    void ToggleGhostModeSlot();
    void EndOfFrightenedGhostModeSlot();
    
private:
    bool allowToPlay;
    QImage *_mapImage;  
    double ortho[4];
    QVector<GLuint> texIds;   
    int mapWidth, mapHeight;
    int *obstacles, firstTime;
    int nPacman;
    Pacman **pacmanArray;
    int nGhosts;
    Ghosts **ghostsArray;
    QTimer *ghostModeTimer;
    const int ghostModeTimes[8] = {3000, 7000, 20000, 7000, 20000, 5000, 20000, 5000};
    int ghostRemainingTime;
    int contGhostModePhases;
    QTimer *frightenedGhostModeTimer;
    const int frightenedModeTimeMs = 10000;
    bool isInFrightenedMode;
    int sCookies;
    int sBonus;
    QPoint *pacmanCoord;
    QPoint *cookiesCoord;
    QPoint *bonusCoord;
    QPoint *ghostsCoord;
    Utilities utilities;

signals:
    void UpdatePacmanPos(QPoint *pacmanCoord, int nPacman);
    void UpdateGhostsPos(QPoint *ghostsCoord, int nGhosts);
    void UpdateCookiesPos(QPoint *cookiesCoord, int sCookies);
    void UpdateBonusPos(QPoint *bonusCoord, int sBonus);
};

#endif
