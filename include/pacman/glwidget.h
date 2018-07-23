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

using namespace std;

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent = 0);
    ~GLWidget();
    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;
    void loadTexture (QImage* img);
    void loadNewTexture (QImage* img);
    void drawMap();
    void drawPacman();
    void drawGhosts();
    void drawCookies();
    void drawBonus();
    void drawCircle(float x, float y, float radius, float red, float green, float blue);
    int getIndexRowFromCoord(QPoint coord);
    int getIndexColFromCoord(QPoint coord);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;
    
private slots:
    void updateSimulationSlot();
    void receiveMapDataGL(int blockWidht, int blockHeight, QImage* mapImage, int *mObstacles, QVector<int> *pPacman, QVector<int> *pGhosts, QVector<int> *pCookies, QVector<int> *pBonus);
    void setPacmanCommand(int aPacmanCommand);
   
private:
    QImage *_mapImage;  
    double ortho[4];
    QVector<GLuint> texIds;   
    int mapWidth, mapHeight;
    int *obstacles, firstTime;
    
    QImage *pacmanImage;    
    int nPacman;
    Pacman **pacmanArray;

    QPoint *cookiesCoord;
    QPoint *bonusCoord;
    int sCookies;
    int sBonus;

    Ghosts **ghostsArray;
    int nGhosts;
    QPoint **ghostsCoord;

signals:
    void UpdatePacmanPos(QPoint coordPacman);
};

#endif
