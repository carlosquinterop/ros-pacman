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
    int getIndexRowFromCoord(QPoint coord);
    int getIndexColFromCoord(QPoint coord);
    void loadNewTexture (QImage* img);
    void drawCircle(float x, float y, float radius, float red, float green, float blue);
    void drawCookies();
    void drawBonus();
    void setCoordCookies(QVector<int> *pCookies);
    void setCoordBonus(QVector<int> *pBonus);
    
    enum class Action {Up, Down, Right, Left, None};
    
        
private slots:
    void updateSimulationSlot();
    void receiveMapDataGL(int wPacman, int hPacman, QImage* mapImage, int *mObstacles, int rowPacman, int colPacman, QVector<int> *pGhosts, QVector<int> *pCookies, QVector<int> *pBonus);
    void setPacmanCommand(int aPacmanCommand);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;
   
private:
    double ortho[4];
    QVector<GLuint> texIds;
    QImage *_mapImage, *pacmanImage;
    int mapWidth, mapHeight, pacmanHeight, pacmanWidth;
    int *obstacles, firstTime;
    QPoint pacmanCoord;
    QPoint *cookiesCoord;
    QPoint *bonusCoord;
    double w;
    int sCookies;
    int sBonus;
    Action pacmanCommand;

    signals:
    void UpdatePacmanPos(QPoint coordPacman);
};

#endif
