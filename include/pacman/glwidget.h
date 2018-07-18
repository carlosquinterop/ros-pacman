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
    
private slots:
    void receiveKeySlot(int key);
    void receiveMapDataGL(int wPacman, int hPacman, QImage* mapImage, bool *mObstacles, int rowPacman, int colPacman);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;
   
private:
    double ortho[4];
    QVector<GLuint> texIds;
    QImage *_mapImage, *pacmanImage;
    int mapWidth, mapHeight, pacmanHeight, pacmanWidth;
    bool *obstacles, firstTime;
    QPoint pacmanCoord;
    double w;
};

#endif
