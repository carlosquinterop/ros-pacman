#ifndef MAPS_H
#define MAPS_H

#include <iostream>

#include <QWidget>
#include <QObject>
#include <QImage>
#include <QFile>
#include <QLabel>
#include <QPixmap>
#include <QRgb>
#include <QComboBox>
#include <QVBoxLayout>
#include <QDir>
#include <QDirIterator>
#include <ros/package.h>

using namespace std;

class Maps : public QWidget
{
    Q_OBJECT

public:
    Maps(QWidget *parent = 0);
    
private:
    QByteArray file2ArrayMap(QString fileName, int &colsText, int &rowsText);
    void createObstaclesArray(QByteArray text, int colsText, int rowsText);
    void printObstaclesArray();
    void createImageFromObstaclesArray();
    void saveImage(QString fileName);
   
private:
    int rows, cols;
    int BLOCK_SIZE = 50;
    int *mObstacles;
    
    QVector<int> *pPacman;
    QVector<int> *pGhosts;
    QVector<int> *pCookies;
    QVector<int> *pBonus;
    QImage* image;
    
public slots:
    void createMap(QString nameMap);
    
signals:
    //void sendMapData(int wPacman, int hPacman, QImage *mapImage, int *mObstacles, int rowPacman, int colPacman);
    void sendMapData(int blockWidth, int blockHeight, QImage *mapImage, int *mObstacles, QVector<int> *pPacman, QVector<int> *pGhosts, QVector<int> *pCookies, QVector<int> *pBonus);
};

#endif
