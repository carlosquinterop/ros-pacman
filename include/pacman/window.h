#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include <QObject>
#include "maps.h"
#include <iostream>
#include <ros/package.h>
#include "pacman/glwidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QPushButton>
#include <QDesktopWidget>
#include <QApplication>
#include <QMessageBox>
#include <QTimer>
#include <QList>  //GED
#include "pacman/listenmsgthread.h"
#include "ros/ros.h"
#include "pacman/pacmanPos.h"
#include "pacman/ghostsPos.h"
#include "pacman/cookiesPos.h"
#include "pacman/bonusPos.h"
#include "pacman/game.h"
#include "pacman/pos.h"
#include "pacman/mapService.h"


using namespace std;

class QSlider;
class QPushButton;

class GLWidget;
class MainWindow;
class Window : public QWidget
{
    Q_OBJECT

public:
    Window(QStringList args);		//GED Jul-27: Se recibe QStringList args con  argumentos
    int getArguments(QStringList args);	//GED Jul-28
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;
    
private:
    void ListArrayMap(QString path);
    bool obsService(pacman::mapService::Request& req, pacman::mapService::Response &res);

protected:
    void keyPressEvent(QKeyEvent *event) override;

public slots: 			//GED Jul-27
    void timerFunction();	//Jul-27

private slots:
    void PlaySlot();
    void UpdatePacmanPosSlot(QVector<QPoint>* pos);
    void UpdateGhostsPosSlot(QVector<QPoint>* pos, bool* ghostsMode);
    void UpdateCookiesPosSlot(QVector<QPoint>* pos);
    void UpdateBonusPosSlot(QVector<QPoint>* pos);
    void UpdateObstaclesPosSlot(QVector<QPoint>* pos, int xMin, int xMax, int yMin, int yMax);
    void UpdateSizeSlot();
    void DeadPacmanSlot();
    void EndOfDeadPacmanSlot();
    void UpdateGameStateSlot();
    void UpdateScoresSlot(int score, int lives);
    
private:
    GLWidget *glWidget;
    const int maxWidth = 1000;
    const int maxHeight = 700;
    QVBoxLayout *mainLayout;
    QPushButton *playBtn;
    QPushButton *counterBtn;	//GED Jul-27
    MainWindow *mainWindow;
    Maps *maps;
    int mode;
    bool gameState = false;
    QComboBox *mapsList;
    QHBoxLayout *container;
    bool allowPlay;
    QTimer *refreshTimer;
    QTimer *counterTimer; 	//GED Jul-27
    const int refreshTimeMs = 150;
    ListenMsgThread *listenMsg;
    ros::NodeHandle *node;
    ros::Subscriber subscriber;
    ros::Publisher pacmanPublisher;
    ros::Publisher ghostPublisher;
    ros::Publisher cookiesPublisher;
    ros::Publisher bonusPublisher;
    ros::Publisher gameStatePublisher;
    ros::ServiceServer mapResponseServer;
    pacman::pacmanPos msgPacman;
    pacman::ghostsPos msgGhosts;
    pacman::cookiesPos msgCookies;
    pacman::bonusPos msgBonus;
    pacman::game msgState;
    QVector<QPoint> *posObstacles;
    int minX, maxX, minY, maxY;
    QHBoxLayout *containerScores;
    QLabel *scoreName;
    QLabel *scoreLabel;
    QLabel *livesName;
    QLabel *livesLabel;
    
signals:
    void ArrowKey(int key);
    
};

#endif
