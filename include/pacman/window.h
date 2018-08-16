#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include <QObject>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QPushButton>
#include <QDesktopWidget>
#include <QApplication>
#include <QMessageBox>
#include <QTimer>
#include <QTime>
#include <QList>  //GED
#include <QLCDNumber>
#include <QSound>
#include <iostream>
#include "ros/package.h"
#include "ros/ros.h"
#include "maps.h"
#include "pacman/glwidget.h"
#include "pacman/listenmsgthread.h"
#include "pacman/pacmanPos.h"
#include "pacman/ghostsPos.h"
#include "pacman/cookiesPos.h"
#include "pacman/bonusPos.h"
#include "pacman/game.h"
#include "pacman/pos.h"
#include "pacman/mapService.h"

using namespace std;

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
    QString verifyMapArgument(QStringList args, QComboBox *mapsList, int pacmanMode);	//GED Jul-30
private:
    void ListArrayMap(QString path);
    bool ObsService(pacman::mapService::Request& req, pacman::mapService::Response &res);
   
protected:
    void keyPressEvent(QKeyEvent *event) override;

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
    void EndGame();	//GED Ag-01
    void InitializeCounterTimerSlot();	//Jul-27
    void ReceiveMaxValues(int maxScore, int maxLives);
    void InitializeGameSlot();
    void restartReadySlot();
    
private:
    GLWidget *glWidget;
    const int maxWidth = 1000;
    const int maxHeight = 700;
    const int scoreWidth = 750;
    const int scoreHeight = 60;
    const double wScore = 0.3;
    const double wLives = 0.6;
    const double wTime = 0.1;
    int MAX_SCORE, MAX_LIVES, MAX_TIME_SEC;
    double performEval;
    QVBoxLayout *mainLayout;
    QPushButton *playBtn;
    QPushButton *counterBtn;	//GED Jul-27
    QPushButton *EndGameBtn1;	//GED Ag-01
    QPushButton *EndGameBtn2;	//GED Ag-01
    MainWindow *mainWindow;
    Maps *maps;
    QString mapName;
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
    QLCDNumber *gameTimeRemainingLCD;
    QLabel *performValName;
    QLabel *performValLabel;
    QTimer *remainingTimeTimer;
    QTime *gameTime;
    const int initialGameTimeMins = 3;
    const int initialGameTimeSecs = 0;
    const int oneSecondTimeMilisecs = 1000;
    QSound *initSound;
    QTimer *restartGameTimer;
    const int restartGameTime = 5000;
    
signals:
    void ArrowKey(int key);
    void InitializeGame();
};

#endif
