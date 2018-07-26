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
#include "pacman/listenmsgthread.h"
#include "ros/ros.h"
#include "pacman/pacmanPos.h"
#include "pacman/ghostsPos.h"
#include "pacman/cookiesPos.h"
#include "pacman/bonusPos.h"
#include "pacman/pos.h"


using namespace std;

class QSlider;
class QPushButton;

class GLWidget;
class MainWindow;

class Window : public QWidget
{
    Q_OBJECT

public:
    Window();
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

private:
    void ListArrayMap(QString path);

protected:
    void keyPressEvent(QKeyEvent *event) override;

private slots:
    void PlaySlot();
    void UpdatePacmanPosSlot(QPoint* pos, int nPacman);
    void UpdateGhostsPosSlot(QPoint* pos, int nGhosts);
    void UpdateCookiesPosSlot(QPoint* pos, int nCookies);
    void UpdateBonusPosSlot(QPoint* pos, int nBonus);
    void UpdateSizeSlot();
    
private:
    GLWidget *glWidget;
    const int maxWidth = 1000;
    const int maxHeight = 700;
    QVBoxLayout *mainLayout;
    QPushButton *playBtn;
    MainWindow *mainWindow;
    Maps *maps;
    QComboBox *mapsList;
    QHBoxLayout *container;
    bool allowPlay;
    QTimer *refreshTimer;
    const int refreshTimeMs = 150;
    ListenMsgThread *listenMsg;
    ros::NodeHandle *node;
    ros::Subscriber subscriber;
    ros::Publisher pacmanPublisher;
    ros::Publisher ghostPublisher;
    ros::Publisher cookiesPublisher;
    ros::Publisher bonusPublisher;
    pacman::pacmanPos msgPacman;
    pacman::ghostsPos msgGhosts;
    pacman::cookiesPos msgCookies;
    pacman::bonusPos msgBonus;
    
signals:
    void ArrowKey(int key);
};

#endif
