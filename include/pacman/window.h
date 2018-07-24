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

private:
	void listArrayMap(QString path);

protected:
    void keyPressEvent(QKeyEvent *event) override;

private slots:
    void playSlot();
    void updatePacmanPosSlot(QPoint pos);
    
private:
    GLWidget *glWidget;
    QVBoxLayout *mainLayout;
    QPushButton *playBtn;
    MainWindow *mainWindow;
    Maps *maps;
    QComboBox *mapsList;
    QHBoxLayout *container;
    bool allowPlay;
    QTimer *refreshTimer;
    const int refreshTimeMs = 200;
    ListenMsgThread *listenMsg;
    ros::NodeHandle *node;
    ros::Subscriber subscriber;
    ros::Publisher publisher; 
    pacman::pacmanPos msg;
    
signals:
    void arrowKey(int key);
};

#endif
