#ifndef LISTENMSGTHREAD_H
#define LISTENMSGTHREAD_H

#include <QObject>
#include <QThread>
#include <iostream>
#include "ros/ros.h"
#include "pacman/Num.h"
#include "pacman/glwidget.h"

using namespace std;

class ListenMsgThread : public QThread
{
    Q_OBJECT
    
    void run();
public:
    ListenMsgThread();
    void setWorkingThread();
    void callback(const pacman::Num::ConstPtr& msg);
        
private:
    bool work;
    ros::NodeHandle *node;
    ros::Subscriber sub;
    
signals:
    void UpdatePacmanCommand(int action);
};

#endif // LISTENMSGTHREAD_H