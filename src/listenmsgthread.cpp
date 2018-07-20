#include "pacman/listenmsgthread.h"

void ListenMsgThread::callback(const pacman::Num::ConstPtr& msg)
{
    if(msg->num < 5 && work)
      emit UpdatePacmanCommand(msg->num);
}

ListenMsgThread::ListenMsgThread()
{
    work = false;
}

void ListenMsgThread::setWorkingThread(bool aWork)
{
    work = aWork;
}

void ListenMsgThread::run()
{
    ros::spin();      
}