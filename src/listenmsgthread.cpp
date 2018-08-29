#include "pacman/listenmsgthread.h"

void ListenMsgThread::callback0(const pacman::Num::ConstPtr& msg)
{
    if(msg->num < 5 && work)
      emit UpdatePacmanCommand(msg->num, 0);
}
void ListenMsgThread::callback1(const pacman::Num::ConstPtr& msg)
{
    if(msg->num < 5 && work)
      emit UpdatePacmanCommand(msg->num, 1);
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