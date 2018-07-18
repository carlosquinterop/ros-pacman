#include "pacman/listenmsgthread.h"

void ListenMsgThread::callback(const pacman::Num::ConstPtr& msg)
{
    //ROS_INFO("I heard: [%d]", msg->num);
    if(msg->num < 5)
      emit UpdatePacmanCommand(msg->num);
}

ListenMsgThread::ListenMsgThread()
{
    char **argv;
    int argc = 0;
    ros::init(argc, argv, "pacman_world");
    node = new ros::NodeHandle();
}

void ListenMsgThread::setWorkingThread()
{
    node->shutdown();
}

void ListenMsgThread::run()
{
    sub = node->subscribe("exampletopic", 100, &ListenMsgThread::callback, this);
    ros::spin();      
}