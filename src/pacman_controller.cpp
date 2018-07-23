#include "ros/ros.h"
#include "pacman/Num.h"
#include "pacman/pacmanPos.h"
#include "pacman/pos.h"

void pacmanPosCallback(const pacman::pacmanPos::ConstPtr& msg)
{
  int x = msg->pacmanPos.x;
  int y = msg->pacmanPos.y;
  ROS_INFO("Pos Recibida:  x = [%d]  y = [%d]", x, y);
}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    srand (time(NULL));
    ros::init(argc, argv, "pacman_controller");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<pacman::Num>("exampletopic", 100);
    ros::Subscriber posPacmanSub = n.subscribe("pacmanCoord", 100, pacmanPosCallback);
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok())
    {
      pacman::Num msg;
      msg.num = rand() % 5;
      //ROS_INFO("%d", msg.num);
      chatter_pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
  return 0;
}