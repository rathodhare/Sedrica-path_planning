#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <sstream>
#include <fstream>
#include <cmath>
#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>


geometry_msgs::Pose2D vel_sa;
geometry_msgs::Pose2D acc_br;
using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "status_e2o");
  ros::NodeHandle n;

  

  ros::Publisher chatter_pub1 = n.advertise<geometry_msgs::Pose2D>("chatter_vel_sa", 1);
  ros::Publisher chatter_pub2 = n.advertise<geometry_msgs::Pose2D>("chatter_acc_br", 1);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ifstream myfile("/home/sine/rise_final_pp/status_can_.txt");
    if(myfile.is_open())
    {
          myfile>>vel_sa.x;
          cout<<vel_sa.x<<" << ";
          myfile>>vel_sa.theta;
          cout<<vel_sa.theta<<" << ";
          myfile>>acc_br.x;
          cout<<acc_br.x<<" << ";
          myfile>>acc_br.theta;
          cout<<acc_br.theta<<endl;
    }
        myfile.close();
     
    chatter_pub1.publish(vel_sa);
    chatter_pub2.publish(acc_br);

    ros::spinOnce();

    loop_rate.sleep();
  

  }  
  return 0;
}
