#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>
#include <sstream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>


std_msgs::Float64MultiArray ze_array;
std_msgs::Float64MultiArray tr_array;
std_msgs::Int8 traffic_direc;
using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_ze_tr");
  ros::NodeHandle n;
  int i = 0;

  ze_array.data.push_back(-1);

  ros::Publisher zebra_pub = n.advertise<std_msgs::Float64MultiArray>("/zebra", 1);
  ros::Publisher traffic_pub = n.advertise<std_msgs::Float64MultiArray>("/traffic", 1);
  ros::Publisher traffic_dir_pub = n.advertise<std_msgs::Int8>("/traffic_direction", 1);
  ros::Rate loop_rate(100);
  double begin = ros::Time::now().toSec();
  while (ros::ok())
  {

    
    tr_array.data.push_back(12-i);
    if(i > 13)
    {
      tr_array.data.clear();
      tr_array.data.push_back(-1);
    }
    traffic_direc.data = 1;
    cout<<tr_array.data[0]<< " " <<ze_array.data[0]<<endl;
    traffic_dir_pub.publish(traffic_direc);
    zebra_pub.publish(ze_array);
    traffic_pub.publish(tr_array);

    tr_array.data.clear();

    double check = ros::Time::now().toSec();

    if((check-begin)>=2)
    {
      i++;
      begin = check;
    }

    ros::spinOnce();

    loop_rate.sleep();
  

  }  
  return 0;
}
