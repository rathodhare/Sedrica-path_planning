#include "ros/ros.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <SerialStream.h>
#include "std_msgs/String.h"
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>

int str_theta_des;

float velD_err,velD_des=2,velD_curr,velD_out=0;
float velD_KP=8.0,velD_KI=15,velD_KD=0.3;

float velD_integral = 0;
float velD_diff;
float velD_ittime = 0.08;
float velD_err_prior = 0;
float curr_vel_factor = 1;

float acc_thresh=12;

float brake_scale=0.008,acc_scale=0.001,vel_scale=1;

int brake_can=0,acc_can=0,str_can=0,gear_can=1,velD_curr_can=0;

char final_can_[17],brake_can_[3],acc_can_[3],gear_can_,str_can_[3],ind_can_=0;

const float pi = 3.14159265;
const float RAD_TO_DEG = 180.0/pi;
const float DEG_TO_RAD = pi/180.0;

bool master;

using namespace std;

ofstream myfile_write;

void callback(const std_msgs::String::ConstPtr& msg)////////////////////////write callback
{
	stringstream(msg->data)>>str_theta_des;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavioral_cloning");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/message",1,callback);
  ros::Rate loop_rate(200);


  while (ros::ok())
  {

  	velD_curr = velD_curr_can*5/18*curr_vel_factor;
    velD_err = (velD_des - velD_curr); 
    velD_integral = velD_integral + velD_err*velD_ittime;
    velD_diff = (velD_err - velD_err_prior)/velD_ittime;

    velD_out = velD_KP*velD_err + velD_KI*velD_integral + velD_KD*velD_diff;
    velD_out = velD_out*vel_scale;
    velD_err_prior = velD_err;
    //cout<<"velD_out = "<<velD_out<<endl;
    if (-velD_out*acc_scale<=2)
    {
      acc_can = 100 + velD_out*acc_scale + acc_thresh;
      brake_can = 100;
      gear_can = 2;
    }
    else 
    {
      acc_can = 100;
      brake_can = 100 - velD_out*brake_scale;
      gear_can = 2;
    }

    if(acc_can>200) acc_can=200;
    if(acc_can<100) acc_can=100;
    if(brake_can>200) brake_can=200;
    if(brake_can<100) brake_can=100;

    cout<<"acc_can = "<<acc_can<<" "<<"brake_can = "<<brake_can<<endl;

    for (int i = 0; i<3;i++)
      {
        acc_can_[i] = acc_can/pow(10,(2-i));
        acc_can = acc_can%(int)(pow(10,(2-i)));
      }
    for (int i = 0; i<3;i++)
      {
        brake_can_[i] = brake_can/(pow(10,(2-i)));
        brake_can = brake_can%(int)pow(10,(2-i));
      }
      
    gear_can_ = gear_can;
    


   
    final_can_[0]='/';
    final_can_[1]=gear_can_+48;
    final_can_[2]=ind_can_+48;
    final_can_[3]='0';
    for(int i = 4;i<7;i++)
      final_can_[i]=str_can_[i-4]+48;
    for(int i = 7;i<10;i++)
      final_can_[i]=brake_can_[i-7]+48;
    for(int i = 10;i<13;i++)
      final_can_[i]=acc_can_[i-10]+48;
    final_can_[13]=final_can_[14]='0';
    final_can_[15]=0x5C;
    final_can_[16]='n';


    myfile_write.open("/home/sine/rise_final_pp/final_can_.txt", ios::trunc);
    for(int i=0;i<17;i++)
    {
    	myfile_write<<final_can_[i];
    }
    myfile_write.close();

 	ifstream myfile_read0("/home/sine/rise_final_pp/status_can_.txt"); 
	if(myfile_read0.is_open())
    {  
        myfile_read0>>velD_curr_can;
        myfile_read0.close();
    } 
    //cout<<velD_curr_can<<endl;

/*
    ifstream myfile_read1("/home/sine/rise_final_pp/status_can_.txt"); ///////////change path
	if(myfile_read1.is_open())
    {  
        myfile_read1>>str_theta_des;
        myfile_read1.close();
    } */
    str_theta_des = max((int)str_theta_des,-40);
    str_theta_des = min((int)str_theta_des,40);

    str_can = 140 + str_theta_des;
    //cout<<"str_can = "<<str_can<<endl;

    for (int i = 0; i<3;i++)
      {
        str_can_[i] = str_can/(pow(10,(2-i)));
        str_can = str_can%(int)pow(10,(2-i));
      }
     

   	ros::spinOnce();

    loop_rate.sleep();

  }
 


  return 0;
}