#include "ros/ros.h"
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>
#include <cmath>
#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
float y;
float y_initial = 3;
float res_path = 0.5;
float y_des_i=1;
float x_des,y_des;
float Dincm = 1;
int turn = 10;

float str_theta_err,str_theta_des,str_theta_out;
float str_KP = 0.65,str_KI=0.0002,str_KD=0.01;

float str_integral = 0;
float str_diff;
float str_ittime = 0.3;
float str_err_prior = 0;

float velD_err,velD_des,velD_curr,velD_out=0;
float velD_KP=1950.0,velD_KI=35,velD_KD=0.03; //old value KP = 1850 old value KI 24 old value 0.3

float velD_integral = 0;
float velD_diff;
float velD_ittime = 0.08;
float velD_err_prior = 0;
float curr_vel_factor = 1;

float vel_scale_wo_obs = 0.5, vel_scale_w_obs = 0.1, vel_scale = vel_scale_wo_obs,sa_scale_wo_obs=0.8,sa_scale_w_obs = 1.4,sa_scale=sa_scale_wo_obs,acc_scale=2,brake_scale=8,y_scale=0.9;//y_scale = 1.2 //old acc_scale = 5
float acc_thresh=12;

int width,height;
float map_res,frt_obs_wid=2.8,frt_obs_dist=4.5;

bool traffic_override = false;

nav_msgs::OccupancyGrid grid;
geometry_msgs::Point acc_brake_cnn;


int brake_can=0,acc_can=0,str_can=0,gear_can=1,velD_curr_can=0;

char final_can_[17],brake_can_[3],acc_can_[3],gear_can_,str_can_[3],ind_can_=0;

const float pi = 3.14159265;
const float RAD_TO_DEG = 180.0/pi;
const float DEG_TO_RAD = pi/180.0;

bool master;

using namespace std;

ofstream myfile_write;


void car_vel_bool(const std_msgs::Bool::ConstPtr& msg)
{
  if ((msg->data == 1) && !traffic_override)
    master = 1;
  else
    master = 0;
}

void e2o_vel(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	if (msg->data[1] == 0)
		traffic_override = true;
	else 
		traffic_override = false;

  if (master == 1)
  {
    velD_des = msg->data[y_des_i];
    velD_curr = velD_curr_can*5.0/18.0*curr_vel_factor;
    cout<<"velD_des "<<velD_des;
    velD_err = (velD_des - velD_curr)/Dincm;
    //cout<<"Dincm"<<Dincm<<endl; 
    velD_integral = velD_integral + velD_err*velD_ittime;
    velD_diff = (velD_err - velD_err_prior)/velD_ittime;
    cout<<"velD_curr "<<velD_curr;
    cout<<"error "<<velD_err; 
    velD_out = velD_KP*velD_err + velD_KI*velD_integral + velD_KD*velD_diff;
    velD_out = velD_out*vel_scale;
    velD_err_prior = velD_err;
    cout<<" p value "<<velD_KP*velD_err;
    cout<<" i value"<<velD_KI*velD_integral;
    cout<<" d value"<<velD_KD*velD_diff;
    cout<<"velD_out"<<velD_out<<endl;
    if(1)// (-velD_out*acc_scale<=2)
    {
      acc_can = 100 + velD_out*acc_scale+acc_thresh;
      brake_can = 100;
      gear_can = 2;
    }
    else 
    {
      acc_can = 100;
      brake_can = 100 - velD_out*brake_scale;
      gear_can = 2;
    }
  }
  else
  {
    {
      acc_can = 100;
      brake_can = 190;
      gear_can = 1;
    }
  }

  acc_brake_cnn.x = acc_can - 100;
  acc_brake_cnn.y = brake_can - 100;

  cout<<vel_scale<<" << "<<sa_scale<<endl;
  cout<<"y "<<y_des_i<<endl;
  //cout<<"master "<<master<<endl;
  //cout<<acc_can<<" "<<brake_can<<endl;
  if(acc_can>200) acc_can=200;
  if(acc_can<100) acc_can=100;
  if(brake_can>200) brake_can=200;
  if(brake_can<100) brake_can=100;
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
  

}


void e2o_sa(const nav_msgs::Path::ConstPtr& msg)
{
  if(master==1)
  {
  if(!msg->poses.empty())
  {
    y = y_scale*velD_curr + y_initial;
    cout<<y<<endl;
    Dincm = 1;
  for(int i=msg->poses.size();i>=0;i--)
  {

      if(abs(msg->poses[i].pose.position.y - y) <  res_path)
      {
        y_des_i = i;
        break;
      }
  }
  for(int i=msg->poses.size();i>y_des_i;i--)
    Dincm = Dincm + 100*sqrt(pow((msg->poses[i+1].pose.position.x-msg->poses[i].pose.position.x),2) + pow((msg->poses[i+1].pose.position.y-msg->poses[i].pose.position.y),2));


  x_des = msg->poses[y_des_i].pose.position.x;
  y_des = msg->poses[y_des_i].pose.position.y;

  str_theta_des = atan((x_des-msg->poses[msg->poses.size()-1].pose.position.x)/(y_des-msg->poses[msg->poses.size()-1].pose.position.y));
  //cout<<str_theta_des*RAD_TO_DEG<<endl;
  str_theta_err = str_theta_des; 
  str_integral = str_integral + str_theta_err*str_ittime;
  str_diff = (str_theta_err - str_err_prior)/str_ittime;

  str_theta_out = str_KP*str_theta_err + str_KI*str_integral + str_KD*str_diff;

  str_err_prior = str_theta_err;
  //cout<<str_theta_out<<"yaa"<<endl;
  str_theta_out = str_theta_out*RAD_TO_DEG;

  if (str_theta_out>40 )
    str_theta_out = 40;
  else if(str_theta_out <-40)
    str_theta_out = -40;


  str_can = str_theta_out*sa_scale + 140;
  //cout<<str_can<<"yaa"<<endl;
  if(str_can>(140+turn))
    ind_can_ = 2;
  else if(str_can<(140-turn))
    ind_can_ = 1;
  else
    ind_can_ = 0;
  for (int i = 0; i<3;i++)
    {
      str_can_[i] = str_can/pow(10,(2-i));
      str_can = str_can%(int)pow(10,(2-i));
    }
  }
 // cout<<"empty\n";

  }
  else
  {
    str_can = 140;
    ind_can_ = 0;  
    for (int i = 0; i<3;i++)
    {
      str_can_[i] = str_can/pow(10,(2-i));
      str_can = str_can%(int)pow(10,(2-i));
    }
  }

  
}

void map_in(const nav_msgs::OccupancyGrid::Ptr map) {
  grid = (*map);
  width = grid.info.width;
  height = grid.info.height;
  map_res = grid.info.resolution;
  for(int i = (int)(width/2 - frt_obs_wid/(2*map_res));i<width/2 + frt_obs_wid/(2*map_res);i++)
  {
    for(int j = 0;j<frt_obs_dist/(map_res);j++)
    {
      if(grid.data[j*width + i]==100)
      {
        sa_scale = sa_scale_w_obs;
        vel_scale = vel_scale_w_obs;

        goto no_obs;
      }
      else 
        {
          sa_scale = sa_scale_wo_obs;
          vel_scale = vel_scale_wo_obs;
        }
        if(grid.data[j*width + i]==80)
      {
        sa_scale = sa_scale_w_obs;
        //vel_scale = vel_scale_w_obs;

        goto no_obs;
      }
      else 
        {
          sa_scale = sa_scale_wo_obs;
          //vel_scale = vel_scale_wo_obs;
        }
    }
  }
  no_obs:;
  //cout<<vel_scale<<" << "<<sa_scale<<endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker_e2o_vel_ros");
 

  ros::NodeHandle n,n1,n2,n3;
  ros::Subscriber path_sub = n1.subscribe("/path", 1, e2o_sa);
  ros::Subscriber car_heading = n2.subscribe("/newpath_required", 1, car_vel_bool);
  ros::Subscriber vel_sub = n3.subscribe("/velocity_array", 1, e2o_vel);
  ros::Subscriber map_sub = n1.subscribe("scan/local_map",1,map_in);
  ros::Publisher cnn_pub = n.advertise<geometry_msgs::Point>("/acc_brake_cnn",1);

  ros::Rate loop_rate(200);
//ros::spin();

  while (ros::ok())
  {
   
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

/*for(int i=0;i<17;i++)
cout<<final_can_[i];
 cout<<endl;*/
 ifstream myfile_read("/home/sine/rise_final_pp/status_can_.txt"); 
if(myfile_read.is_open())
    {
       
          myfile_read>>velD_curr_can;
            //cout<<velD_curr_can<<endl;
        
        myfile_read.close();
    } 

  cnn_pub.publish(acc_brake_cnn);
   ros::spinOnce();

    loop_rate.sleep();

  }
 


  return 0;
}