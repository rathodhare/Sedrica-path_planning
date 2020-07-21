#include "ros/ros.h"
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <cmath>
#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
float y=3;
float y_initial = 3.5;
float res_path = 0.5;
float y_des_i=1;
float x_des,y_des;
float Dincm = 1;

float str_theta_err,str_theta_des,str_theta_out;
float str_KP =0.7,str_KI=0.02,str_KD=0.001;

float str_integral = 0;
float str_diff;
float str_ittime = 0.3;
float str_err_prior = 0;

float velD_err,velD_des,velD_curr,velD_out=0;
float velD_KP=0.51,velD_KI=0.1,velD_KD=0.003;

float velD_integral = 0;
float velD_diff;
float velD_ittime = 0.005;
float velD_err_prior = 0;

float vel_scale_wo_obs = 0.8, vel_scale_w_obs = 0.4, vel_scale = vel_scale_wo_obs,sa_scale_wo_obs=0.8,sa_scale_w_obs = 1,sa_scale=sa_scale_wo_obs,acc_scale=5,brake_scale=20,y_scale=7*5/18;
float acc_thresh=15;

int width,height;
float map_res,frt_obs_wid=1.5,frt_obs_dist=3;

nav_msgs::OccupancyGrid grid;


int brake_can=0,acc_can=0,str_can=0,gear_can=1,velD_curr_can=0;

char final_can_[17],brake_can_[3],acc_can_[3],gear_can_,str_can_[3];

const float pi = 3.14159265;
const float RAD_TO_DEG = 180.0/pi;
const float DEG_TO_RAD = pi/180.0;

bool master;
int count1=0;

using namespace std;

ofstream myfile_write;

bool check_updated_path=true,new_path_recieved = false,previous_path_updated = false;
geometry_msgs::Pose previous_position_in_map,current_pos_in_map,current_local_pos,previous_local_pos;

nav_msgs::Path previous_path,new_path;

float curr_head   = 0,initial_heading = 0,delta_head = 0;
float loc_r=0;

bool currhead_received(false) ,initial_heading_recieved(true),map_recieved(false);

void car_vel_bool(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == 1)
    master = 1;
  else
    master = 0;
}

void e2o_vel(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if (master == 1)
  {
    velD_des = msg->data[msg->data.size() - y_des_i];
    //cout<<"vel_arr\n";
    /*velD_curr = velD_curr_can*5/18;
    velD_err = (velD_des - velD_curr)/Dincm; 
    velD_integral = velD_integral + velD_err*velD_ittime;
    velD_diff = (velD_err - velD_err_prior)/velD_ittime;

    velD_out = velD_KP*velD_err + velD_KI*velD_integral + velD_KD*velD_diff;
    velD_out = velD_out*vel_scale;
    velD_err_prior = velD_err;
    //cout<<velD_out<<endl;
    if (velD_out>0)
    {
      acc_can = 100+velD_out*acc_scale+acc_thresh;
      brake_can = 100+0;
      gear_can = 2;
    }
    else if(velD_out <0)
    {
      acc_can = 100+0;
      brake_can = 100+velD_out*brake_scale;
      gear_can = 2;
    }*/
  }
  else
  {
    /*{
      acc_can = 100+0;
      brake_can = 100+90;
      gear_can = 1;
    }*/
  }
  
  //cout<<acc_can<<" "<<brake_can<<endl;

 /* for (int i = 0; i<3;i++)
    {
      acc_can_[i] = acc_can/pow(10,(2-i));
      acc_can = acc_can%(int)(pow(10,(2-i)));
    }
  for (int i = 0; i<3;i++)
    {
      brake_can_[i] = brake_can/(pow(10,(2-i)));
      brake_can = brake_can%(int)pow(10,(2-i));
    }
    
  gear_can_ = gear_can;*/
  

}

void update_curr_position(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  //cout<<" enter update_curr_position"<<endl;
  loc_r =sqrt(msg->x*msg->x + msg->y*msg->y);
  //cout<<"ok\n";
  curr_head = msg->theta;
  currhead_received = true;
  if(initial_heading_recieved)
  {
    initial_heading = msg->theta;
    initial_heading_recieved = false;
  }
  delta_head = curr_head - initial_heading;
  current_local_pos.position.x= loc_r*cos(delta_head);
  current_local_pos.position.y= loc_r*sin(delta_head);
  //cout<<"1\n";
  if(check_updated_path&&new_path_recieved)
  {
  current_pos_in_map.position.x= (current_local_pos.position.x - previous_local_pos.position.x)*cos(delta_head) + (current_local_pos.position.y - previous_local_pos.position.y)*sin(delta_head) + new_path.poses[new_path.poses.size()-1].pose.position.x;
  current_pos_in_map.position.y= (current_local_pos.position.x - previous_local_pos.position.x)*sin(delta_head)*(-1) + (current_local_pos.position.y - previous_local_pos.position.y)*cos(delta_head) + new_path.poses[new_path.poses.size()-1].pose.position.y;
  //cout<<"exit update_curr_position"<<endl;
}
}
void e2o_sa(const nav_msgs::Path::ConstPtr& msg)
{
  if(!msg->poses.empty())
  {
    new_path = (*msg);
    new_path_recieved = true;
  }
  
}




void map_in(const nav_msgs::OccupancyGrid::Ptr map) {
  map_recieved = true;
  grid = (*map);
  width = grid.info.width;
  height = grid.info.height;
  map_res = grid.info.resolution;
  for(int i = (int)(width/2 - frt_obs_wid/(2*map_res));i<width/2 + frt_obs_wid/(2*map_res);i++)
  {
    for(int j = 0;j<frt_obs_dist/(map_res);j++)
    {
      if(grid.data[j*width + i]>0)
      {
        sa_scale = sa_scale_w_obs;
        vel_scale = vel_scale_w_obs;

        goto no_obs;
      }
      else 
        {
          sa_scale = sa_scale_wo_obs;
          vel_scale = vel_scale_wo_obs;
          velD_des = 0.5;
        }
    }
  }
  no_obs:;
}
/*
void current_heading(const geometry_msgs::Pose2D::ConstPtr& msg){
  curr_head = msg->theta;
  currhead_received = true;
  if(initial_heading_recieved)
  {
    initial_heading = msg->theta;
    initial_heading_recieved = false;
  }
}*/

int main(int argc, char **argv)
{
  previous_local_pos.position.x=0;
  previous_local_pos.position.y=0;
  ros::init(argc, argv, "talker_e2o_loc_vel");
 

  ros::NodeHandle n1,n2,n3;
  ros::Subscriber path_sub = n1.subscribe("/path", 1, e2o_sa);
  ros::Subscriber car_heading = n2.subscribe("/newpath_required", 1, car_vel_bool);
  ros::Subscriber vel_sub = n3.subscribe("/velocity_array", 1, e2o_vel);
  ros::Subscriber map_sub = n1.subscribe("scan/local_map",1,map_in);
  ros::Subscriber loc_sub = n1.subscribe("angle_for_pp",1,update_curr_position); //odometry/fil_map
  //ros::Subscriber sub2 = n1.subscribe("/imu/HeadingTrue_degree", 1, current_heading);

  ros::Rate loop_rate(200);
  //ros::spin();
  /*current_local_pos.position.x=0;
  current_local_pos.position.y=0;
  current_pos_in_map.position.x = width/2;
  current_pos_in_map.position.y = 0;
*/
  while (ros::ok())
  {

   
    
  ifstream myfile_read("/home/sine/rise_final_pp/status_can_.txt"); 
  if(myfile_read.is_open())
    {
       
          myfile_read>>velD_curr_can;
            //cout<<velD_curr_can<<endl;
        
        myfile_read.close();
    } 
    if(master==1)
  {
    if(previous_path_updated)
    {
       if(previous_path.header.stamp==new_path.header.stamp) 
      check_updated_path = false;
    else check_updated_path = true;
    }
   
    //cout<<"check_updated_path "<<check_updated_path<<endl;
    if(check_updated_path&&!new_path.poses.empty()&&new_path_recieved)
    {
      //cout<<"new_path"<<endl;
      previous_path = new_path;
      previous_path_updated = true;
      for(int i=new_path.poses.size()-1;i>=0;i--)
      {

          if(abs(new_path.poses[i].pose.position.y - y) <  res_path)
          {
            y_des_i = i;
            //cout<<i<<"  "<<new_path.poses.size()<<endl;
            break;
          }
      }
      x_des = new_path.poses[y_des_i].pose.position.x;
      y_des = new_path.poses[y_des_i].pose.position.y;
      str_theta_des = atan((x_des - new_path.poses[new_path.poses.size()-1].pose.position.x)/(y_des-new_path.poses[new_path.poses.size()-1].pose.position.y));
      previous_position_in_map.position.x = new_path.poses[new_path.poses.size()-1].pose.position.x;previous_position_in_map.position.y = new_path.poses[new_path.poses.size()-1].pose.position.y;
      previous_local_pos.position.x=current_local_pos.position.x;
      previous_local_pos.position.y=current_local_pos.position.y;cout<<str_theta_des*RAD_TO_DEG<<endl;


  /*str_theta_err = str_theta_des;
  //cout<<str_theta_des<<"yesss\n"; 
  str_integral = str_integral + str_theta_err*str_ittime;
  str_diff = (str_theta_err - str_err_prior)/str_ittime;

  str_theta_out = str_KP*str_theta_err + str_KI*str_integral + str_KD*str_diff;
  //cout<<" str str_theta_out "<<str_theta_out<<" " <<str_integral<<" "<<str_diff<< endl;
  str_err_prior = str_theta_err;
  str_theta_out = str_theta_out*RAD_TO_DEG;

  if (str_theta_out>40 )
    str_theta_out = 40;
  else if(str_theta_out <-40)
    str_theta_out = -40;

  //cout<<sa_scale<<" "<<vel_scale<<endl;
  str_can = str_theta_out*sa_scale + 140;
  cout<<str_can<<"yaa"<<endl;*/

    }

  else if(!previous_path.poses.empty())
    {

      //cout<<"old_path"<<endl;
      for(int i=previous_path.poses.size()-1;i>=0;i--)
      {

          if(abs(previous_path.poses[i].pose.position.y-y-current_pos_in_map.position.y)<res_path)
          {
            y_des_i = i;
            //cout<<i<<"  "<<previous_path.poses.size()<<endl;
            break;
          }
      }
      x_des = previous_path.poses[y_des_i].pose.position.x;
      y_des = previous_path.poses[y_des_i].pose.position.y;
      str_theta_des=atan((x_des - current_pos_in_map.position.x)/(y_des - current_pos_in_map.position.y));
      previous_position_in_map = current_pos_in_map;
      //velD_des = 
      //cout<<str_theta_des*RAD_TO_DEG<<endl;
    }   
  str_theta_err = str_theta_des; 
  str_integral = str_integral + str_theta_err*str_ittime;
  str_diff = (str_theta_err - str_err_prior)/str_ittime;

  str_theta_out = str_KP*str_theta_err + str_KI*str_integral + str_KD*str_diff;
  //cout<<" str str_theta_out "<<str_theta_out<<" " <<str_integral<<" "<<str_diff<< endl;
  str_err_prior = str_theta_err;
  str_theta_out = str_theta_out*RAD_TO_DEG;

  if (str_theta_out>40 )
    str_theta_out = 40;
  else if(str_theta_out <-40)
    str_theta_out = -40;

  //cout<<sa_scale<<" "<<vel_scale<<endl;
  str_can = str_theta_out*sa_scale + 140;
  //cout<<str_can<<"yess"<<endl;
    for (int i = 0; i<3;i++)
      {
        str_can_[i] = str_can/pow(10,(2-i));
        str_can = str_can%(int)pow(10,(2-i));
      }
    




    velD_curr = velD_curr_can*5/18;
    velD_err = (velD_des - velD_curr)/Dincm; 
    velD_integral = velD_integral + velD_err*velD_ittime;
    velD_diff = (velD_err - velD_err_prior)/velD_ittime;

    velD_out = velD_KP*velD_err + velD_KI*velD_integral + velD_KD*velD_diff;
    velD_out = velD_out*vel_scale;
    velD_err_prior = velD_err;
    cout<<velD_out<<endl;
    if (-velD_out*acc_scale<=acc_thresh)
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
   
  

else
{
  str_can = 140;
    for (int i = 0; i<3;i++)
      {
        str_can_[i] = str_can/pow(10,(2-i));
        str_can = str_can%(int)pow(10,(2-i));
      }

  {
    acc_can = 100+0;
    brake_can = 100+90;
    gear_can = 1;
  }
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
    

  

final_can_[0]='/';
    final_can_[1]=gear_can_+48;
    final_can_[2]=final_can_[3]='0';
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

ros::spinOnce();

 loop_rate.sleep();

  }
 


  return 0;
}
