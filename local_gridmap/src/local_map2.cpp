//ros libraries
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Int8MultiArray.h"
//c++ libraries
#include <iostream>
#include <vector>
#include <utility>
#include <math.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2D.h>
//opencv libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

const float pi = 3.14159265;
const float RAD_TO_DEG = 180.000000/pi;
const float DEG_TO_RAD = pi/180.000000;

const float res = 0.25;          //angular resolution of lidar in degrees
const int num_values = 1080;     //no. of rays emitted by the lidar

const int radius_in_cm = 20;     //radius around on object to be in safe limit for path-planning
const int lane_in_cm = 0;       //radius around lane for path planning
const int side_in_cm = 15;
const int base_lidar_dist = 60;   //distance from vehicle centre to lidar in cm(only in y-direction)
float map_res = 0.3;//map resolution(m/cell)

//const int cam_lidar_dist = 30;    //distance from camera to lidar in cm

//const float base_cam_dist_in_cells = ((cam_lidar_dist - base_lidar_dist)*0.01)/map_res;

int l_width,l_height;//lane map size
int l_width_new,l_height_new;
float l_res = 0.05;          

const int width =60/map_res;//map size
const int height = 60/map_res;
const int range = 45/map_res;//10m in cells
ros::Publisher map_pub,blown_map_pub;
std_msgs::Int8MultiArray lane_map;
std_msgs::Int8MultiArray lane_map_new;

float starting_head=0,curr_head=0;
bool take_initial_head(true);

pair<float,float> convToCart(float i,float r ,int p) //convert from polar to cartesian co-ordinates
{
   float ang = i*0.25*DEG_TO_RAD;
   float x = r*cos(ang +(curr_head-starting_head)*DEG_TO_RAD);
   float y = p*r*sin(ang+(curr_head-starting_head)*DEG_TO_RAD);
   return make_pair(x,y);
}



// void laneCallback(nav_msgs::OccupancyGrid msg)
// {
//    int i,j;
//    l_width = msg.info.width;
//    l_height = msg.info.height;
//    //cout<<l_width<<" "<<l_height<<endl;
//    for(i=0;i<l_height;i++)//initialising map
//    {
//       for(j=0;j<l_width;j++)
//       {
//          lane_map.data.push_back(0);
//       }
//    }
   
//    for(i=l_height-1;i>=0;i--)//initialising map
//    {
//       for(j=0;j<l_width;j++)
//       {
//          int i1;
//          i1=l_height-1-i;

//          lane_map.data[i1*l_width+j] = msg.data[i*l_width+j];
//       }
//    }    
// }
 
void laneCallback(nav_msgs::OccupancyGrid msg)
{
   int i,j;
   l_width = msg.info.width;
   l_height = msg.info.height;
   l_width_new = 2*l_width*l_res/map_res;
   l_height_new = 2*l_height*l_res/map_res ;//+ base_cam_dist_in_cells ;
  // cout<<l_width_new<<" "<<l_height_new<<endl;
   for(i=0;i<l_height;i++)//initialising map
   {
      for(j=0;j<l_width;j++)
      {
         lane_map.data.push_back(0);
      }
   }

   for(i=0;i<l_height_new;i++)//initialising map
   {
      for(j=0;j<l_width_new;j++)
      {
         lane_map_new.data.push_back(0);
      }
   }
   

   for(i=l_height-1;i>=0;i--)//initialising map
   {
      for(j=0;j<l_width;j++)
      {
         double i1,x;
         i1=l_height-1-i;
         lane_map.data[i1*l_width+j] = msg.data[i*l_width+j];
         if(lane_map.data[i1*l_width+j]==2){ 
            //x = ((l_height_new-((l_height-i1)*l_res/map_res))*l_width_new)+(((j- l_width/2)*l_res/map_res)+ l_width_new/2);
            float dist = sqrt(i1*i1*l_res*l_res+(j-l_width/2)*(j-l_width/2)*l_res*l_res);
            double ang_lane =  atan2(i1,(j-l_width/2));
            vector< pair<float,float> > lane_cartesian(1,make_pair(0,0));
            lane_cartesian[0] = convToCart(4*ang_lane*RAD_TO_DEG-(curr_head-starting_head)*0 ,dist,1);
            int x_coord_lane = round(lane_cartesian[0].first/map_res) ;
            int y_coord_lane = round(lane_cartesian[0].second/map_res) + 30/map_res ; 
            lane_map_new.data[y_coord_lane*l_width_new+x_coord_lane+l_width_new/2] = 2; 
            //x = ((int(i1*l_res/map_res))*l_width_new)  +  int(j*l_res/map_res) +1  ;
            //cout<<x<<endl;
            //lane_map_new.data[x] = 2;
            
         }
      }
   }   

} 
void current_heading(const geometry_msgs::Pose2D::ConstPtr& msg){
   curr_head = msg->theta;
   if(take_initial_head)
   {
      starting_head = curr_head;
      take_initial_head = false;
   }
}
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
   //float map_res = 0.4;//map resolution(m/cell)
float min_r=12,min_ang=0;

   float radius_in_cells;
   float minor_axis_in_cells = (lane_in_cm*0.01)/map_res;//elliptically blowing lanes
   float major_axis_in_cells = minor_axis_in_cells;
   float base_lidar_dist_in_cells = (base_lidar_dist*0.01)/map_res;
   
   //cout<<base_lidar_dist_in_cells<<endl;
   int i,j,k,l;

   nav_msgs::OccupancyGrid grid_map;
   grid_map.header.stamp = ros::Time::now();
   grid_map.header.frame_id = "/map";
   grid_map.info.resolution = map_res;
   grid_map.info.origin.position.x = 0.0;
   grid_map.info.origin.position.y = 0.0;
   grid_map.info.origin.position.z = 0.0;
   grid_map.info.origin.orientation.x = 0.0;
   grid_map.info.origin.orientation.y = 0.0;
   grid_map.info.origin.orientation.z = 0.0;
   grid_map.info.origin.orientation.w = 1.0;
   grid_map.info.width = width;
   grid_map.info.height = height;
   grid_map.info.map_load_time = ros::Time::now();

   nav_msgs::OccupancyGrid blown_grid_map;
   blown_grid_map.header.stamp = ros::Time::now();
   blown_grid_map.header.frame_id = "/map";
   blown_grid_map.info.resolution = map_res;
   blown_grid_map.info.origin.position.x = 0.0;
   blown_grid_map.info.origin.position.y = 0.0;
   blown_grid_map.info.origin.position.z = 0.0;
   blown_grid_map.info.origin.orientation.x = 0.0;
   blown_grid_map.info.origin.orientation.y = 0.0;
   blown_grid_map.info.origin.orientation.z = 0.0;
   blown_grid_map.info.origin.orientation.w = 1.0;
   blown_grid_map.info.width = width;
   blown_grid_map.info.height = height;
   blown_grid_map.info.map_load_time = ros::Time::now();
   
   vector< pair<float,float> > cartesian(num_values,make_pair(0,0));
   for(i=num_values/4;i<3*num_values/4;i++)
   {
      //cartesian[i] = convToCart((i-num_values/2),msg->ranges[i]);
      cartesian[i] = convToCart((i-num_values/2),msg->ranges[i-180],-1);
      cartesian[i].first += base_lidar_dist*0.01;
      if(msg->ranges[i-180]<min_r) {min_r = msg->ranges[i-180];
         min_ang = i*res;
         }

   }
   //if(min_r<=2.5)
   //cout<<min_r<<" "<<min_ang<<endl;
   //float min=20;
   /*for(i=0;i<num_values;i++)
   {
      if(msg->ranges[i]<min)
      {min=msg->ranges[i];}
      //if(msg->ranges[i]==0.0)
      //{cout<<i;}
   }*/
   //cout<<"min="<<min<<",straight="<<msg->ranges[540]<<endl;
   //int def=round(cartesian[540].first/map_res);
   //cout<<def;
   //std_msgs::Int8MultiArray map;
   for(i=0;i<height;i++)//initialising map
   {
      for(j=0;j<width;j++)
      {
         grid_map.data.push_back(0);
         blown_grid_map.data.push_back(0);
      }
   }
   //cout<<"hi"<<endl;
   for(i=0;i<num_values;i++)
   {
      int x_coord = round(cartesian[i].first/map_res) + 30/map_res;
      int y_coord = round(cartesian[i].second/map_res) ;
      /*if(x_coord==0 && y_coord==0)
      {
        cout<<i<<" "<<msg->ranges[i];
      }*/
      if(x_coord > 0 && x_coord <= range)//limiting obstacle detection to only 10m
      {
         if(y_coord <= range && y_coord >= -range)
         {
            if(!(x_coord==(99)))
            grid_map.data[x_coord*width+y_coord+width/2] = 100;
         } 
      }
   }
   //int x_ = round((cartesian[540].first + base_lidar_dist)/map_res);
   //int y_ = round(cartesian[540].second/map_res);
   //int a=blown_grid_map.data[400];
   //int b=grid_map.data[400];
   //cout<<a<<" "<<b<<endl;
   //grid_map.data = map.data;
   
   // for(i=0;i<l_height;i++)//adding lane data 
   // {
   //    for(j=0;j<l_width;j++)
   //    {
   //       if(lane_map.data[(i+base_cam_dist_in_cells)*l_width+j]==2)
   //       grid_map.data[i*width+j+width/2-l_width/2] = 50;
   //    }
   // }

      // for(i=0;i<l_height;i++)//adding lane data 
      // {
      //    for(j=0;j<l_width;j++)
      //    {
      //        if(lane_map.data[(i+base_cam_dist_in_cells)*l_width+j]==2)
      //        grid_map.data[i*width+j+width/2-l_width/2] = 50;
      //    }
      // }
    

     for(int i=0;i<l_height_new;i++)//adding lane data 
      {
         for(int j=0;j<l_width_new;j++)
         {
             if(lane_map_new.data[(i)*l_width_new+j]==2)
               //if(lane_map_new.data[(i+base_cam_dist_in_cells)*l_width_new+j-1]!=2)
             grid_map.data[(i)*width+int(j+width/2-l_width_new/2)] = 50;
             //cout<<int(i*width+j+width/2-l_width_new/2)<<endl;
               
         }
      }


   for(i=0;i<height;i++)//creating blown map
   {
      for(j=0;j<width;j++)
      {
         if(grid_map.data[i*width+j]==100)//blowing obstacle data
         {
            if(i<base_lidar_dist_in_cells)
            radius_in_cells = (side_in_cm*0.01)/map_res;
            else
            radius_in_cells = (radius_in_cm*0.01)/map_res;

            for(k=-1*radius_in_cells;k<=radius_in_cells;k++)
            {
               for(l=-1*radius_in_cells;l<=radius_in_cells;l++)
               {
                  if((k*k + l*l) < radius_in_cells*radius_in_cells)
                  {
                     int x = i + k;
                     int y = j + l - width/2;
                     if(x >= 0 && x <= range)
                     {
                        if(y <= range && y >= -range)
                        {
                           blown_grid_map.data[x*width+y+width/2] = 100;
                        }
                     }
                  }
               }
            }
         }
         
         if(grid_map.data[i*width+j]==50)//blowing lane data
         {
            for(k=-1*major_axis_in_cells;k<=major_axis_in_cells;k++)
            {
               for(l=-1*minor_axis_in_cells;l<=minor_axis_in_cells;l++)
               {
                  if(((minor_axis_in_cells*minor_axis_in_cells)*(k*k) + (major_axis_in_cells*major_axis_in_cells)*(l*l)) < (minor_axis_in_cells*minor_axis_in_cells)*(major_axis_in_cells*major_axis_in_cells))//inside ellipse condition
                  {
                     int x = i + k;
                     int y = j + l - width/2;
                     if(x >= 0 && x <= range)
                     {
                        if(y <= range && y >= -range)
                        {
                           blown_grid_map.data[x*width+y+width/2] = 80;
                        }
                     }
                  }
               }
            }
         }

      }
   }   

   map_pub.publish(grid_map);
   blown_map_pub.publish(blown_grid_map);
   Mat local_map = Mat::zeros(height,width,CV_8UC1);
   Mat blown_local_map = Mat::zeros(height,width,CV_8UC1);
   for(i=0;i<height;i++)
   {
     for(j=0;j<width;j++)
     {
       int i_= height - i;
       local_map.at<uchar>(i,j) = grid_map.data[i_*width+j]*2.55;
       blown_local_map.at<uchar>(i,j) = blown_grid_map.data[i_*width+j]*2.55;
       //local_map.at<Vec3b>(i,j)[1] = map.data[i*width+j]*255;
       //local_map.at<Vec3b>(i,j)[2] = map.data[i*width+j]*255;
     }
   }
   imshow("local_map",local_map);
   imshow("blown_local_map",blown_local_map);
   waitKey(1);
   lane_map.data.clear();
   lane_map_new.data.clear();
   grid_map.data.clear();
   blown_grid_map.data.clear();
}

int main(int argc,char** argv)
{
   ros::init(argc,argv,"local_map_new");
   ros::NodeHandle n,n1,n2,n3;
   ros::Subscriber lane_sub=n.subscribe<nav_msgs::OccupancyGrid>("/Lane_Occupancy_Grid",1,laneCallback);
   ros::Subscriber head_sub = n.subscribe("/imu/HeadingTrue_degree", 1, current_heading);
   ros::Subscriber lidar_sub=n1.subscribe<sensor_msgs::LaserScan>("scan",1,lidarCallback);
   map_pub=n2.advertise<nav_msgs::OccupancyGrid>("scan/local_map",1);
   blown_map_pub=n3.advertise<nav_msgs::OccupancyGrid>("scan/blown_local_map",1);

   namedWindow("local_map",CV_WINDOW_NORMAL);
   namedWindow("blown_local_map",CV_WINDOW_NORMAL);

   ros::Rate loop_rate(15.0);

   while(ros::ok())
   {
     ros::spinOnce();//check for incoming messages
     loop_rate.sleep(); 
   }
   return 0; 
}
