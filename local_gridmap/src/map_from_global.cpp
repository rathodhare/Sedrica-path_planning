//ros libraries
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_datatypes.h>
//c++ libraries
#include <iostream>
#include <utility>
#include <math.h>
//opencv libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

geometry_msgs::Pose2D base_pose;
ros::Publisher local_map_pub;

Mat global_map,local_map;

const int glo_width=2600;//global map size
const int glo_height=2600;
const float glo_map_res=0.05;

const int radius_in_cm = 60;     //radius around on object to be in safe limit for path-planning
const int lane_in_cm = 30;       //radius around lane for path planning
const int side_in_cm = 30;
const int base_lidar_dist = 50;   //distance from vehicle centre to lidar in cm(only in y-direction)
const int width = 500;//map size
const int height = 250;
const int range = 200;//10m in cells

float map_res = 0.05;//local map resolution(m/cell)
float radius_in_cells;
float minor_axis_in_cells = (lane_in_cm*0.01)/map_res;//elliptically blowing lanes
float major_axis_in_cells = minor_axis_in_cells + 3;
float base_lidar_dist_in_cells = (base_lidar_dist*0.01)/map_res;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
   base_pose.x = msg->pose.pose.position.x;
   base_pose.y = msg->pose.pose.position.y;

   tf::Quaternion quat(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
   tf::Matrix3x3 matrix(quat);
   double roll,pitch,yaw;
   matrix.getRPY(roll,pitch,yaw);

   base_pose.theta = yaw;
}

void create_local()
{
   local_map = Mat::zeros(height,width,CV_8UC1);
   float loc_x,loc_y,glob_x,glob_y;	
   for(int i=0;i<range;i++)
   {
      for(int j=(width/2 - range);j<(width/2 + range);j++)
      {
         loc_x = i*map_res;
         loc_y = ((width/2)-j)*map_res;
         glob_x = base_pose.x + loc_x*cos(base_pose.theta) - loc_y*sin(base_pose.theta);
         glob_y = base_pose.y + loc_x*sin(base_pose.theta) + loc_y*cos(base_pose.theta);

         int x_coord = round(glob_x/glo_map_res);
         int y_coord = round(glob_y/glo_map_res);
         x_coord = x_coord + glo_height/2;
         y_coord = glo_width/2 - y_coord;
            
         if(x_coord>=0 && x_coord<glo_height)
         {
            if(y_coord>=0 && y_coord<glo_width)
            {
               local_map.at<uchar>(i,j) = global_map.at<uchar>(x_coord,y_coord);
            }
         }
      }
   }
   imshow("local_map",local_map);
   waitKey(1);
}

void blow_map()
{
   nav_msgs::OccupancyGrid blown_grid_map;
   blown_grid_map.header.stamp = ros::Time::now();
   blown_grid_map.header.frame_id = "/lidar";
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

   int i,j,k,l;
   for(i=0;i<height;i++)//initialising map
   {
      for(j=0;j<width;j++)
      {
         blown_grid_map.data.push_back(0);
      }
   }

   for(i=0;i<height;i++)//creating blown map
   {
      for(j=0;j<width;j++)
      {
         if(local_map.at<uchar>(i,j)==127)//blowing obstacle data
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
                           blown_grid_map.data[x*width+y+width/2] = 1;
                        }
                     }
                  }
               }
            }
         }
         
         if(local_map.at<uchar>(i,j)==255)//blowing lane data
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
                           blown_grid_map.data[x*width+y+width/2] = 2;
                        }
                     }
                  }
               }
            }
         }

      }
   }
   local_map_pub.publish(blown_grid_map);
   Mat blown_local_map = Mat::zeros(height,width,CV_8UC1);
   //Mat map = Mat::zeros(height,width,CV_8UC1);
   for(i=0;i<height;i++)
   {
     for(j=0;j<width;j++)
     {
       int i_= height - i;
       blown_local_map.at<uchar>(i,j) = blown_grid_map.data[i_*width+j]*0.5*255;
       //map.at<uchar>(i,j) = local_map.at<uchar>(i_,j)*0.5*255;
     }
   }
   imshow("global_map",global_map);
   //imshow("local_map",local_map);
   imshow("blown_local_map",blown_local_map);
   waitKey(1);


}

int main(int argc,char** argv)
{
   ros::init(argc,argv,"map_from_global");
   ros::NodeHandle n,n1;
   ros::Subscriber odom_sub=n.subscribe<nav_msgs::Odometry>("/odometry/fil_map",1,odomCallback);
   //ros::Subscriber global_map_sub=n.subscribe<nav_msgs::OccupancyGrid>("global_map",1,mapCallback);
   local_map_pub=n1.advertise<nav_msgs::OccupancyGrid>("local_map",1);

   global_map = imread("global.jpg");

   namedWindow("global_map",CV_WINDOW_NORMAL);
   namedWindow("local_map",CV_WINDOW_NORMAL);
   namedWindow("blown_local_map",CV_WINDOW_NORMAL);
   
   ros::Rate loop_rate(10.0);

   while(ros::ok())
   {
   	   //local_map = Mat::zeros(height,width,CV_8UC1);
   	   ros::spinOnce();
   	   create_local();
   	   blow_map();
         loop_rate.sleep();
   }	

}