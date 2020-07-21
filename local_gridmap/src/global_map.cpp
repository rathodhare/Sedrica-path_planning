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

const int width=2600;
const int height=2600;
const float map_res=0.05;

geometry_msgs::Pose2D base_pose;
nav_msgs::OccupancyGrid grid_map;
ros::Publisher global_map_pub;

Mat global_map;

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

void mapCallback(nav_msgs::OccupancyGrid msg)
{
   //cout<<"hi";
   int loc_width = msg.info.width;
   int loc_height = msg.info.height;
   float loc_map_res = msg.info.resolution;
   float loc_x,loc_y,glob_x,glob_y;

   for(int i=0;i<loc_height/5;i++)
   {
      for(int j=0;j<loc_width;j++)
      {
         if(msg.data[i*loc_width+j] > 0)
         {
            loc_x = i*loc_map_res;
            loc_y = ((width/2)-j)*loc_map_res;
            glob_x = base_pose.x + loc_x*cos(base_pose.theta) - loc_y*sin(base_pose.theta);
            glob_y = base_pose.y + loc_x*sin(base_pose.theta) + loc_y*cos(base_pose.theta);

            int x_coord = round(glob_x/map_res);
            int y_coord = round(glob_y/map_res);
            x_coord = x_coord + height/2;
            y_coord = width/2 - y_coord;
            
            if(x_coord>=0 && x_coord<height)
            {
               if(y_coord>=0 && y_coord<width)
               {
                  grid_map.data[x_coord*width+y_coord]=msg.data[i*loc_width+j];
               }
            }
         }
      }
   }

   global_map=Mat::zeros(height,width,CV_8UC1);
   for(int i=0;i<height;i++)
   {
      for(int j=0;j<width;j++)
      {
         global_map.at<uchar>(i,j)=grid_map.data[i*width+j]*0.5*255;
      }
   }
   imshow("globalmap",global_map);
   //imwrite("global.jpg",global_map);
   waitKey(1);
}

int main(int argc,char** argv)
{
   ros::init(argc,argv,"global_map");
   ros::NodeHandle n,n1;
   ros::Subscriber odom_sub=n.subscribe<nav_msgs::Odometry>("/odometry/fil_map",1,odomCallback);
   ros::Subscriber map_sub=n.subscribe<nav_msgs::OccupancyGrid>("scan/local_map",1,mapCallback);
   global_map_pub=n1.advertise<nav_msgs::OccupancyGrid>("global_map",1);

   grid_map.header.stamp = ros::Time::now();
   grid_map.header.frame_id = "/global_map";
   grid_map.info.resolution = map_res;
   grid_map.info.origin.position.x = -1*(height/2)*map_res;
   grid_map.info.origin.position.y = (width/2)*map_res;
   grid_map.info.origin.position.z = 0.0;
   grid_map.info.origin.orientation.x = 0.0;
   grid_map.info.origin.orientation.y = 0.0;
   grid_map.info.origin.orientation.z = 0.0;
   grid_map.info.origin.orientation.w = 1.0;
   grid_map.info.width = width;
   grid_map.info.height = height;
   grid_map.info.map_load_time = ros::Time::now();
   for(int i=0;i<height;i++)//initialising map
   {
      for(int j=0;j<width;j++)
      {
         grid_map.data.push_back(0);
      }
   }
   
   namedWindow("globalmap",CV_WINDOW_NORMAL);
   ros::Rate loop_rate(10.0);

   while(ros::ok())
   {
      //global_map=Mat::zeros(height,width,CV_8UC1);
      ros::spinOnce();
      loop_rate.sleep();
   }
   if(!ros::ok())
   { 
     imwrite("global.jpg",global_map);
     return 0;
   }
}
