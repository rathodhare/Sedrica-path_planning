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
//opencv libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

const float res = 0.25;          //angular resolution of lidar in degrees
const int num_values = 1080;     //no. of rays emitted by the lidar

const int radius_in_cm = 60;     //radius around on object to be in safe limit for path-planning
const int lane_in_cm = 40;       //radius around lane for path planning
const int side_in_cm = 30;
const int base_lidar_dist = -30;   //distance from vehicle centre to lidar in cm(only in y-direction)

const int cam_lidar_dist = -90;    //distance from camera to lidar in cm

const float map_res = 0.05;//map resolution(m/cell)
const int width_dist = 24;
const int height_dist = 24;
const int width = width_dist/map_res;//map size
const int height = height_dist/map_res;
const int range = 10/map_res;//10m in cells
int l_width,l_height;//lane map size
int r_width,r_height;//road map size
const int obstacle_cost = 100,lane_cost = 80;
const float final_map_res = 0.3;          
const int final_width = width_dist/final_map_res;//map size
const int final_height = height_dist/final_map_res;

int w1=18,w2=14; //w1 = obstacle_cot , w2 = lane_cost
float obs_wgt=0.5,lane_wgt=0.5;
ros::Publisher map_pub,blown_map_pub,final_cost_map_pub;
std_msgs::Int8MultiArray lane_map,road_map;

pair<float,float> convToCart(int i,float r) //convert from polar to cartesian co-ordinates
{
   float ang = i*0.25*(3.14/180);
   float x = r*cos(ang);
   float y = -1*r*sin(ang);
   return make_pair(x,y);
}

void laneCallback(nav_msgs::OccupancyGrid msg)
{
   int i,j;
   l_width = msg.info.width;
   l_height = msg.info.height;
   //cout<<l_width<<" "<<l_height<<endl;
   for(i=0;i<l_height;i++)//initialising map
   {
      for(j=0;j<l_width;j++)
      {
         lane_map.data.push_back(0);
      }
   }
   
   for(i=l_height-1;i>=0;i--)//initialising map
   {
      for(j=0;j<l_width;j++)
      {
         int i1;
         i1=l_height-1-i;

         lane_map.data[i1*l_width+j] = msg.data[i*l_width+j];
      }
   }    
}
void roadCallback(nav_msgs::OccupancyGrid msg)
{
   int i,j;
   r_width = msg.info.width;
   r_height = msg.info.height;
   //cout<<r_width<<" "<<r_height<<endl;
   
   for(i=r_height-1;i>=0;i--)//initialising map
   {
      for(j=0;j<r_width;j++)
      {
         int i1;
         i1=r_height-1-i;
         if(msg.data[i*r_width+j]==2)
            lane_map.data[i1*r_width+j] = msg.data[i*r_width+j];
      }
   }    
}
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
   
   float radius_in_cells;
   float minor_axis_in_cells = (lane_in_cm*0.01)/map_res;//elliptically blowing lanes
   float major_axis_in_cells = minor_axis_in_cells + 3;
   float base_lidar_dist_in_cells = (base_lidar_dist*0.01)/map_res;
   float base_cam_dist_in_cells = ((  base_lidar_dist - cam_lidar_dist )*0.01)/map_res;
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

   nav_msgs::OccupancyGrid grid_map_cost;
   grid_map_cost.header.stamp = ros::Time::now();
   grid_map_cost.header.frame_id = "/map";
   grid_map_cost.info.resolution = map_res;
   grid_map_cost.info.origin.position.x = 0.0;
   grid_map_cost.info.origin.position.y = 0.0;
   grid_map_cost.info.origin.position.z = 0.0;
   grid_map_cost.info.origin.orientation.x = 0.0;
   grid_map_cost.info.origin.orientation.y = 0.0;
   grid_map_cost.info.origin.orientation.z = 0.0;
   grid_map_cost.info.origin.orientation.w = 1.0;
   grid_map_cost.info.width = width;
   grid_map_cost.info.height = height;
   grid_map_cost.info.map_load_time = ros::Time::now();

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

   nav_msgs::OccupancyGrid final_grid_map;
   final_grid_map.header.stamp = ros::Time::now();
   final_grid_map.header.frame_id = "/map";
   final_grid_map.info.resolution = final_map_res;
   final_grid_map.info.origin.position.x = 0.0;
   final_grid_map.info.origin.position.y = 0.0;
   final_grid_map.info.origin.position.z = 0.0;
   final_grid_map.info.origin.orientation.x = 0.0;
   final_grid_map.info.origin.orientation.y = 0.0;
   final_grid_map.info.origin.orientation.z = 0.0;
   final_grid_map.info.origin.orientation.w = 1.0;
   final_grid_map.info.width = final_width;
   final_grid_map.info.height = final_height;
   final_grid_map.info.map_load_time = ros::Time::now();

   nav_msgs::OccupancyGrid final_grid_map_cost;
   final_grid_map_cost.header.stamp = ros::Time::now();
   final_grid_map_cost.header.frame_id = "/map";
   final_grid_map_cost.info.resolution = final_map_res;
   final_grid_map_cost.info.origin.position.x = 0.0;
   final_grid_map_cost.info.origin.position.y = 0.0;
   final_grid_map_cost.info.origin.position.z = 0.0;
   final_grid_map_cost.info.origin.orientation.x = 0.0;
   final_grid_map_cost.info.origin.orientation.y = 0.0;
   final_grid_map_cost.info.origin.orientation.z = 0.0;
   final_grid_map_cost.info.origin.orientation.w = 1.0;
   final_grid_map_cost.info.width = final_width;
   final_grid_map_cost.info.height = final_height;
   final_grid_map_cost.info.map_load_time = ros::Time::now();
   
   vector< pair<float,float> > cartesian(num_values,make_pair(0,0));
   for(i=179;i<num_values-180;i++)
   { //if(msg->intensities[i-180]>=400)
      {
         //cartesian[i] = convToCart((i-num_values/2),msg->ranges[(i-180)]);
         cartesian[i] = convToCart((i-num_values/2),msg->ranges[num_values - (i)]);
         cartesian[i].first += base_lidar_dist*0.01;  
      }
      
   }
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
         grid_map_cost.data.push_back(0);
      }
   }
   for(i=0;i<final_height;i++)//initialising map
   {
      for(j=0;j<final_width;j++)
      {
         final_grid_map.data.push_back(0);
         final_grid_map_cost.data.push_back(0);
      }
   }
   //cout<<"hi"<<endl;
    for(i=0;i<l_height;i++)//adding lane data 
   {
      for(j=0;j<l_width;j++)
      {
         if(i >=0 && i <= range)//limiting obstacle detection to only 10m
      {
         if(j - l_width/2 <= range && j - l_width/2 >= -range)
         {
            if(lane_map.data[(i)*l_width+j]==2)
               grid_map.data[(i+base_cam_dist_in_cells)*width+j+width/2-l_width/2] = lane_cost;
            } 
         }
      }
   }
   for(i=0;i<num_values;i++)
   {
      int x_coord = round(cartesian[i].first/map_res);
      int y_coord = round(cartesian[i].second/map_res);
      /*if(x_coord==0 && y_coord==0)
      {
        cout<<i<<" "<<msg->ranges[i];
      }*/
      if(x_coord >0 && x_coord <= range)//limiting obstacle detection to only 10m
      {
         if(y_coord <= range && y_coord >= -range)
         {
            if(!(x_coord==base_lidar_dist_in_cells && y_coord==0))
            grid_map.data[x_coord*width+y_coord+width/2] =  obstacle_cost;
         } 
      }
   }
   //int x_ = round((cartesian[540].first + base_lidar_dist)/map_res);
   //int y_ = round(cartesian[540].second/map_res);
   //int a=blown_grid_map.data[400];
   //int b=grid_map.data[400];
   //cout<<a<<" "<<b<<endl;
   //grid_map.data = map.data;
   
  

  /* for(i=0;i<height;i++)//creating blown map
   {
      for(j=0;j<width;j++)
      {
         if(grid_map.data[i*width+j]==obstacle_cost)//blowing obstacle data
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
                           blown_grid_map.data[x*width+y+width/2] = obstacle_cost;
                        }
                     }
                  }
               }
            }
         }
         
         if(grid_map.data[i*width+j]==lane_cost)//blowing lane data
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
                           blown_grid_map.data[x*width+y+width/2] = lane_cost;
                        }
                     }
                  }
               }
            }
         }

      }
   }  */ 
   
   Mat final_local_map = Mat::zeros(final_height,final_width,CV_8UC1);
   Mat final_local_map_cost = Mat::zeros(height,width,CV_8UC1);
   Mat local_map = Mat::zeros(height,width,CV_8UC1);
   Mat blown_local_map = Mat::zeros(height,width,CV_8UC1);
   //Mat mapImg1 = Mat::zeros(height, width, CV_8UC1);
   //Mat mapImg2 = Mat::zeros(height, width, CV_8UC1);
   Mat mapImgn = Mat::zeros(height, width, CV_8UC1); //obstacle matrix
   Mat mapImgm = Mat::zeros(height, width, CV_8UC1); //lane matrix
   Mat cost1 = Mat::zeros(height, width, CV_8UC1); //obstacle matrix
   Mat cost2 = Mat::zeros(height, width, CV_8UC1); //lane matrix
   for(i=0;i<height;i++)
   {
     for(j=0;j<width;j++)
     {
       int i_= height - i;
       local_map.at<uchar>(i,j) = grid_map.data[i_*width+j]*2.55;
       blown_local_map.at<uchar>(i,j) = blown_grid_map.data[i_*width+j]*2.55;
       if (grid_map.data[i_*width+j] == obstacle_cost){
                  mapImgn.at<uchar>(i , j) = 0;
               }
               else mapImgn.at<uchar>(i , j) = 200;
       if (grid_map.data[i_*width+j] == lane_cost){
                  mapImgm.at<uchar>(i , j) = 0;
               }
               else mapImgm.at<uchar>(i , j) = 200;
       //local_map.at<Vec3b>(i,j)[1] = map.data[i*width+j]*255;
       //local_map.at<Vec3b>(i,j)[2] = map.data[i*width+j]*255;
     }
   }
   for(i=0;i<final_height;i++)
   {
     for(j=0;j<final_width;j++)
     {
       int i_= final_height - i;
       final_local_map.at<uchar>(i,j) = final_grid_map.data[i_*final_width+j]*0.5*255;
     }
   }

   distanceTransform (mapImgn, mapImgn, CV_DIST_L2, 5);
   distanceTransform (mapImgm, mapImgm, CV_DIST_L2, 5);
   normalize (mapImgn, mapImgn, 0.0, 1.0, NORM_MINMAX);
   normalize (mapImgm, mapImgm, 0.0, 1.0, NORM_MINMAX);
   subtract(1,mapImgm,mapImgm);
   subtract(1,mapImgn,mapImgn);
   lane_wgt = (float)w2/100.0 ; obs_wgt = (float)w1/100.0;
   addWeighted( mapImgn, obs_wgt, mapImgm, lane_wgt, 0.0, final_local_map_cost);
   //final_local_map_cost.convertTo(final_local_map_cost, CV_8UC1, 255, 0);
      //copying matrix to occupancy grid
   for(i=0;i<height;i++)
   {
     for(j=0;j<width;j++)
     {
       int i_= height - i;
        grid_map_cost.data[i_*width+j] = (final_local_map_cost.at<float>(i,j))*100;
     }
   }
   //cout<<"end  copy cost map"<<endl;
   imshow("final_local_map_cost",final_local_map_cost);
   imshow("final_local_map",final_local_map);
   imshow("local_map",local_map);
   imshow("blown_local_map",blown_local_map);
   imshow("mapImg obstacle",mapImgn);
   imshow("mapImg lane",mapImgm);
   waitKey(1);
   //cout<<"imshow"<<endl;
   for(i=0;i<height;i++)//initialising map
   {
      for(j=0;j<width;j++)
      {
         if(grid_map.data[i*width+j]!=0)
            final_grid_map.data[(int)(i*map_res/final_map_res)*final_width+(int)(j*map_res/final_map_res)]=grid_map.data[i*width+j];
         if(grid_map_cost.data[i*width+j]!=0)
            final_grid_map_cost.data[(int)(i*map_res/final_map_res)*final_width+(int)(j*map_res/final_map_res)]=grid_map_cost.data[i*width+j];
      }
   }
   map_pub.publish(final_grid_map);
   blown_map_pub.publish(grid_map);
   final_cost_map_pub.publish(final_grid_map_cost);
   final_grid_map.data.clear();
   final_grid_map_cost.data.clear();
   lane_map.data.clear();
   grid_map.data.clear();
   grid_map_cost.data.clear();
   blown_grid_map.data.clear();
}

int main(int argc,char** argv)
{
   ros::init(argc,argv,"local_map");
   ros::NodeHandle n,n1,n2,n3;
   ros::Subscriber lane_sub=n.subscribe<nav_msgs::OccupancyGrid>("/Lane_Occupancy_Grid",1,laneCallback);
   ros::Subscriber road_sub=n.subscribe<nav_msgs::OccupancyGrid>("/Road_Occupancy_Grid",1,roadCallback);
   ros::Subscriber lidar_sub=n1.subscribe<sensor_msgs::LaserScan>("scan",1,lidarCallback);
   map_pub=n2.advertise<nav_msgs::OccupancyGrid>("scan/local_map",1);
   final_cost_map_pub=n2.advertise<nav_msgs::OccupancyGrid>("scan/cost_map",1);
   blown_map_pub=n3.advertise<nav_msgs::OccupancyGrid>("scan/blown_local_map",1);

   namedWindow("final_local_map",CV_WINDOW_NORMAL);   
   namedWindow("final_local_map_cost",CV_WINDOW_NORMAL); 
   namedWindow("local_map",CV_WINDOW_NORMAL);
   namedWindow("blown_local_map",CV_WINDOW_NORMAL);
   namedWindow("mapImg obstacle",CV_WINDOW_NORMAL);
   namedWindow("mapImg lane",CV_WINDOW_NORMAL);
   createTrackbar("lane_c", "final_local_map_cost", &w2, 100);
   createTrackbar("obst_o", "final_local_map_cost", &w1, 100);

   ros::Rate loop_rate(15.0);

   while(ros::ok())
   {
     ros::spinOnce();//check for incoming messages
     loop_rate.sleep(); 
   }
   return 0; 
}
