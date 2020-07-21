//ros libraries
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int8.h"
#include "gps_common/conversions.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/NavSatFix.h"
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

double ach_accuracy= 1;

float map_res = 0.25;//map resolution(m/cell)
int width = 100;//map size
int height =100;
int range = 10;//10m in cells
int l_width=200,l_height=400;//lane map size
int l_width_new,l_height_new;
float l_res = 0.05; 
Int8 traffic_direc;
nav_msgs::OccupancyGrid goal_set_map;
ros::Publisher goal_set_map_pub;
const int co_number=1;
double des_lat[co_number] = {19.1295974467};//{ 19.1251380983};//{19.1259112833};////{19.1296389633};//{19.1294204367};//{19.1295804183};    //Define Required GPS location //gate 19.1295974467
double des_lon[co_number] = {72.9173579667};//{72.9159026833};//{72.9155033633};//{72.9155408533};////{72.9172522317};//{72.91613313}; // gate 72.9173579667
double des_xpos=0;
double des_ypos=0;
geometry_msgs::PoseStamped goal,des_goal;

float stop_xpos = 0;
float stop_ypos = 0;

double curr_lat=0;   //Current position variables
double curr_lon=0;
double curr_xpos=0;
double curr_ypos=0;

double dist_x=0; // Global distances from the destination to the current location
double dist_y=0;
double dist_r=0;

float curr_head   = 0;
float last_head   = 0;
float des_head = 0;
float map_theta   = 0;

string zone;
string data;

bool final_goal_reached=false,car_start = false,traffic_move = true;
bool map_received(false), goal_set(false), currhead_received(false),lane_coord_recieved(false),curr_loc_recieved(false);

void lane_extenstion(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	if(!msg->data.empty()){
	map_received = true;
	goal_set_map = (*msg);
	width = msg->info.width;
	height = msg->info.height;
	map_res = msg->info.resolution;
}
}
void lane_coord(const nav_msgs::Path::ConstPtr& msg)
{	lane_coord_recieved = true;
    l_width_new = l_width*l_res/map_res;
    l_height_new = l_height*l_res/map_res;
	int no_of_coord = msg->poses.size();
	cout<<"no_of_coord "<<no_of_coord<<endl;
	if(lane_coord_recieved&&map_received)
	{
			if(!msg->poses.empty())
		{
			for(int i=0;i<no_of_coord/2-1;i++)
		  {
		  	cout<<" i "<<i<<endl;
		  	int y1,x1,y2,x2,y0,x0,y_dash,x_dash,y_dash_2,x_dash_2,x_near,y_near,x_far,y_far;
			y0 = 0;x0 = width /2; 
		  	y1=(l_height - msg->poses[i].pose.position.y)*l_res/map_res;
		  	x1=msg->poses[i].pose.position.x*l_res/map_res+(width- l_width_new)/2;
		  	//cout<<" x1"<<x1<<" y1 "<<y1<<endl;
		  	//cout<<" x1_given"<<msg->poses[i].pose.position.x<<" y1_given "<<msg->poses[i].pose.position.y<<endl;
		  	////cout<<" check 1"<<endl;
		  	goal_set_map.data[y1*width+x1] =  80;
		  	if(x1 >= width/2) //1st quadrant
		  	{
		  		if(y1-y0 == 0) x_dash = x0;
		  		else x_dash = (height-1 - y0)*(x1-x0)/(y1-y0)+x0;
			  	if(x1-x0 == 0) y_dash = y0;
			  	else y_dash = (width-1 - x0)*(y1-y0)/(x1-x0)+y0;
			  	if(y_dash >= height-1)
			  	{
			  		y_dash = height-1;
			  	}
			  	if(x_dash >= width-1)
			  	{
			  		x_dash = width-1;
			  	}
		  	}
		  	if(x1 < width/2) //2nd quadrant
		  	{
		  		if(y1-y0==0) x_dash = x0;
		  		else x_dash = (height-1 - y0)*(x1-x0)/(y1-y0)+x0;
			  	if(x1-x0 == 0) y_dash = y0;
			  	else y_dash = (0 - x0)*(y1-y0)/(x1-x0)+y0;
			  	if(y_dash >= height-1)
			  	{
			  		y_dash = height-1;
			  	}
			  	if(x_dash <0)
			  	{
			  		x_dash = 0;
			  	}
		  	}
		  	//cout<<" i + x "<<i+no_of_coord/2<<endl;
		  	y2=(l_height - msg->poses[i+(int)no_of_coord/2].pose.position.y)*l_res/map_res;
		  	x2=msg->poses[i+ (int)no_of_coord/2].pose.position.x*l_res/map_res+(width- l_width_new)/2;
		  	goal_set_map.data[y2*width+x2] =  10;

		  	//cout<<" x_dash "<<x_dash<<" y_dash "<<y_dash<<endl;
		  	if(x2 >= width/2) //1st quadrant
		  	{
		  		if(y2-y0 == 0) x_dash_2 = width -1;
		  		else x_dash_2 = (height-1 - y0)*(x2-x0)/(y2-y0)+x0;
			  	if(x2-x0 == 0) y_dash_2 = y0;
			  	else y_dash_2 = (width-1 - x0)*(y2-y0)/(x2-x0)+y0;
			  	if(y_dash_2 >= height-1)
			  	{
			  		y_dash_2 = height-1;
			  	}
			  	if(x_dash_2 >= width-1)
			  	{
			  		x_dash_2 = width-1;
			  	}
		  	}
		  	if(x2 < width/2) //2nd quadrant
		  	{
		  		if(y2-y0 == 0) x_dash_2 = 0;
		  		else x_dash_2 = (height-1 - y0)*(x2-x0)/(y2-y0)+x0;
			  	if(x2-x0 == 0) y_dash_2 = y0;
			  	else y_dash_2 = (0 - x0)*(y2-y0)/(x2-x0)+y0;
			  	if(y_dash_2 >= height-1)
			  	{
			  		y_dash_2 = height-1;
			  	}
			  	if(x_dash_2 <0)
			  	{
			  		x_dash_2 = 0;
			  	}
		  	}
		  	//cout<<" x2"<<x2<<" y2 "<<y2<<endl;
		  	//cout<<" x2_given"<<msg->poses[i+(int)no_of_coord/2].pose.position.x<<" y2_given "<<msg->poses[i+(int)no_of_coord/2].pose.position.y<<endl;
		  	//cout<<" x_dash_2 "<<x_dash_2<<" y_dash_2 "<<y_dash_2<<endl;
		  	////cout<<"x_dash "<<x_dash<<" y_dash "<<y_dash<<" x_dash_2 "<<x_dash_2<<"y_dash_2 "<<y_dash_2<<endl;
		  	x_near = min(x_dash,x_dash_2);
		  	y_near = min(y_dash,y_dash_2);
		  	x_far = max(x_dash,x_dash_2); 
		  	y_far = max(y_dash,y_dash_2);
		  	////cout << "check 2"<<endl;
		  	////cout<<"x_near "<<x_near<<" x_far "<<x_far<<" y_near "<<y_near<<"y_far "<<y_far<<endl;
		  	//cout<<"width "<<width<<" height "<<height<<endl;
		  	if(x_near>=0&&x_far<width&&y_near>=0&&y_far<height)
		  	{
		  		////cout<<" check 3 "<<endl;
		  		if(x_dash ==0||x_dash_2 ==0)
			  	{
			  		for(int k=0;k<(y_far - y_near);k++)
			  			goal_set_map.data[(y_near+k)*width] =  60;
			  	}
			  	if(x_dash ==width-1||x_dash_2 ==width-1)
			  	{
			  		for(int k=0;k<(y_far - y_near);k++)
			  			goal_set_map.data[(y_near+k)*width+width-1] =  60;
			  	}
			  	if(y_dash ==height-1||y_dash_2 ==height-1)
			  	{
			  		for(int k=0;k<(x_far - x_near);k++)
			  			goal_set_map.data[(x_near+k)+(height-1)*width] =  60;
			  	}	
		  	}
		  	
		  }
		}
	
	}
	if(traffic_direc.data == 1 || traffic_direc.data == 2)
		  	{
			  		if(traffic_direc.data == 1)
			  	{
			  		x_near = width/2; x_far = width - 1;
			  		y_near = 0; y_far = height - 1;
			  	}
			  	else if (traffic_direc.data == 2)
			  	{
			  		x_near = 0; x_far = width/2;
			  		y_near = 0; y_far = height - 1;
			  	} 
			  	if(x_near>=0&&x_far<width&&y_near>=0&&y_far<height)
			  	{
			  		////cout<<" check 3 "<<endl;
			  		if(x_dash ==0||x_dash_2 ==0)
				  	{
				  		for(int k=0;k<(y_far - y_near);k++)
				  			goal_set_map.data[(y_near+k)*width] =  60;
				  	}
				  	if(x_dash ==width-1||x_dash_2 ==width-1)
				  	{
				  		for(int k=0;k<(y_far - y_near);k++)
				  			goal_set_map.data[(y_near+k)*width+width-1] =  60;
				  	}
				  	if(y_dash ==height-1||y_dash_2 ==height-1)
				  	{
				  		for(int k=0;k<(x_far - x_near);k++)
				  			goal_set_map.data[(x_near+k)+(height-1)*width] =  60;
				  	}	
			  	}
		  	}
	cout << "check 4"<<endl;
	lane_coord_recieved = false;
	map_received = false;

  
}

float calc_angle(float dy, float dx){
	float angle(0);
	angle = atan2(dy , dx)*RAD_TO_DEG;
	return angle;
}
void pos_curr(const sensor_msgs::NavSatFix::ConstPtr& info){
	curr_lat = info->latitude;	
	curr_lon = info->longitude;
	////cout<<curr_lat<<" "<<curr_lon<<endl;
	gps_common::LLtoUTM(curr_lat,curr_lon,curr_ypos,curr_xpos,zone);
	curr_loc_recieved = true;
}
void current_heading(const geometry_msgs::Pose2D::ConstPtr& msg){
	curr_head = msg->theta;
	currhead_received = true;
}
void shutdown(std_msgs::Bool msg)
{
	car_start = msg.data;
}
void traffic_dir(std_msgs::Int8 msg)
{
	traffic_direc.data = msg.data;
}
int main(int argc,char** argv)
{
	float planned_distance = 13;
   ros::init(argc,argv,"lane_goal");
   ros::NodeHandle n;
   ros::Subscriber map_sub=n.subscribe<nav_msgs::OccupancyGrid>("scan/local_map",1,lane_extenstion);
   ros::Subscriber coord_sub = n.subscribe("/lane_coord", 5, lane_coord);
   ros::Subscriber traf_dir_sub = n.subscribe("/traffic_direction", 5, traffic_dir);
   
   ros::Subscriber sub   = n.subscribe("/fix", 1, pos_curr);
   ros::Subscriber sub2 = n.subscribe("/imu/HeadingTrue_degree", 1, current_heading);
   goal_set_map_pub=n.advertise<nav_msgs::OccupancyGrid>("/goal_set_map",1);
   ros::Publisher pub4  = n.advertise<std_msgs::Bool>("/newpath_required",1);
   ros::Subscriber shut_sub = n.subscribe<std_msgs::Bool>("/shutdown",1,shutdown);
   ros::Publisher lane_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/lane_goal", 1);
   ros::Publisher des_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/des_goal", 1);
   //ros::Publisher lane_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
   ros::Publisher output_velocity=n.advertise<std_msgs::Bool>("/output_vel",1); 
   gps_common::LLtoUTM(des_lat[0],des_lon[0],des_ypos,des_xpos,zone);

	std_msgs::Bool path_req,output_vel;
	goal.pose.position.x=0;
  	goal.pose.position.y=0;
  	goal.pose.orientation.x=0;
	goal.pose.orientation.y=0;
	goal.pose.orientation.z=0.707;
	goal.pose.orientation.w=0.707;
	goal.header.frame_id="/map";
	path_req.data = true;
	output_vel.data = false;

	int co_iterator = 0;
	int count = 0;
   ros::Rate loop_rate(15.0);

   while(ros::ok())
   {
   		dist_x = des_xpos - curr_xpos;
	    dist_y = des_ypos - curr_ypos;
	    dist_r = pow((pow(dist_x,2)+pow(dist_y,2)),0.5); 
        ////cout<<"curr_ypos "<<curr_ypos;
        ////cout<<"curr_xpos "<<curr_xpos;
        ////cout<<"des_xpos "<<des_xpos;
        ////cout<<"des_ypos "<<des_ypos;
		//cout<<"dist "<<dist_r<<endl;
		if (dist_r < ach_accuracy||final_goal_reached){  path_req.data = false&&car_start;  
                            final_goal_reached = true;
	                        //cout<<"The vehicle is at the destination"<<endl;                                  
	                        }
	    else if(dist_r>=ach_accuracy){path_req.data = true&&car_start;}
	    if(traffic_direc.data == 0) traffic_move = false;
	    else traffic_move = true;
	    path_req.data = path_req.data && traffic_move;
        pub4.publish(path_req);
        des_head = calc_angle((des_ypos - curr_ypos), (des_xpos - curr_xpos));
        map_theta =  curr_head-des_head;
        int y0,x0,y_dash,x_dash;
		y0 = 0;x0 = width /2; 
		////cout<<"map_theta "<<map_theta<<endl;
		////cout<<" sin (map_theta) "<<sin(map_theta*DEG_TO_RAD)<<endl;
        if(sin((90-map_theta)*DEG_TO_RAD)>=0&&cos((90-map_theta)*DEG_TO_RAD)>=0) //1st quadrant
	  	{
	  		x_dash = (height - y0)*(1/tan((90-map_theta)*DEG_TO_RAD))+x0;
		  	y_dash = (width - x0)*tan((90-map_theta)*DEG_TO_RAD)+y0;
		  	////cout<<" 1 x_dash "<<x_dash<<" 1 y_dash "<<y_dash<<endl;
		  	////cout<<" tan() "<<tan((90-map_theta)*DEG_TO_RAD);
		  	if(y_dash >= height)
		  	{
		  		y_dash = height-1;
		  	}
		  	if(x_dash >= width)
		  	{
		  		x_dash = width-1;
		  	}
	  	}
	  	else if(sin((90-map_theta)*DEG_TO_RAD)>0&&cos((90-map_theta)*DEG_TO_RAD)<0) //2nd quadrant
	  	{
	  		x_dash = (height - y0)*(1/tan((90-map_theta)*DEG_TO_RAD))+x0;
		  	y_dash = (0 - x0)*tan((90-map_theta)*DEG_TO_RAD)+y0;
		  	////cout<<" 2 x_dash "<<x_dash<<"  2 y_dash "<<y_dash<<endl;
		  	////cout<<" tan() "<<tan((90-map_theta)*DEG_TO_RAD);
		  	if(y_dash >= height)
		  	{
		  		y_dash = height-1;
		  	}
		  	if(x_dash <0)
		  	{
		  		x_dash = 0;
		  	}
  		}
  		else if (sin((90-map_theta)*DEG_TO_RAD)<=0&&cos((90-map_theta)*DEG_TO_RAD)>=0)
  		{
  			x_dash = (width-1);
  			y_dash = 0;

  		}
  		else 
  		{
  			x_dash = 0;
  			y_dash = 0;
  		}
  		cout<<x_dash*map_res<<" << "<<y_dash*map_res<<endl;
  		/*if(y_dash<0) y_dash =0;
  		if(x_dash<0) x_dash =0;*/
  		//goal.pose.position.x = x_dash;
        //goal.pose.position.y = y_dash;
        goal.pose.position.x = (x_dash)*map_res;
        goal.pose.position.y = (y_dash)*map_res;
        cout<<"distr "<<dist_r<<endl;
        cout<<" lane goal pose x: "<<goal.pose.position.x<<"lane goal pose y: "<<goal.pose.position.y <<endl;
        if(dist_r<(width/2)*map_res&&dist_r<height*map_res) output_vel.data = false;
        else output_vel.data = true;
        if(dist_r <= ach_accuracy) { //1.0                                
                                  if(co_iterator<co_number-1){gps_common::LLtoUTM(des_lat[co_iterator],des_lon[co_iterator],des_ypos,des_xpos,zone);
                                  co_iterator++;
                                  final_goal_reached = false;}
                              }
        des_goal.pose.position.x = des_xpos;
        des_goal.pose.position.y = des_ypos;
        des_goal_pub.publish(des_goal);
        output_velocity.publish(output_vel);
        cout<<"output_vel "<<output_vel<<endl;
        lane_goal_pub.publish(goal);
        goal_set_map_pub.publish(goal_set_map);
  		goal_set_map.data.clear();
  		
     ros::spinOnce();//check for incoming messages
     loop_rate.sleep(); 
    }
   return 0; 
}