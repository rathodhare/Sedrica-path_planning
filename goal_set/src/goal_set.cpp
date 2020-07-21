#include "ros/ros.h"
#include "std_msgs/String.h"
#include "stdio.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "gps_common/conversions.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2D.h>
#include <fstream>
#include "std_msgs/String.h"

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

using namespace std;

const float pi = 3.14159265;
const float RAD_TO_DEG = 180.000000/pi;
const float DEG_TO_RAD = pi/180.000000;

int co_number = 0; //Ensure co_number is not greater than size of des_lat
double des_lat[3] = {19.1294204367,0,0};//{19.1295804183};//19.132710; 19.1327367167     //Define Required GPS location // 1st left 19.1296025017 // main gate 19.1256739383//main aunty19.1252821017//bef 1st left 19.1295853433
double des_lon[3] = {72.9172522317,0,0};//{72.91613313};//72.915509;  72.9159374433 // first left 72.9172153867   // main gate 72.9156287667//main aunty 72.9158177267 // bef 1st left 72.9172280217
double des_xpos=0;
double des_ypos=0;

float stop_xpos = 0;
float stop_ypos = 0;
float stop_xpos_inside = 0;
float stop_ypos_inside = 0;

double curr_lat=0;
double curr_lon=0;
double curr_xpos=0;
double curr_ypos=0;

double dist_x=0;
double dist_y=0;
double dist_r=0;

float curr_head   = 0;
float last_head   = 0;
float des_head = 0;
float map_theta   = 0;

bool final_goal_reached=false,car_start = false;
bool map_received(false), goal_set(false), currhead_received(false);

geometry_msgs::PoseStamped goal;

double ach_accuracy= 5.5;
int width = 160;
int height = 120;
float map_res = 0.25;
float angletochange = 0;
float reduce_velforang = 1;
float dist_to_obs = 10;
float dist_to_frt = 3;
float distmax = 3.0;  //reduce = 1
float distmin = 0.4;  //reduce = alphamin
float max_ang_vel = 120;
float plan_vel = 0;
float max_vel_wobs = 1.6;//1.6
float max_vel_obs = 1.3;//1.4
bool new_path_stats = true;
bool one_back = false;
bool st_obj = false; //false-no obj   true-obs

 ofstream myfile;


string zone;
string data;

geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw)
{
	geometry_msgs::Quaternion q;
        // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	q.w = cy * cr * cp + sy * sr * sp;
	q.x = cy * sr * cp - sy * cr * sp;
	q.y = cy * cr * sp + sy * sr * cp;
	q.z = sy * cr * cp - cy * sr * sp;
	return q;
}

float calc_angle(float dy, float dx){
	float angle(0);
	angle = atan2(dy , dx)*RAD_TO_DEG;
	return angle;
}

void pos_curr(const sensor_msgs::NavSatFix::ConstPtr& info){
	curr_lat = info->latitude;	
	curr_lon = info->longitude;
	//cout<<curr_lat<<" "<<curr_lon<<endl;
		
		gps_common::LLtoUTM(curr_lat,curr_lon,curr_ypos,curr_xpos,zone);
	
}
/*void pos_curr( const geometry_msgs::Pose2D::ConstPtr& msg){
        curr_ypos=msg->y;
        curr_xpos=msg->x;
       
}*/

void current_heading(const geometry_msgs::Pose2D::ConstPtr& msg){
	curr_head = msg->theta;
	currhead_received = true;
}

void shutdown(std_msgs::Bool msg)
{
    car_start = msg.data;
}
void vel_coeff(const nav_msgs::OccupancyGrid::ConstPtr& map_in){
   // reduce = 1;
    width    = map_in->info.width;
    height   = map_in->info.height;
    map_res  = map_in->info.resolution;
    dist_to_obs=5;
}

int main(int argc, char **argv){	
	ros::init(argc, argv, "goal_set");
	float planned_distance = 15;
    myfile.open("gps_distance.txt", ios::trunc);

 
	ros::NodeHandle n;

	 ros::Subscriber sub   = n.subscribe("/fix", 1, pos_curr);
    //ros::Subscriber sub = n.subscribe<geometry_msgs::Pose2D>("utm", 1,pos_curr);
	 ros::Subscriber sub2 = n.subscribe("/imu/HeadingTrue_degree", 1, current_heading);
	 ros::Subscriber sub3  = n.subscribe("/scan/local_map", 1, vel_coeff);
     ros::Subscriber shut_sub = n.subscribe<std_msgs::Bool>("/shutdown",1,shutdown);
	 ros::Publisher pub4  = n.advertise<std_msgs::Bool>("/newpath_required",1);
	 ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1); 
     ros::Publisher output_velocity=n.advertise<std_msgs::Bool>("/output_vel",1); 
	 co_number=0;
     /*des_lat[0]=19.1296250517;
     des_lon[0]=72.917282925;
     des_lat[1]=19.1359035;
     des_lon[1]=72.9046810;
     des_lat[3]=19.13575993;
     des_lon[3]=72.90471378;*/

     gps_common::LLtoUTM(des_lat[0],des_lon[0],des_ypos,des_xpos,zone); 

     std_msgs::Bool path_req,output_vel;
     std_msgs::Bool st_obj_st;
     std_msgs::Float32 v_des;
     std_msgs::Float32 theta_des;
     std_msgs::Float32 ang_des;

     goal.pose.position.x=36.7;
  	 goal.pose.position.y=15.0;
  	 goal.pose.orientation.x=0.0;
	 goal.pose.orientation.y=0.0;
	 goal.pose.orientation.z=0.707;
	 goal.pose.orientation.w=0.707;
	 goal.header.frame_id="/map";

     path_req.data = true;
     v_des.data=0;
     theta_des.data=0; 
     ang_des.data=max_ang_vel;
     st_obj_st.data=false;

        int co_iterator = 0;
        int count = 0;
ros::Rate loop_rate(10);

    while (ros::ok()){

    	//cout<<"Coorde No:: "<<co_iterator<<endl;

	    /*dist_x = des_xpos - curr_xpos;
	    dist_y = des_ypos - curr_ypos;
	    dist_r = pow((pow(dist_x,2)+pow(dist_y,2)),0.5); 
        cout<<"curr_ypos "<<curr_ypos;
        cout<<"curr_xpos "<<curr_xpos;
        cout<<"des_xpos "<<des_xpos;
        cout<<"des_ypos "<<des_ypos;
cout<<"dist "<<dist_r<<endl;
 myfile<<dist_r<<endl;
  
 	    if (dist_r < ach_accuracy||final_goal_reached){  path_req.data = false&&car_start;  
                            final_goal_reached = true;
	                        cout<<"The vehicle is at the destination"<<endl;
                            myfile.close();                                   
	                        }
	    else if(dist_r>=ach_accuracy){path_req.data = true&&car_start;}
        pub4.publish(path_req);
        des_head = calc_angle((des_ypos - curr_ypos), (des_xpos - curr_xpos));
        map_theta =  curr_head-des_head;
        goal.pose.orientation = toQuaternion(0 , 0 , (90-map_theta)*DEG_TO_RAD) ;
        //goal.pose.orientation = toQuaternion(0 , 0 , (90)*DEG_TO_RAD) ;
        cout<<"map_theta "<<map_theta<<"des_head "<<des_head<<"curr_head "<<curr_head<<endl; 
        /*if(dist_x<float(width)*map_res/2&&dist_y<float(height)*map_res)
        {
        	goal.pose.position.x = dist_r*cos(map_theta*DEG_TO_RAD)+float(width)*map_res/2;
        	goal.pose.position.y = dist_r*sin(map_theta*DEG_TO_RAD);
        }
        else
        {
        	goal.pose.position.x = planned_distance*cos(map_theta*DEG_TO_RAD)+float(width)*map_res/2;
        	goal.pose.position.y = planned_distance*sin(map_theta*DEG_TO_RAD); 
        }*/
        //cout<<planned_distance<<endl;
        /*stop_ypos = int(planned_distance*(cos((curr_head-des_head)*DEG_TO_RAD))/map_res);
	 	stop_xpos = (width/2)-1 + int((planned_distance*sin((curr_head-des_head)*DEG_TO_RAD))/map_res);
        stop_ypos_inside = int(dist_r*(cos((curr_head-des_head)*DEG_TO_RAD))/map_res);
        stop_xpos_inside = (width/2)-1 + int((dist_r*sin((curr_head-des_head)*DEG_TO_RAD))/map_res);
	 	if (stop_ypos >= height) stop_ypos = height - 1;
               // }
	 	
	 	else if (stop_ypos < 0) {stop_ypos = 0;}
	 	if (stop_xpos >= width){ stop_xpos = width-1;}
	 	if (stop_xpos < 0) {stop_xpos = 0;}

	 	if(stop_ypos == 0){  // for back standerization
              	goal.pose.orientation = toQuaternion(0 , 0 , (270)*DEG_TO_RAD) ; 
                  if(stop_xpos < width/2){stop_xpos=(width/8);}
                  else stop_xpos = width-1-(width/8);
	 	}
        if(dist_r<=planned_distance)
        {
            if (stop_ypos_inside < 0) {stop_ypos_inside = 0;}
          goal.pose.position.x = stop_xpos_inside*map_res;
        goal.pose.position.y = stop_ypos_inside*map_res;  
        }
	 	else
        {
        goal.pose.position.x = stop_xpos*map_res;
        goal.pose.position.y = stop_ypos*map_res;
        }
         //goal.pose.position.x=(width/2)*map_res;
        //goal.pose.position.y=(height/2)*map_res;
        cout<<"pose x: "<<goal.pose.position.x<<" pose y: "<<goal.pose.position.y <<endl;
        if(dist_r<(width/2)*map_res&&dist_r<height*map_res) output_vel.data = false;
        else output_vel.data = true;
        cout<<"output_vel "<<output_vel<<endl;
        output_velocity.publish(output_vel);*/
        goal_pub.publish(goal);
        cout<<" ch"<<endl;
        if(dist_r <= ach_accuracy) { //1.0                                
                                  if(co_iterator<co_number){gps_common::LLtoUTM(des_lat[co_iterator],des_lon[co_iterator],des_ypos,des_xpos,zone);
                                  co_iterator++;
                                    final_goal_reached = false;}
                              }

        ros::spinOnce();
		loop_rate.sleep();
     }
	return 0;                    
}
