//ros libraries
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "gps_common/conversions.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/NavSatFix.h"
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"
//c++ libraries
#include <iostream>
#include <vector>
#include <utility>
#include <math.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2D.h>

using namespace std;


ros::Publisher local_goal_pub;
geometry_msgs::PoseStamped local_goal;

const float pi = 3.14159265;
const float RAD_TO_DEG = 180.000000/pi;
const float DEG_TO_RAD = pi/180.000000;


float car_length = 4;   //with padding
float car_breadth = 2;  //with padding

float map_res = 0.3;
float map_breadth = 0 ;
int map_breadth_count ;
float map_height = 0;
int map_height_count ;

int length_count; //= car_length/map_res;
int breadth_count; //= car_breadth/map_res;

int goal_pointer;
int goal_shadow, goal_shadow_pos=6;


int L_end ,R_end;
int bounds[12] ={ 0}; //{ 0,(map_breadth_count-1)*map_height_count/2-1,0,0,map_breadth_count/2-1,0,map_breadth_count-1,0,map_breadth_count*map_height_count/2-1,0,0} ;

int search_dir;

//double des_lat[3] = {19.1296337667,0,0};//{19.1295804183};//19.132710; 19.1327367167     //Define Required GPS location // 1st left 19.1296025017
//double des_lon[3] = {72.917449405,0,0};//{72.91613313};//72.915509;  72.9159374433 // first left 72.9172153867
double des_xpos=0;
double des_ypos=0;

float stop_xpos_inside = 0;
float stop_ypos_inside = 0;

double curr_lat=0;
double curr_lon=0;
double curr_xpos=0;
double curr_ypos=0;

double dist_x=0;
double dist_y=0;
double dist_r=0;

bool map_received(false), goal_set(false), currhead_received(false);
float curr_head   = 0;
float last_head   = 0;
float des_head = 0;

string zone;
string data;

std_msgs::Int32MultiArray arr;
int traffic_sign[4];
vector<float> vehicle_data;
void copy_arr(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	int i = 0;
	for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		traffic_sign[i] = *it;
		i++;
	}

return;
} 
void copy_arr_CR(const std_msgs::Float64MultiArray::ConstPtr& A)
{
	int i = 0;
	for(std::vector<double>::const_iterator it = A->data.begin(); it != A->data.end(); ++it)
		vehicle_data.push_back(*it);
}
int creep_and_crawl()
{
	int x=vehicle_data[0];
	int y=vehicle_data[1];
	return (map_height_count*(y-2)+x);
}
int local_goal_set(int goal_shadow_pos, int* sign, nav_msgs::OccupancyGrid map)
{
	if(map_received)
	{
			/*if(sign[3]==1 && sign[2]==0)
		    {
		    	//cout<<sign[0]<<" "<<sign[1]<<" "<<sign[2]<<" "<<sign[3]<<endl;
		    	//cout<<"Right blocked "<<endl;
		    	for(int i=bounds[5];i<=bounds[7];i++)
		    	{
                    map.data[i]=1;
		    	}
                for(int i=0;i<map_height_count;i++)
                {
                    map.data[(i+1)*map_breadth_count-1]=1;
                }
		    }
		    if(sign[3]==1 && sign[0]==0)
		    {
		    	//cout<<sign[0]<<" "<<sign[1]<<" "<<sign[2]<<" "<<sign[3]<<endl;
		    	//cout<<"Left blocked "<<endl;
		    	for(int i=(bounds[5]-1);i>=0;i--)
		    	{
		    		map.data[i]=1;
		    	}
		    	for(int i=0;i<map_height_count;i++)
		    	{
		    		map.data[i*map_breadth_count]=1;
		    	}
		    }*/
			if(goal_shadow_pos>5)
		{
			cout<<">5\n";
			int task = 0;
			int breadth_count_=0;
			int i = 0;
			//////////////////TASK-1--------------------
			while(1)
			{
				search_dir = 1;
				if(bounds[goal_shadow_pos-1]<map_breadth_count-1)
				{
					if(map.data[bounds[goal_shadow_pos-1]+i*search_dir] == 0)
					{
						breadth_count_++;
						if(breadth_count_ == breadth_count)
						{
							//cout<<"fe\n";
							//cout<<"7th bound "<<bounds[goal_shadow_pos-1]<<"  "<<i<<" actual out "<< (bounds[goal_shadow_pos-1]+i*search_dir-(int)(breadth_count/2))<<endl;
							return (bounds[goal_shadow_pos-1]+i*search_dir-(int)(breadth_count/2));
							
						}
						
					}
					else
						breadth_count_=0;
					i++;
					if(bounds[goal_shadow_pos-1]+i*search_dir == bounds[goal_shadow_pos+1])
						break;
				}
				else
				{
					if(map.data[bounds[goal_shadow_pos-1]+i*search_dir*map_breadth_count] == 0)
					{

						breadth_count_++;
						//cout<<breadth_count_<<"\n";
						if(breadth_count_ == breadth_count)
						{
							//cout<<i<<"\n";
							//cout<<bounds[goal_shadow_pos-1]<<"9th   bound\n";
							//cout<<bounds[goal_shadow_pos-1]+(i-(int)(breadth_count/2))*search_dir*map_breadth_count<<"\n";
							return (bounds[goal_shadow_pos-1]+(i-(int)(breadth_count/2))*search_dir*map_breadth_count);
						}
					}
					else
						breadth_count_=0;
					i++;
					//cout<<bounds[goal_shadow_pos+1]<<endl;
					if(bounds[goal_shadow_pos-1]+i*search_dir*map_breadth_count == bounds[goal_shadow_pos+1])
					{
						//cout<<"csaca";
						break;
					}
				}
			}
			breadth_count_=0;
			i = 0;
			task++;
			//cout<<task<<endl;
			//////////////////TASK-2--------------------
			while(1)
			{
				search_dir = -1;
				if(bounds[goal_shadow_pos-1]<map_breadth_count-1)
				{
					if(map.data[bounds[goal_shadow_pos-1]+i*search_dir] == 0)
					{
						breadth_count_++;
						if(breadth_count_ == breadth_count)
							return (bounds[goal_shadow_pos-1]+(i-(int)(breadth_count/2)*search_dir));
					}
					else
						breadth_count_=0;
					i++;
					if(bounds[goal_shadow_pos-1]+i*search_dir == bounds[goal_shadow_pos-3])
						break;
				}

				else
				{
					if(map.data[bounds[goal_shadow_pos-1]+i*search_dir*map_breadth_count] == 0)
					{
						breadth_count_++;
						if(breadth_count_ == breadth_count)
							return (bounds[goal_shadow_pos-1]+(i-(int)(breadth_count/2))*search_dir*map_breadth_count);
					}
					i++;
					if(bounds[goal_shadow_pos-1]+i*search_dir*map_breadth_count == bounds[goal_shadow_pos-3])
						break;
				}
			}
			breadth_count_=0;
			i = 0;
			task++;
			//cout<<task<<endl;
			//////////////////TASK-3--------------------
			while(1)
			{
				search_dir = 1;
				if(map.data[bounds[goal_shadow_pos+1]+i*search_dir*map_breadth_count] == 0)
				{
					breadth_count_++;
					if(breadth_count_ == breadth_count)
						return (bounds[goal_shadow_pos+1]+(i-(int)(breadth_count/2))*search_dir*map_breadth_count);
				}
				else
					breadth_count_=0;
				i++;
				if(bounds[goal_shadow_pos+1]+i*search_dir*map_breadth_count == R_end || bounds[goal_shadow_pos+1] == R_end)
					break;
			}
			breadth_count_=0;
			i = 0;
			task++;
			//cout<<task<<endl;
			//////////////////TASK-4--------------------
			search_dir = -1;
			int map_breadth_count_ = 1; 
			while(1)
			{

				if((bounds[goal_shadow_pos-3]+i*search_dir*map_breadth_count_) == 0)
				{
					//cout<<"aefef"<<endl;
					search_dir = 1;
					map_breadth_count_ = map_breadth_count;
					breadth_count_=0;
				}
				if(map.data[bounds[goal_shadow_pos-3]+i*search_dir*map_breadth_count_] == 0)
				{
					breadth_count_++;
					if(breadth_count_==breadth_count)
						return (bounds[goal_shadow_pos-3]+(i-(int)(breadth_count/2))*search_dir*map_breadth_count_);
				}
				else
					breadth_count_=0;
				i++;
				//cout<<i<<"hi"<<endl;
				if(bounds[goal_shadow_pos-3]+i*search_dir == L_end)
					break;
			}

		}


		
		else if(goal_shadow_pos<5)
		{ 
			int breadth_count_=0;
			int i = 0;
			int task=0;
			//////////////////TASK-1--------------------
			while(1)
			{
				search_dir = -1;
				if(bounds[goal_shadow_pos+1]<map_breadth_count && bounds[goal_shadow_pos+1]>0 )
				{
					if(map.data[bounds[goal_shadow_pos+1]+i*search_dir] == 0)
					{
						breadth_count_++;
						if(breadth_count_ == breadth_count)
						{
							//cout<<(bounds[goal_shadow_pos+1])<<endl;
							return (bounds[goal_shadow_pos+1]+i*search_dir+(int)(breadth_count/2));
						}
						
					}
					else
						breadth_count_=0;
					i++;
					if(bounds[goal_shadow_pos+1]+i*search_dir == bounds[goal_shadow_pos-1])
						break;
				}
				else
				{
					search_dir = 1;
					if(map.data[bounds[goal_shadow_pos+1]+i*search_dir*map_breadth_count] == 0)
					{
						breadth_count_++;
						if(breadth_count_ == breadth_count)
						{
							//cout<<"i"<<i<<endl;
							//cout<<(bounds[goal_shadow_pos+1]+(i-(int)(breadth_count/2))*search_dir*map_breadth_count)<<endl;
							return (bounds[goal_shadow_pos+1]+(i-(int)(breadth_count/2))*search_dir*map_breadth_count);
						}
					}
					else
						breadth_count_=0;
					i++;
					if (bounds[goal_shadow_pos+1]+i*search_dir*map_breadth_count == L_end)
						break;
					if(bounds[goal_shadow_pos+1]+i*search_dir*map_breadth_count == bounds[goal_shadow_pos-1])
						break;
				}
			}
			breadth_count_=0;
			i = 0;
			task++;
			//cout<<task<<endl;
			//////////////////TASK-2--------------------
			while(1)
			{
				search_dir = 1;
				if(bounds[goal_shadow_pos+1]<map_breadth_count)
				{
					if(map.data[bounds[goal_shadow_pos+1]+i*search_dir] == 0)
					{
						breadth_count_++;
						if(breadth_count_ == breadth_count)
							return (bounds[goal_shadow_pos+1]+(i-(int)(breadth_count/2))*search_dir);
					}
					else
						breadth_count_=0;
					i++;
					if(bounds[goal_shadow_pos+1]+i*search_dir == bounds[goal_shadow_pos+3])
						break;
				}

				else
				{
					search_dir = -1;
					if(map.data[bounds[goal_shadow_pos+1]+i*search_dir*map_breadth_count] == 0)
					{
						breadth_count_++;
						if(breadth_count_ == breadth_count)
							return (bounds[goal_shadow_pos+1]+(i-(int)(breadth_count/2))*search_dir*map_breadth_count);
					}
					else
						breadth_count_=0;
					i++;
					if(bounds[goal_shadow_pos+1]+i*search_dir*map_breadth_count == bounds[goal_shadow_pos+3])
						break;
				}
			}
			breadth_count_=0;
			i = 0;
			task++;
			//cout<<task<<endl;
			//////////////////TASK-3--------------------
			while(1)
			{
				if(!goal_shadow_pos==0)
				{
					search_dir = 1;
					if(map.data[bounds[goal_shadow_pos-1]+i*search_dir*map_breadth_count] == 0)
					{
						breadth_count_++;
						if(breadth_count_ == breadth_count)
							return (bounds[goal_shadow_pos-1]+(i-(int)(breadth_count/2))*search_dir*map_breadth_count);
					}
					else
						breadth_count_=0;
					i++;
					if(bounds[goal_shadow_pos-1]+i*search_dir == L_end || bounds[goal_shadow_pos-1] == L_end)
						break;
				}
				else
					break;
			}
			breadth_count_=0;
			i = 0;
			task++;
			//cout<<task<<endl;
			//////////////////TASK-4--------------------
			int map_breadth_count_ = 1; 
			while(1)
			{
				search_dir = 1;
				if((bounds[goal_shadow_pos+3]+i*search_dir*map_breadth_count_) == map_breadth_count-1)
				{
					map_breadth_count_ = map_breadth_count;
					breadth_count_=0;
				}
				if(map.data[bounds[goal_shadow_pos+3]+i*search_dir*map_breadth_count_] == 0)
				{
					breadth_count_++;
					if(breadth_count_==breadth_count)
						return (bounds[goal_shadow_pos+3]+(i-(int)(breadth_count/2))*search_dir*map_breadth_count_);
				}
				else
					breadth_count_=0;
				i++;
				if(bounds[goal_shadow_pos+3]+i*search_dir == R_end)
					break;
			}

		}



			/*int k = 1;
							while(breadth_count_ == breadth_count)
							{
								length_count_++;
								breadth_count_ = 0;
								for(int j=0,j<breadth_count,j++)
								{
									if(map.data[bounds[goal_shadow_pos-1]+i*search_dir+k*map_breadth_count-breadth_count+j] == 0)
										breadth_count_++;
								}
								k++;
								if(length_count_ == length_count)*/
	}
	

map_received = false;
}
void goal_shadow_locator(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	//cout<<msg->pose.position.x<<"yo"<<msg->pose.position.y<<" ch1 "<<(map_breadth-map_res)/2<<" ch2 "<<(map_height-map_res)<<"\n";
	// if((msg->pose.position.x==0 && msg->pose.position.y<=(map_height-map_res)/2)||(msg->pose.position.x<=(map_breadth-map_res)/2 && msg->pose.position.y==0))
	// 	goal_shadow_pos = 0;
	// else if(msg->pose.position.x==0 && msg->pose.position.y<=(map_height-map_res) && msg->pose.position.y>(map_height-map_res)/2 )
	// 	goal_shadow_pos = 2;
	// else if(msg->pose.position.x<=(map_breadth-map_res)/2 && msg->pose.position.y==(map_height-map_res))
	// 	goal_shadow_pos = 4;
	// 	//cout<<goal_shadow_pos<<"\n";}
	// else if(msg->pose.position.x<=(map_breadth-map_res) && msg->pose.position.x>(map_breadth-map_res)/2 && msg->pose.position.y==(map_height-map_res))
	// 	goal_shadow_pos = 6;
	// else if(msg->pose.position.x==(map_breadth-map_res) && msg->pose.position.y<=(map_height-map_res) && msg->pose.position.y>(map_height-map_res)/2)
	// 	goal_shadow_pos = 8;
	// else 
	// 	goal_shadow_pos = 10;
	// //cout<<map_height<<endl;
	if((msg->pose.position.x==0 && msg->pose.position.y>=(map_height-map_res)/2 && msg->pose.position.y<=(map_height-map_res)*0.75) ||(msg->pose.position.x<=(map_breadth-map_res)/2 && msg->pose.position.y==(map_height-map_res)/2))
		goal_shadow_pos = 0;
	else if(msg->pose.position.x==0 && msg->pose.position.y<=(map_height-map_res) && msg->pose.position.y>(map_height-map_res)*0.75 )
		goal_shadow_pos = 2;
	else if(msg->pose.position.x<=(map_breadth-map_res)/2 && msg->pose.position.y==(map_height-map_res))
		goal_shadow_pos = 4;
		//cout<<goal_shadow_pos<<"\n";}
	else if(msg->pose.position.x<=(map_breadth-map_res) && msg->pose.position.x>(map_breadth-map_res)/2 && msg->pose.position.y==(map_height-map_res))
		goal_shadow_pos = 6;
	else if(msg->pose.position.x==(map_breadth-map_res) && msg->pose.position.y<=(map_height-map_res) && msg->pose.position.y>(map_height-map_res)*0.75)
		goal_shadow_pos = 8;
	else if(msg->pose.position.x==(map_breadth-map_res) && msg->pose.position.y>=(map_height-map_res)/2 && msg->pose.position.y<=(map_height-map_res)*0.75)
		goal_shadow_pos = 10;
	else if(msg->pose.position.x<=(map_breadth-map_res)/2)
		goal_shadow_pos = 0;
	else if(msg->pose.position.x>=(map_breadth-map_res)/2 && msg->pose.position.x<=(map_breadth-map_res))
		goal_shadow_pos = 10;
	cout<<"x = "<<msg->pose.position.x<<" y = "<<msg->pose.position.x<<endl;
	//goal_shadow_pos = 0;
	//cout<<"shadow "<<goal_shadow_pos<<"\n";

}

void goal_set_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)/////////////////index check
{
	map_received = true;
	nav_msgs::OccupancyGrid custom_map;
	custom_map.info.width = msg->info.width;
	map_breadth_count = msg->info.width;
	custom_map.info.height = msg->info.height;
	map_height_count = msg->info.height; 
	custom_map.info.resolution = msg->info.resolution;
	map_res=msg->info.resolution;
	
	breadth_count=car_breadth/map_res;
	length_count=car_length/map_res;
	for(int i=0;i<map_height_count;i++)//initialising map
   {
      for(int j=0;j<map_breadth_count;j++)
      {
         custom_map.data.push_back(0);
      }
   }
	//bounds = { 0,(map_breadth_count-1)*map_height_count/2-1,0,0,map_breadth_count/2-1,0,map_breadth_count-1,0,map_breadth_count*map_height_count/2-1,0,0};
	bounds[1] = (map_breadth_count)*((int)(map_height_count/2)-1);
	bounds[3] = 0;
	bounds[5] = map_breadth_count/2-1;
	bounds[7] = map_breadth_count-1;
	bounds[9] = map_breadth_count*(int)(map_height_count/2)-1;

	L_end = (map_breadth_count)*(map_height_count-1);
	R_end = map_breadth_count*map_height_count-1;

	bounds[0] = L_end;
	bounds[11] = R_end;
	map_breadth = map_breadth_count*map_res;
	map_height = map_height_count*map_res;
	//cout<<"2\goal_set_mapn";goal_set_map
	//cout<<" <<< "<<map_breadth_count<<" "<<map_height_count<<endl;
	if(!msg->data.empty())
	{	
		int j = 1;
		for(int i=0;i<msg->data.size();i++)
		{	
			//cout<<i<<"\n";
			int t = map_breadth_count*(map_height_count - 1*j)+i%map_breadth_count+1;
			custom_map.data[i] = msg->data[t-1];
			if(i == j*map_breadth_count-1)
			{
				j++;
			}
		}
	}	
	//int m = msg->data[map_breadth_count*map_height_count-1-map_breadth_count];
	//cout<<m<<"sadsad"<<endl;
	//cout<<"1\n";
	//cout<<endl;
	if(vehicle_data.size()==0)
	goal_pointer = local_goal_set(goal_shadow_pos,traffic_sign, custom_map);
	else goal_pointer=creep_and_crawl(); 
	cout<<goal_pointer<<"\n";
	if(!(goal_pointer%map_breadth_count==0))
		local_goal.pose.position.x = ((goal_pointer)%map_breadth_count-1)*map_res;
	else
	{
		//cout<<"check\n";
		local_goal.pose.position.x = map_res;
	}
	local_goal.pose.position.y = map_height-((int)((goal_pointer)/map_breadth_count)-1)*map_res - 2*map_res;
	local_goal.pose.orientation.x = 0;
	local_goal.pose.orientation.y = 0;
	local_goal.pose.orientation.z = 0;
	local_goal.pose.orientation.w = 1;
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

void current_heading(const geometry_msgs::Pose2D::ConstPtr& msg){
	curr_head = msg->theta;
	currhead_received = true;
}
void des_goal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{ 	
	des_xpos = msg->pose.position.x;
	des_ypos = msg->pose.position.y;
}
int main(int argc,char** argv)
{
	ros::init(argc,argv,"local_goal_setter");
    ros::NodeHandle n;
    ros::Subscriber sub3 = n.subscribe("/traffic_direction", 100, copy_arr);
    ros::Subscriber subn = n.subscribe("/vehicle", 100, copy_arr_CR);
    ros::Subscriber map_sub = n.subscribe("/goal_set_map",10,goal_set_map_callback);
    ros::Subscriber coord_sub = n.subscribe("/lane_goal",1,goal_shadow_locator);
    ros::Subscriber des_goal_sub = n.subscribe("/des_goal",1,des_goal);
    local_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1); 
    ros::Subscriber sub   = n.subscribe("/fix", 1, pos_curr);
	ros::Subscriber sub2 = n.subscribe("/imu/HeadingTrue_degree", 1, current_heading);
    local_goal.header.frame_id="/map";

    //cout<<"main\n";
	ros::Rate loop_rate(15.0);
	float planned_distance = 19;
    while(ros::ok())
	{
		dist_x = des_xpos - curr_xpos;
	    dist_y = des_ypos - curr_ypos;
	    dist_r = pow((pow(dist_x,2)+pow(dist_y,2)),0.5); 
	    des_head = calc_angle((des_ypos - curr_ypos), (des_xpos - curr_xpos));
	    stop_ypos_inside = int(dist_r*(cos((curr_head-des_head)*DEG_TO_RAD))/map_res);
        stop_xpos_inside = (map_breadth_count/2)+1 + int((dist_r*sin((curr_head-des_head)*DEG_TO_RAD))/map_res);
        if(dist_r<=(map_height_count-1)*map_res&&dist_r<=(map_breadth_count-1)*map_res&&dist_r>=1*map_res)
        {
          local_goal.pose.position.x = stop_xpos_inside*map_res;
          local_goal.pose.position.y = stop_ypos_inside*map_res;  
        }
		local_goal_pub.publish(local_goal);

		if(local_goal.pose.position.x<0)
		{
			local_goal.pose.position.x = 0;
		}
		if(local_goal.pose.position.y<0)
			local_goal.pose.position.y = 0;

		
		ros::spinOnce();//check for incoming messages
     	loop_rate.sleep(); 
	}
	
	return 0;
}









