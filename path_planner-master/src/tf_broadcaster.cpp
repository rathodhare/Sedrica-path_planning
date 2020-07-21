//###################################################
//                        TF MODULE FOR THE HYBRID A*
//###################################################
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
// map pointer
nav_msgs::OccupancyGridPtr grid;
double x,y,z,w;
float t =0,initial_t=0;
bool initial_t_value(true);
const float pi = 3.14159265;
const float RAD_TO_DEG = 180.000000/pi;
const float DEG_TO_RAD = pi/180.000000;
float start_x = 0;
// map callback
void setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  std::cout << "Creating transform for map..." << std::endl;
  grid = map;
  start_x = map->info.width/2*map->info.resolution;
  //cout << start_x<<endl;
}

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
void current_heading(const sensor_msgs::Imu::ConstPtr& msg){
 t = tf::getYaw(msg->orientation);
 if(initial_t_value)
 {
  initial_t = t;
  initial_t_value=false;
 }
 t=t - initial_t+90*DEG_TO_RAD ;
 geometry_msgs::Quaternion q = toQuaternion(0,0,t);
x=q.x;
y=q.y;
z=q.z;
w=q.w;

}
int main(int argc, char** argv) {
  // initiate the broadcaster
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle n;

  // subscribe to map updates
  ros::Subscriber sub_map = n.subscribe("/occ_map", 1, setMap);
  ros::Subscriber sub2 = n.subscribe("/imu/data", 1, current_heading);
  tf::Pose tfPose;


  ros::Rate r(100);
  tf::TransformBroadcaster broadcaster;

  while (ros::ok()) {
    // transform from geometry msg to TF
    if (grid != nullptr) {
      tf::poseMsgToTF(grid->info.origin, tfPose);
    }

    // odom to map
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tfPose.getOrigin()),
        ros::Time::now(), "odom", "map"));

    // map to path
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
        ros::Time::now(), "map", "path"));
    ros::spinOnce();
   broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.0, 0.0, 0.707,0.707), tf::Vector3(start_x, 0, 0)),
        //tf::Transform(tf::Quaternion(x, y, z,w), tf::Vector3(20, 2, 0)),
        //tf::Transform(tf::Quaternion(0, 0, 0,1), tf::Vector3(0, 0, 0)),
        ros::Time::now(), "map", "base_link"));
    r.sleep();
  }
}
