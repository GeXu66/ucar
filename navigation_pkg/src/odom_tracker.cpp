#include <tf/transform_broadcaster.h>
#include "navigation_pkg/common_include.h" 
#include "navigation_pkg/config.h"

Config cfg;
ros::Publisher odom_pub;
ros::Subscriber vel_sub;

double velX = 0;
double velY = 0;
double velAng = 0;

void timer_callback(const ros::TimerEvent &evt, tf::TransformBroadcaster& odom_broadcaster){

	static double x = cfg.globalSttX;
  	static double y = cfg.globalSttY;
  	static double Ang = 0; //cfg.globalSttOri;

	static ros::Time last_time = evt.current_real;
	static ros::Time current_time = evt.current_real;

	current_time = evt.current_real;
 
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (velX * cos(Ang) - velY * sin(Ang)) * dt;
    double delta_y = (velX * sin(Ang) + velY * cos(Ang)) * dt;
    double delta_ang = velAng * dt;
 
    x += delta_x;
    y += delta_y;
    Ang += delta_ang;
 
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Ang);
 
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
 
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
 
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
 
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
 
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
 
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = velX;
    odom.twist.twist.linear.y = velY;
    odom.twist.twist.angular.z = Ang;
 
    //publish the message
    odom_pub.publish(odom);
 
    last_time = current_time;
}

void vel_callback(const geometry_msgs::Twist::ConstPtr &msg){
	velX = msg->linear.x;
	velY = msg->linear.y;
	velAng = msg->angular.z;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "odom_tracker_node");
	ros::NodeHandle nh;

	cfg.set(nh);
	tf::TransformBroadcaster odom_broadcaster;
  	//n.subscribe("chatter", 1000, boost::bind(&chatterCallback,_1,arg1,arg2,...,argN); 
	odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
	vel_sub = nh.subscribe<geometry_msgs::Twist>("/vel_pub", 10, vel_callback);

	ros::Timer odomThread = nh.createTimer(ros::Duration(1.0/cfg.loopHz), boost::bind(&timer_callback, _1, odom_broadcaster));

	ros::AsyncSpinner spinner(0);
	spinner.start();
	ros::waitForShutdown();

	return 0;
}
