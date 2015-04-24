#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <fstream>
#include <iostream>
#include <ros::geometry_msgs/Pose.h>
#include <ros::geometry_msgs/Wrench.h>
#include <ros::sensor_msgs/JointState.h>
#include <ros::std_msgs/Bool.h>
#include <vector>
#include <cmath>

using namespace std;
using namespace ros;

//callback for AR tag location
void ARtag_callback(const geometry_msgs::Pose& msg)
{
	//code in here for getting the transform to the tag on the table
}

double tool_position[2] = {0.0;0.0}; //tool_position global variable

//callback for tool_pose location
void tool_pose_callback(const geometry_msgs::Pose& msg)
{
	tool_pose.position.x = msg.position.x; 
	tool_pose.position.y = msg.position.y;
}

//calculate error of end-effector to closest point on path
void error_calc()
{
	//find closest point to end-effector
	//determine which of the two lines is closest
	//determine closest point
	//calculate error vector {dx,dy,0,0,0,0}
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "magic_node");
	ros::NodeHandle nh;

	ROS_INFO("Creating Publisher and Subscriber");

	//subscribe to end-effector pose
	ros::Subscriber tool_pose_sub = nh.subscribe("/dvrk_psm1/joint_position_cartesian", 1, tool_pose_callback);
	//subscribe to AR tag 
	ros::Subscriber ARtag_sub = nh.subscribe("something for AR tag", 1, ARtag_callback);
	
	//publisher for force vector
	ros::Publisher w_pub = nh.advertise<geometry_msgs::Wrench>("/dvrk_mtmr/set_wrench_static", 1);

	//publisher to enable torque mode
	ros::Publisher torque_mode_pub = nh.advertise<std_msgs::Bool>("/dvrk_mtmr/enable_torque_mode",1);

	ros::Rate loop_rate(50);

	//vector of vectors, assuming this is what we'd 
	vector<vector<double>> points(10, vector<double>(3));

	/**Need help with file parsing..
	 * and putting the data into the vector of vectors
	 * 
	 * 
	 * 
	**/

	double P_gain = .5 * 100; //.5 @ 1cm offset
	double buffer_zone = 10.0; // 1cm
	double error_vec[6] = {0;0;0;0;0;0};
	double error_mag = 0.0;
	geometry_msgs::Pose tool_pose;
	geometry_msgs::Wrench force_wrench = {0;0;0;0;0;0};
	std_msgs::Bool bool_true = true;

	while (ros::ok())
	{	

		error_mag = sqrt((error_vec[0]*error_vec[0]) + (error_vec[1]*error_vec[1]));
		if (error_mag >= buffer_zone){
			force_wrench[0] = P_gain / error_vec[0];
			force_wrench[1] = P_gain / error_vec[1];
		} else {	//within buffer zone
			force_wrench[0] = P_gain * error_vec[0];
			force_wrench[1] = P_gain * error_vec[1];
		}

		//enable torque mode, just in case (only actually necessary after Mono is released, but it doesn't hurt)
		torque_mode_pub.publish(bool_true);
			
		//send the joint state and transform
		w_pub.publish(force_wrench);

		//sleep
		loop_rate.sleep();
	}
	
	return 0;
}
