#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <vector>
#include <cmath>

using namespace std;
using namespace ros;

typedef sensor_msgs::JointState* JointStatePtr;

const double degree = M_PI/180;
sensor_msgs::JointState original;
sensor_msgs::JointState joint_state;
int initial = 0;

void joint_position_callback(const sensor_msgs::JointState& msg)
{
	//cout << msg << endl;
	if (initial == 0)
	{
		original = msg;
		initial = 1;
	}
	else
		joint_state = msg;
}

void ar_pose_callback(const ar_track_alvar::AlvarMarkers& msg)
{
	/*
	cout << joint_state.position.size() << endl;
	cout << original.position.size() << endl;
	if (msg.markers.size() > 0 && joint_state.position.size() > 0 && original.position.size() > 0)
	{
		if (joint_state.position[0] != original.position[0] &&
			joint_state.position[2] != original.position[2] &&
			joint_state.position[3] != original.position[3] &&
			joint_state.position[4] != original.position[4] &&
			joint_state.position[5] != original.position[5] &&
			joint_state.position[6] != original.position[6] &&
			joint_state.position[7] != original.position[7])
		{*/
		if (msg.markers.size() > 0)
		{
			cout << msg.markers[0].pose.pose.position.x <<", "; //x
			cout << msg.markers[0].pose.pose.position.y <<", "; //y
			cout << msg.markers[0].pose.pose.position.z <<", "; //z 
			cout << msg.markers[0].pose.pose.orientation.w <<", "; //w 
			cout << msg.markers[0].pose.pose.orientation.x <<", "; //x 
			cout << msg.markers[0].pose.pose.orientation.y <<", "; //y
			cout << msg.markers[0].pose.pose.orientation.z <<"\n"; //z
		}
		//}
	//}
}

int buffer;
int count_b;

void update_joint_state()
{
	if (joint_state.position.size() > 0)
	{
		//TODO : tests show system going pased joint limits. We need to examine the limts method here.
		ROS_INFO("Incrementing joint states");

		double mult = 1;

		if (count_b % buffer == 0)
		{
			int rand_m = rand();
			cout << rand_m << endl;
			if (rand_m == 0)
				mult = -1;			
		}
	    double outer_yaw_rand = 0.001 * mult;
	    if (abs(joint_state.position[0] + outer_yaw_rand) < 1.57)
			joint_state.position[0] += outer_yaw_rand;

	    double outer_pitch_rand = 0.001 * mult;
	    if (abs(joint_state.position[1] + outer_pitch_rand) < 1.57)
			joint_state.position[1] += outer_pitch_rand;

		/*
	    double outer_insertion_rand = 0.001;
	    if (abs(joint_state.position[6] + outer_insertion_rand) < 2.6179)
			joint_state.position[6] += outer_insertion_rand;

	    double outer_roll_rand = 0.001;
	    if (abs(joint_state.position[7] + outer_roll_rand) < 2.6179)
			joint_state.position[7] += outer_roll_rand;

	    double outer_wrist_pitch_rand = 0.001;
	    if (abs(joint_state.position[8] + outer_wrist_pitch_rand) < 2.6179)
			joint_state.position[8] += outer_wrist_pitch_rand;

	    double outer_wrist_yaw_rand = 0.001;
	    if (abs(joint_state.position[9] + outer_wrist_yaw_rand) < 2.6179)
			joint_state.position[9] += outer_wrist_yaw_rand;

	    double outer_wrist_open_rand = 0.001;
	    if (abs(joint_state.position[10] + outer_wrist_open_rand) < 2.6179)
			joint_state.position[10] += outer_wrist_open_rand;
		*/
		count_b++;
		ROS_INFO("Done incrementing joint states");
	}
}

int main(int argc, char** argv)
{
	buffer = 100;
	count_b = 0;

	ros::init(argc, argv, "test_broadcaster");
	ros::NodeHandle nh;

	ROS_INFO("Creating Publisher and Subscriber");
	//publish joint state messages to actuate psm
	//note: execute rosnode kill /joint_state_publisher to prevent commands from conflicting
	//ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states/joint_position_current", 1);
	//attempt 2 publisher message based on actual_psm_control.py:'/dvrk_psm/set_position_joint'
	//ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/dvrk_psm/set_position_joint", 1);
	//subscribe to pulications of joint States to get starting state  TODO: would this cause a infinite loop of reacting to our own publications?
	//ros::Subscriber joint_sub_1 = nh.subscribe("/joint_states/joint_position_current", 1, joint_position_callback);
	ros::Subscriber joint_sub = nh.subscribe("/ar_pose_marker", 1, ar_pose_callback);
	
	ros::Rate loop_rate(50);
	ros::spin();
	
	//message
	//ROS_INFO("Setting joint names");
	//have to figure out which ones to move
	/* //MESSAGES attempt 1
	joint_state.name.push_back("one_outer_yaw_joint");
	joint_state.name.push_back("one_outer_pitch_joint_1");
	joint_state.name.push_back("one_outer_insertion_joint");
	joint_state.name.push_back("one_outer_roll_joint");
	joint_state.name.push_back("one_outer_wrist_pitch_joint");
	joint_state.name.push_back("one_outer_wrist_yaw_joint");
	joint_state.name.push_back("one_outer_wrist_open_angle_joint_1");//this is not in the FILE ACTUAL_PSM_CONTROL, ALBEIT IT IS NEEDED? ASKED FOR LICENSE???
*/
	//attempt 2 to mirror messages used by dvrk test telop launch file
	//this worked. The issue was that the message we declared only had 7 of the 12 needed joint names.

	/*
	joint_state.name.push_back("one_outer_yaw_joint");
	joint_state.name.push_back("one_outer_pitch_joint_1");
	joint_state.name.push_back("one_outer_pitch_joint_2");
	joint_state.name.push_back("one_outer_pitch_joint_3");
	joint_state.name.push_back("one_outer_pitch_joint_4");
	joint_state.name.push_back("one_outer_pitch_joint_5");
	joint_state.name.push_back("one_outer_insertion_joint");
	joint_state.name.push_back("one_outer_roll_joint");
	joint_state.name.push_back("one_outer_wrist_pitch_joint");
	joint_state.name.push_back("one_outer_wrist_yaw_joint");
	joint_state.name.push_back("one_outer_wrist_open_angle_joint_1");
	joint_state.name.push_back("one_outer_wrist_open_angle_joint_2");

	//Defined position as home condition, to test joint publications, due to subscription function activation not working
	joint_state.position.push_back(0.0);
	joint_state.position.push_back(0.0);
	joint_state.position.push_back(0.0);
	joint_state.position.push_back(0.0);
	joint_state.position.push_back(0.0);
	joint_state.position.push_back(0.0);
	joint_state.position.push_back(0.0);
	joint_state.position.push_back(0.0);
	joint_state.position.push_back(0.0);
	joint_state.position.push_back(0.0);
	joint_state.position.push_back(0.0);
	joint_state.position.push_back(0.0);
	*/

	//cout << joint_state.position.size() << endl;
	
	/*
	while (ros::ok())
	{	
		//update joint state header
		//ROS_INFO("Updating joint state header");
		//joint_state.header.stamp = ros::Time::now();

		//update joint state
		//ROS_INFO("Updating joint state");
		//update_joint_state();
		
		//send the joint state and transform
		//ROS_INFO("Publishing joint state");
		//joint_pub.publish(joint_state);

		//sleep
		//ROS_INFO("Sleeping");
		loop_rate.sleep();
	}
	*/
	
	return 0;
}