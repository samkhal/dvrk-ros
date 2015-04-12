#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <cmath>

using namespace std;
using namespace ros;

typedef sensor_msgs::JointState* JointStatePtr;

const double degree = M_PI/180;
sensor_msgs::JointState joint_state;

void joint_position_callback(const sensor_msgs::JointState& msg)
{
	if (msg.position.size() != 12)
	{
		ROS_ERROR("Message has to have length 12");
	}
	else
	{
		ROS_INFO("Joint Position Callback");
		joint_state.position.resize(0);
		joint_state.position.push_back(msg.position[0]);
		joint_state.position.push_back(msg.position[1]);
		joint_state.position.push_back(msg.position[6]*1000);
		joint_state.position.push_back(msg.position[7]);
		joint_state.position.push_back(msg.position[8]);
		joint_state.position.push_back(msg.position[9]);
		joint_state.position.push_back(msg.position[10]);
	}
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
	ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states/joint_position_current", 1);
	//attempt 2 publisher message based on actual_psm_control.py:'/dvrk_psm/set_position_joint'
	//ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/dvrk_psm/set_position_joint", 1);
	//subscribe to pulications of joint States to get starting state  TODO: would this cause a infinite loop of reacting to our own publications?
	//ros::Subscriber joint_sub = nh.subscribe("joint_states/joint_position_current", 1, joint_position_callback);
	ros::Rate loop_rate(50);
	
	//message
	ROS_INFO("Setting joint names");
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

	cout << joint_state.position.size() << endl;
	
	while (ros::ok())
	{	
		//update joint state header
		ROS_INFO("Updating joint state header");
		joint_state.header.stamp = ros::Time::now();

		//update joint state
		ROS_INFO("Updating joint state");
		update_joint_state();
		
		//send the joint state and transform
		ROS_INFO("Publishing joint state");
		joint_pub.publish(joint_state);

		//sleep
		ROS_INFO("Sleeping");
		loop_rate.sleep();
	}
	
	return 0;
}