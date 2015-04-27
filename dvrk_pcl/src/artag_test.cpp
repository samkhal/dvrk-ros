#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <fstream>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <vector>
#include <cmath>
#include <ar_track_alvar/AlvarMarkers.h>
using namespace std;
using namespace ros;

geometry_msgs::Pose ARTag_pose;
Publisher pub_pose;


//callback for AR tag location
void ARtag_callback(const ar_track_alvar::AlvarMarkers& msg)
{

//pub_pose.publish(msg);
	//code in here for getting the transform to the tag on the table
	for (int i = 0; i < msg.markers.size(); i++){
		if (msg.markers[i].id == 2){
			//1 for arm tag, 2 for table tag
			ARTag_pose.position.x = msg.markers[i].pose.pose.position.x;
			ARTag_pose.position.y = msg.markers[i].pose.pose.position.y;
			ARTag_pose.position.z = msg.markers[i].pose.pose.position.z;
			ARTag_pose.orientation.x = msg.markers[i].pose.pose.orientation.x;
			ARTag_pose.orientation.y = msg.markers[i].pose.pose.orientation.y;
			ARTag_pose.orientation.z = msg.markers[i].pose.pose.orientation.z;
			ARTag_pose.orientation.w = msg.markers[i].pose.pose.orientation.w;
			pub_pose.publish(ARTag_pose);
			break;
		}
	}
}



int main(int argc, char** argv)
{

	ros::init(argc, argv, "magic_node");
	ros::NodeHandle nh;

	ROS_INFO("Creating Publisher and Subscribers");
	
	//subscribe to AR tag 
	ros::Subscriber ARtag_sub = nh.subscribe("/ar_pose_marker", 1, &ARtag_callback);
	pub_pose = nh.advertise<geometry_msgs::Pose>("Transform_From_Tag",1);

	ros::Rate loop_rate(50);

	while (ok())
	{	
	        spin();

		//sleep
		loop_rate.sleep();
	}
	
	return 0;
}
