
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <visualization_msgs/Marker.h>
#include <rosbag/bag.h>
#include <iostream>

using namespace std;
using namespace ros;


Publisher pub;
Subscriber sub;

void PoseExtractionCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& pose)
{
	//pub.publish(pose);
	if (pose->markers.size() == 0)
		return;

	double x = pose->markers[0].pose.pose.position.x;
	double y = pose->markers[0].pose.pose.position.y;

	geometry_msgs::Twist positionMsg;

	positionMsg.linear.x = x;
	positionMsg.linear.y = y;

	pub.publish(positionMsg);


	//double linearVelocity ;
	//double angularVelocity;


   
   //pose.header.frame_id = 'First_Pose';

/*
   for(int i = 0; i >= 100; i++){

   ROS_INFO("Pose Elements");
   cout << pose.markers[i].id << endl;
   cout << pose.markers[i].pose.pose.position << endl;
   cout << pose.markers[i].pose.pose.orientation << endl;

}
*/
   }
   //cerr<<pose.markers[0]<<endl;



int main(int argc, char **argv)
{

  ros::init(argc, argv, "cartesian_pose_node");
  ros::NodeHandle n;


   
  	 sub = n.subscribe("/ar_pose_marker", 1, &PoseExtractionCallback);
	 pub = n.advertise<ar_track_alvar::AlvarMarkers>("/dvrk_psm1/arbitrary_pose",1);
  
 
	while(ok()){
		  spinOnce();
	}

	return 0;
}
