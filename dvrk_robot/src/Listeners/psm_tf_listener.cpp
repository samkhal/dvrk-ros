
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <rosbag/bag.h>

using namespace std;
using namespace ros;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "cartesian_pose_node");
  ros::NodeHandle n;


   
  	//Subscriber sub = n.subscribe("/dvrk_psm1/set_cartesian_pose", 1, &PoseExtractionCallback);
	Publisher pub = n.advertise<geometry_msgs::Pose>("/dvrk_psm1/arbitrary_pose",1);
  	geometry_msgs::Pose pose;

	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 1;
	pose.orientation.w = 0;

  	for (int i = 0; i < 10; i++)
  	{
		pose.position.x +=0.1;
		pose.position.y +=0.1;
		pose.position.z +=0.1;
		sleep(2.0);
	}

 
	while(ok()){
		  spinOnce();
	}

	return 0;
}
