#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/package.h>
#include <string>
#include <std_msgs/String.h>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <cmath>
#include <ar_track_alvar/AlvarMarkers.h>

using namespace std;
using namespace ros;
using namespace boost;

double P_gain; //.5 @ 1cm offset
double buffer_zone; // 1cm
//double error_vec[6];
//double error_mag;

geometry_msgs::Wrench force_wrench;
std_msgs::Bool bool_true;

double tool_position[3] = {0.0,0.0,0.0}; //tool_position in origin frame

std::vector< std::vector<double> > points; //vector of vectors

double error_vec[2] = {0.0,0.0}; //paper frame
double error_mag = 0.0;

geometry_msgs::Pose ARTag_pose_1;
geometry_msgs::Pose ARTag_pose; //Relative to camera frame
tf::Transform artag_tf_1; //Relative to camera frame
tf::Transform artag_tf; //Relative to camera frame

std::vector<tf::Transform> path_tfs; //Vector of point transforms with respect to origin frame
nav_msgs::Path path; //Path object of points with respect to origin

void ParseData()
{

	string line;

	string path = ros::package::getPath("dvrk_haptics")+"/Data/sinVirtualFixtureWaypointsmmResolutionV3.csv";
	ifstream myfile(path.c_str());

    //import csv file if opening was successful
	if (myfile.is_open())
	{
		 while(getline (myfile,line))
		 {

			std::string delim = ",";
			std::vector<double> point_from_line;
			
			size_t pos = 0;
			std::string token;
			while ((pos = line.find(delim)) != std::string::npos) 
			{
				token = line.substr(0, pos);
				point_from_line.push_back(atof(token.c_str()));
				line.erase(0, pos + delim.length());
			}
			token = line.substr(0, pos);
			point_from_line.push_back(atof(token.c_str()));
			points.push_back(point_from_line);

	 	}
		myfile.close();
	    cerr<<"READ the CSV FILE\n";
	}
}

//Transform AR tag pose into the base frame
void PoseTransform()
{

			
	tf::Vector3 point_vec(ARTag_pose.position.x, ARTag_pose.position.y, ARTag_pose.position.z);
	tf::Quaternion quat_vec(ARTag_pose.orientation.x, ARTag_pose.orientation.y, ARTag_pose.orientation.z, ARTag_pose.orientation.w);	
	artag_tf.setIdentity();
	artag_tf.setOrigin(point_vec);
	artag_tf.setRotation(quat_vec);


	tf::Vector3 point_vec_1(ARTag_pose_1.position.x, ARTag_pose_1.position.y, ARTag_pose_1.position.z);
	tf::Quaternion quat_vec_1(ARTag_pose_1.orientation.x, ARTag_pose_1.orientation.y, ARTag_pose_1.orientation.z, ARTag_pose_1.orientation.w);	
	artag_tf_1.setIdentity();
	artag_tf_1.setOrigin(point_vec_1);
	artag_tf_1.setRotation(quat_vec_1);
	
}

//callback for AR tag location
void ARtag_callback(const ar_track_alvar::AlvarMarkers& msg)
{
	//code in here for getting the transform to the tag on the table
	for (int i = 0; i < msg.markers.size(); i++){
		if (msg.markers[i].id == 1){
			ARTag_pose_1.position.x = msg.markers[i].pose.pose.position.x;
			ARTag_pose_1.position.y = msg.markers[i].pose.pose.position.y;
			ARTag_pose_1.position.z = msg.markers[i].pose.pose.position.z;
			ARTag_pose_1.orientation.x = msg.markers[i].pose.pose.orientation.x;
			ARTag_pose_1.orientation.y = msg.markers[i].pose.pose.orientation.y;
			ARTag_pose_1.orientation.z = msg.markers[i].pose.pose.orientation.z;
			ARTag_pose_1.orientation.w = msg.markers[i].pose.pose.orientation.w;
		}
		else if (msg.markers[i].id == 2){
			//1 for arm tag, 2 for table tag

			ARTag_pose.position.x = msg.markers[i].pose.pose.position.x;
			ARTag_pose.position.y = msg.markers[i].pose.pose.position.y;
			ARTag_pose.position.z = msg.markers[i].pose.pose.position.z;
			ARTag_pose.orientation.x = msg.markers[i].pose.pose.orientation.x;
			ARTag_pose.orientation.y = msg.markers[i].pose.pose.orientation.y;
			ARTag_pose.orientation.z = msg.markers[i].pose.pose.orientation.z;
			ARTag_pose.orientation.w = msg.markers[i].pose.pose.orientation.w;

		}
	}
	PoseTransform();
}


//Transform the path into the base frame
std::vector<tf::Transform> transform_path()
{
	std::vector<tf::Transform> ret_tf; //Vector of point transforms with respect to origin frame
	static tf::TransformListener listener;
	tf::StampedTransform stamp_transform;
	tf::Transform origin_to_paper;

	bool done = false;
	while(!done){
		try{
			listener.lookupTransform("origin","ar_marker_2", ros::Time(0), stamp_transform);
			origin_to_paper = stamp_transform;// * artag_tf;
			done = true;
		}
		catch (tf::TransformException &ex) {
			ros::Duration(1.0).sleep();
		}
	}

	//cout << "artag_tf_1: " << artag_tf_1.getOrigin()[0] << ", " << artag_tf_1.getOrigin()[1] << ", " << artag_tf_1.getOrigin()[2] << endl;
	//cout << "ptip: " << ptip[0] << ", " << ptip[1] << ", " << ptip[2] << endl;
	//cout << "AR tag in base frame: " << origin_to_paper.getOrigin()[0] << ", " << origin_to_paper.getOrigin()[1] << ", " << origin_to_paper.getOrigin()[2] << endl;

	// print rotation or ar tab relative to origin
	//cout << origin_to_paper.getBasis().getColumn(0)[0] << " " << origin_to_paper.getBasis().getColumn(0)[1] << " " << origin_to_paper.getBasis().getColumn(0)[2] << endl;
	//cout << origin_to_paper.getBasis().getColumn(1)[0] << " " << origin_to_paper.getBasis().getColumn(1)[1] << " " << origin_to_paper.getBasis().getColumn(1)[2] << endl;
	//cout << origin_to_paper.getBasis().getColumn(2)[0] << " " << origin_to_paper.getBasis().getColumn(2)[1] << " " << origin_to_paper.getBasis().getColumn(2)[2] << endl;

	std::vector<geometry_msgs::PoseStamped> path_vector;

	for (size_t i = 0; i < points.size(); i++)
	{
		tf::Vector3 point_vec(points[i][1], points[i][2], 0);
		tf::Transform point_trans;
		point_trans.setIdentity();
		point_trans.setOrigin(point_vec);

		//to_mult.setIdentity(); //needs paper in origin frame

		//Transform each point on path
		tf::Transform final_point = origin_to_paper * point_trans;		

		ret_tf.push_back(final_point);	

		//Add stamped pose
		geometry_msgs::PoseStamped stamped_pose;
		stamped_pose.header.frame_id = "origin";
		stamped_pose.pose.position.x = final_point.getOrigin()[0]; 
		stamped_pose.pose.position.y = final_point.getOrigin()[1];
		stamped_pose.pose.position.z = final_point.getOrigin()[2];
		path_vector.push_back(stamped_pose);

		// cout << "origin_to_camera ROTATION\n";
		// cout << origin_to_camera.getBasis().getColumn(0)[0] << " " << origin_to_camera.getBasis().getColumn(0)[1] << " " << origin_to_camera.getBasis().getColumn(0)[2] << endl;
		// cout << origin_to_camera.getBasis().getColumn(1)[0] << " " << origin_to_camera.getBasis().getColumn(1)[1] << " " << origin_to_camera.getBasis().getColumn(1)[2] << endl;
		// cout << origin_to_camera.getBasis().getColumn(2)[0] << " " << origin_to_camera.getBasis().getColumn(2)[1] << " " << origin_to_camera.getBasis().getColumn(2)[2] << endl;
		

		// cout << points[i][1] << ", " << points[i][2] << endl;
		// cout << point_trans.getOrigin()[0] << ", " << point_trans.getOrigin()[1] << endl;
		
		 
	}

	//cout << "TRANSLATION\n";
	//cout << origin_to_paper.getOrigin()[0] << ", " << origin_to_paper.getOrigin()[1] << ", " << origin_to_paper.getOrigin()[2] << endl;
	//cout << final_point.getOrigin()[0] << ", " << final_point.getOrigin()[1] << endl;

	path.header.frame_id = "origin";
	path.poses = path_vector;

	return ret_tf;
}



//callback for tool_pose location
void tool_pose_callback(const geometry_msgs::Pose& pose_msg)
{
	static tf::TransformBroadcaster tf_broadcaster; //broadcaster for tfs

	tf::Transform tool_tf;
	tool_tf.setOrigin(tf::Vector3(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z));
	tool_tf.setRotation(tf::Quaternion( pose_msg.position.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w));

	tf_broadcaster.sendTransform(tf::StampedTransform(tool_tf, ros::Time::now(), "origin", "tooltip"));

	tool_position[0] = pose_msg.position.x; 
	tool_position[1] = pose_msg.position.y;
	tool_position[2] = pose_msg.position.z;

	//cout << "Tool_position: " << tool_position[0] <<" "<< tool_position[1] << "\n";
}

//callback for mtm tip pose
void mtm_pose_callback(const geometry_msgs::Pose& pose_msg)
{
	static tf::TransformBroadcaster tf_broadcaster; //broadcaster for tfs

	tf::Transform mtm_tf;
	mtm_tf.setOrigin(tf::Vector3(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z));
	mtm_tf.setRotation(tf::Quaternion( pose_msg.position.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w));

	tf_broadcaster.sendTransform(tf::StampedTransform(mtm_tf, ros::Time::now(), "mtm_base", "mtm_tip"));
}

//calculate error of end-effector to closest point on path
int error_calc()
{	

	//find closest point to end-effector
	//determine which of the two lines is closest
	//determine closest point
	//calculate error vector {dx,dy,0,0,0,0}
	double closest_dist = 99999999999; //DBL_MAX or INFINITY (dunno if wouldn't work for not C++ 11)
	int closest_point_idx = -1;
	for (size_t i = 0; i < points.size(); i++)
	{
		//double x_path = points[i][1];
		//double y_path = points[i][2];
		double x_path = path_tfs[i].getOrigin()[0];
		double y_path = path_tfs[i].getOrigin()[1];
		double diff_x = x_path - tool_position[0];
		double diff_y = y_path - tool_position[1];
		
		double dist = sqrt(pow(diff_x, 2) + pow(diff_y,2));
		if (dist < closest_dist)
		{
			closest_dist = dist;
			error_vec[0] = diff_x;
			error_vec[1] = diff_y;
			error_mag = dist;
			closest_point_idx = i;
		}
		
	}
	
	double ax_1 = path_tfs[closest_point_idx].getOrigin()[0];
	double ay_1 = path_tfs[closest_point_idx].getOrigin()[1];
	double ax_2 = path_tfs[closest_point_idx-1].getOrigin()[0];
	double ay_2 = path_tfs[closest_point_idx-1].getOrigin()[1];
	double ax_3 = path_tfs[closest_point_idx+1].getOrigin()[0];
	double ay_3 = path_tfs[closest_point_idx+1].getOrigin()[1];
	
	double bx = tool_position[0];
	double by = tool_position[1];
	
	double t1 = ((ax_1-bx)*(ax_2-bx)+(ay_1-by)*(ay_2-by))/(pow((ax_2-ax_1),2)+pow((ay_2-ay_1),2)); //perpendicular distance from line defined by closest point - 1 to closest point
	double t2 = ((ax_2-bx)*(ax_3-bx)+(ay_2-by)*(ay_3-by))/(pow((ax_3-ax_2),2)+pow((ay_3-ay_2),2)); //perpendicular distance from line defined by closest point to closest point + 1
	
	double dx1 = ax_1 + t1*(ax_2 - ax_1); //x of point on line closest point - 1 to closest point
	double dy1 = ay_1 + t1*(ay_2 - ay_1); //y of point on line closest point - 1 to closest point
			 
	double dx2 = ax_2 + t2*(ax_3 - ax_2); //x of point on line closest point to closest point + 1
	double dy2 = ay_2 + t2*(ay_3 - ay_2); //y of point on line closest point to closest point + 1
	if (0 <= t1 && t1 <= 1 && 0 <= t2 && t2 <= 1) //both points are in their respective lines
	{
		if (t1 < t2)
		{
			error_vec[0] = dx1 - bx;
			error_vec[1] = dy1 - by;
		}
		else
		{
			error_vec[0] = dx2 - bx;
			error_vec[1] = dy2 - by;
		}
	}
	else if (0 <= t1 && t1 <= 1) //first point is in its line
	{
		error_vec[0] = dx1 - bx;
		error_vec[1] = dy1 - by;

		double dist1 = sqrt(pow(bx-ax_2,2) + pow(by-ay_2,2));
		double dist2 = sqrt(pow(error_vec[0],2) + pow(error_vec[1],2));

		if (dist1 < dist2)
		{
			error_vec[0] = ax_2 - bx;
			error_vec[1] = ay_2 - by;
		}
	}
	else if (0 <= t2 && t2 <= 1) //second point is in its line
	{
		error_vec[0] = dx2 - bx;
		error_vec[1] = dy2 - by;

		double dist1 = sqrt(pow(bx-ax_2,2) + pow(by-ay_2,2));
		double dist2 = sqrt(pow(error_vec[0],2) + pow(error_vec[1],2));

		if (dist1 < dist2)
		{
			error_vec[0] = ax_2 - bx;
			error_vec[1] = ay_2 - by;
		}
	}

	return closest_point_idx; //failure case (no orthogonal projection) has error_vec storing the proper distance already
}


//Continuously update and publish force wrench
int main(int argc, char** argv)
{

	ros::init(argc, argv, "haptics_node");
	ros::NodeHandle nh;

	//force_wrench = {0,0,0,0,0,0};
	force_wrench.force.x = 0.0;
	force_wrench.force.y = 0.0;
	force_wrench.force.z = 0.0;
	force_wrench.torque.x = 0.0;
	force_wrench.torque.y = 0.0;
	force_wrench.torque.z = 0.0;


	static tf::TransformListener listener;

	ROS_INFO("Creating Publisher and Subscriber");

	//subscribe to end-effector pose
	ros::Subscriber tool_pose_sub = nh.subscribe("/dvrk_psm1/joint_position_cartesian", 1, tool_pose_callback);

	//subscribe to mtm_pose
	ros::Subscriber mtm_pose_sub = nh.subscribe("/dvrk_mtmr/joint_position_cartesian", 1, mtm_pose_callback);
	
	//publisher for force vector
	ros::Publisher w_pub = nh.advertise<geometry_msgs::Wrench>("/dvrk_mtmr/set_wrench_static", 1);

	//publisher to enable torque mode
	ros::Publisher torque_mode_pub = nh.advertise<std_msgs::Bool>("/dvrk_mtmr/enable_torque_mode",1);

	//publisher for publishing path
	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path",1);

	//publisher for error vector
	ros::Publisher error_pub = nh.advertise<visualization_msgs::Marker>("/error_vec",1);

	ros::Rate loop_rate(10);


	//subscribe to AR tag 
	ros::Subscriber ARtag_sub = nh.subscribe("/ar_pose_marker", 1, &ARtag_callback);



		double f_max = .5; //.5N
		double width = .02; //1cm

		double kp = f_max/width; //Proportional Gain

		
		bool_true.data = true;

		ParseData();

		//should return at index 341 
		path_tfs = transform_path();

		while (ros::ok())
		{	
			path_tfs = transform_path();
			
			int index = error_calc();

			error_mag = sqrt((error_vec[0]*error_vec[0]) + (error_vec[1]*error_vec[1]));

			double f_mag; //Magnitude of applied force

			if(error_mag < width){ //Within inner range
				f_mag = kp * error_mag;
			}else if(error_mag < 2*width){ //within outer range
				f_mag = -kp * (error_mag - width) + f_max;
			}else{ //outside of outer range
				f_mag = 0;
			}

			//just in case
			if(f_mag>0.5){
				f_mag = 0.5;
			}
			if(f_mag<-0.5){
				f_mag = -0.5;
			}
			
			force_wrench.force.x = f_mag * error_vec[0] / error_mag;
			force_wrench.force.y = f_mag * -error_vec[1] / error_mag;
			
			cout << "Tool position: " << tool_position[0] << "  " << tool_position[1] << " " << tool_position[2] << endl;
			cout << "Error Vec: " << error_vec[0] << " " << error_vec[1] << endl;
			//cout << "Error Mag: " << error_mag << endl;
			//cout << "Point index: " << index << endl;
			//cout << "Point pose (origin): " << path_tfs[index].getOrigin()[0] << " " << path_tfs[index].getOrigin()[1] << " " << path_tfs[index].getOrigin()[2] << " " << endl;
			cout << "Force Wrench: " << force_wrench.force.x << " " << force_wrench.force.y << endl;

			//enable torque mode, just in case (only actually necessary after Mono is released, but it doesn't hurt)
			torque_mode_pub.publish(bool_true);
				
			//if we have to transform the force_wrench into the master tip frame
			/*
			tf::StampedTransform stamp_transform;
			try{
				listener.lookupTransform("mtm_tip","mtm_base", ros::Time(0), stamp_transform);

				tf::Vector3 force_vec(force_wrench.force.x, force_wrench.force.y, 0); //Create vector from wrench
				tf::Vector3 force_vec_tip_frame = stamp_transform * force_vec;        //Transform vector to tip frame
				
				force_wrench.force.x = force_vec_tip_frame[0];
				force_wrench.force.y = force_vec_tip_frame[1];
				force_wrench.force.z = force_vec_tip_frame[2];

				//send the force_wrench
				w_pub.publish(force_wrench);
			}
			catch (tf::TransformException ex){ //If not found continue without publishing wrench				
			}*/
			
			//send the force_wrench
			w_pub.publish(force_wrench);

			//Publish path
			path_pub.publish(path);

			//Publish error vector
			visualization_msgs::Marker arrow;
			arrow.header.frame_id = "origin";
			arrow.header.stamp = ros::Time();
			arrow.ns = "error";
			arrow.id = 0;
			arrow.type = visualization_msgs::Marker::ARROW;
			arrow.action = visualization_msgs::Marker::ADD;
			arrow.color.a = 1.0;
			arrow.color.r = 1.0;
			arrow.color.b = 0.0;
			arrow.color.g = 0.0;
			arrow.scale.x = 0.002; //shaft diam
			arrow.scale.y = 0.004; //head diam
			arrow.scale.z = 0.01; //head length

			geometry_msgs::Point start;
			start.x = tool_position[0];
			start.y = tool_position[1];
			start.z = tool_position[2];
			
			geometry_msgs::Point end;
			end.x = tool_position[0]+error_vec[0];
			end.y = tool_position[1]+error_vec[1];
			end.z = tool_position[2];

			arrow.points.push_back(start);
			arrow.points.push_back(end);
			error_pub.publish(arrow);
			
			//sleep
			spinOnce();
			loop_rate.sleep();
	}

	return 0;
}
