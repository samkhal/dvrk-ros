#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <std_msgs/String.h>
#include <fstream>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <cmath>

using namespace std;
using namespace ros;

double P_gain; //.5 @ 1cm offset
double buffer_zone; // 1cm
//double error_vec[6];
//double error_mag;

geometry_msgs::Pose tool_pose;
geometry_msgs::Wrench force_wrench;
std_msgs::Bool bool_true;
double tool_position[2] = {0.0,0.0}; //tool_position global variable
double points[2][2];

double error_vec[6] = {0,0,0,0,0,0};
double error_mag = 0.0;
 
class ArrayParser
{
public:

	std::string const& operator[](size_t index) const
	{
		return m_data[index];
	}
	std::size_t size() const
	{
		return m_data.size();
	}

	void readNextRow(std::istream& str)
	{
		std::string line;
		std::getline(str, line);
		std::stringstream lineStream(line);
		std::string cell;

		m_data.clear();
		while(std::getline(lineStream, cell, ','))
		{
			m_data.push_back(cell);
		}
	}

private:
	std::vector<std::string> m_data;
};

std::istream& operator>>(std::istream& str, ArrayParser& data)
{
	data.readNextRow(str);
	return str;
}


//callback for AR tag location
void ARtag_callback(const geometry_msgs::Pose& msg)
{
	//code in here for getting the transform to the tag on the table
}


void ParseData()
{
	//std::vector<std::string> m_data;
	//std::istream& operator>>(std::istream& str, ArrayParser& data);

	


	  string line;
	 
      string path = ros::package::getPath("dvrk_haptics")+"/Data/sinVirtualFixtureWaypointsmmResolution.csv";
      ifstream myfile(path.c_str());

      if (myfile.is_open())
       {
   		 while( getline (myfile,line))
   		 {
   		 	//std::istream& operator>>(std::istream& str, ArrayParser& data);


      // cout << data << '\n';

         }
       myfile.close();
        }



  	 // myfile << "Writing this to a file.\n";
  	  cerr<<"READ the CSV FILE\n";
     
     
}




//callback for tool_pose location
void tool_pose_callback(const geometry_msgs::Pose& msg)
{
	tool_pose.position.x = msg.position.x; 
	tool_pose.position.y = msg.position.y;
}



//calculate error of end-effector to closest point on path
int error_calc()
{	

	/*
	//find closest point to end-effector
	//determine which of the two lines is closest
	//determine closest point
	//calculate error vector {dx,dy,0,0,0,0}
	double closest_dist = 99999999999; //DBL_MAX or INFINITY (dunno if wouldn't work for not C++ 11)
	int closest_point_idx = 0;
	for (size_t i = 0; i < points.size(); i++)
	{
		double x_path = points[i][0];
		double y_path = points[i][1];
		double diff_x = tool_position[0] - x_path;
		double diff_y = tool_position[1] - y_path;
		
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

	return closest_point_idx;

	*/
	/*
	//perpendicular stuff
	if (i == 0)
	{

	}
	else if (i == points.size()-1)
	{

	}
	else
	{
		double pointn[2] = {points[i][0], points[i][1]};
		double pointnp1[2] = {points[i-1][0], points[i-1][1]};
		double pointnm1[2] = {points[i+1][0], points[i+1][1]};
		
		double x1 = point np

		if (dist1 < dist2)
		{
			error_vec[0] = tool_position[0] + dist1*()
		}
	}
	*/
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "magic_node");
	ros::NodeHandle nh;

	//force_wrench = {0,0,0,0,0,0};
	force_wrench.force.x = 0.0;
	force_wrench.force.y = 0.0;
	force_wrench.force.z = 0.0;
	force_wrench.torque.x = 0.0;
	force_wrench.torque.y = 0.0;
	force_wrench.torque.z = 0.0;




	ROS_INFO("Creating Publisher and Subscriber");

	//subscribe to end-effector pose
	ros::Subscriber tool_pose_sub = nh.subscribe("/dvrk_psm1/joint_position_cartesian", 1, tool_pose_callback);
	//subscribe to AR tag 
	//ros::Subscriber ARtag_sub = nh.subscribe("something for AR tag", 1, ARtag_callback);
	
	//publisher for force vector
	ros::Publisher w_pub = nh.advertise<geometry_msgs::Wrench>("/dvrk_mtmr/set_wrench_static", 1);

	//publisher to enable torque mode
	ros::Publisher torque_mode_pub = nh.advertise<std_msgs::Bool>("/dvrk_mtmr/enable_torque_mode",1);

	ros::Rate loop_rate(50);

	//vector of vectors, assuming this is what we'd 

	//std::vector<vector<double>> points(10, vector<double>(3));

	/**Need help with file parsing..
	 * and putting the data into the vector of vectors
	 * 
	 * 
	 * 
	**/

	P_gain = .5 * 100; //.5 @ 1cm offset
	buffer_zone = 10.0; // 1cm
	
	bool_true.data = true;

	ParseData();

	while (ros::ok())
	{	
		
		

		error_mag = sqrt((error_vec[0]*error_vec[0]) + (error_vec[1]*error_vec[1]));
		if (error_mag >= buffer_zone){
			force_wrench.force.x = P_gain / error_vec[0];
			force_wrench.force.y = P_gain / error_vec[1];
		} else {	//within buffer zone
			force_wrench.force.x = P_gain * error_vec[0];
			force_wrench.force.y = P_gain * error_vec[1];
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
