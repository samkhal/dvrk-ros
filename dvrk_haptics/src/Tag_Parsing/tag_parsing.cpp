#include <ros/ros.h>
#include <string>
#include <iterator>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>


using namespace std;
using namespace ros;


//Subscriber sub_data; 
//Publisher pub_transforms;

//This class creates rows from data being streamed  
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




int main (int argc, char** argv)
{
	
	//ros::init(argc, argv, "parsing_transform_node");
	//ros::NodeHandle n;
	

	std::ifstream file("sinVirtualFixtureWaypointsmmResolution.csv");
	ArrayParser row;
	while(file>>row)
	{
		cout<<"Testing Element ("<<row[3]<<" )\n";
	}
	
	return 0;

}


// basic file operations

using namespace std;

int main () {
  ofstream myfile;
  myfile.open ("example.txt");
  myfile << "Writing this to a file.\n";
  myfile.close();
  return 0;
}