

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

int main (int argc, char* argv[])
{
	int frame = 1;
	double error = 0.000;
	int oov = 0;
	
	cout << "argc = " << argc << endl;
	for (int i = 0; i < argc; i++)
		cout << argv[i] << endl;
		
	string r_file_name = argv[1];
	ifstream myfile_r (r_file_name.c_str());
	
	string w_file_name = argv[2];
	ofstream myfile_w (w_file_name.c_str());
	
	string line;
	std::string delimiter = ",";
	
	myfile_w << "Q0, Qx, Qy, Qz, x, y, z, Error, OOV" << endl;
	
	if (myfile_r.is_open())
	{
		while (getline(myfile_r, line))
		{
			vector<string>splitted;
			myfile_w << frame << ", ";
			
			size_t pos = 0;
			std::string token;
			while ((pos = line.find(delimiter)) != std::string::npos) 
			{
				token = line.substr(0, pos);
				splitted.push_back(token);
				line.erase(0, pos + delimiter.length());
			}
			std::reverse(splitted.begin(), splitted.end());
			
			for (int i = 0; i < splitted.size(); i ++)
				myfile_w << splitted[i] << ", ";
			
			myfile_w << error << ", "  << oov << endl;
			frame++;
		}
		myfile_r.close();
		myfile_w.close();
	}
	
	
	return 0;
}