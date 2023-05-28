#include "opensimrt_bridge/conversions/misc.h"

std::vector<std::string> Osb::conv_labels(OpenSim::Array<std::string> arg)
{
	std::vector<std::string> out;
	for (int i=0;i<arg.size();i++)
	{
		std::string hello =arg.get(i);
		std::cout << hello << " "<< std::endl;
		out.push_back(arg.get(i));
	};
	return out;
};

