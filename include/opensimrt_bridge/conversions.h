
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Wrench.h"
#include <Common/Array.h>
#include <SimTKcommon/SmallMatrix.h>
#include <geometry_msgs/Wrench.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace Osb
{
	const tf2::Quaternion TO_ROS{0,0.707,0.707,0};
	const geometry_msgs::Quaternion TO_ROS_G = tf2::toMsg(TO_ROS);

	template <class T>
		T get_as(const SimTK::Vec3& v)
		{
			//geometry_msgs::Vector3 outvec;
			T outvec;
			outvec.x = v[0];
			outvec.y = v[1];
			outvec.z = v[2];
			return outvec;
		};

	//gotta love conversions
	std::vector<std::string> conv_labels(OpenSim::Array<std::string> arg)
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
	
	template <class T> 
	geometry_msgs::Wrench get_as_ros_wrench(T oW)
	{
	
		geometry_msgs::Wrench w;

		w.force = get_as<geometry_msgs::Vector3>(oW.force);
		w.torque = get_as<geometry_msgs::Vector3>(oW.torque);
		return w;
	}
}

