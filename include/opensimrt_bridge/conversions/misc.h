#ifndef CONV_MISC_FBK_28052023
#define CONV_MISC_FBK_28052023

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
	const tf2::Quaternion Trial_and_error{0.5,0.5,0.5,0.5};
	const tf2::Quaternion EYE{0,0,0,1};
	const geometry_msgs::Quaternion TO_ROS_G = tf2::toMsg(TO_ROS);
	const geometry_msgs::Quaternion Trial_and_error_G = tf2::toMsg(Trial_and_error);
	const geometry_msgs::Quaternion EYE_G = tf2::toMsg(EYE);

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
	template <class T>
		T get_as_different(const SimTK::Vec3& v)
		{
			//geometry_msgs::Vector3 outvec;
			T outvec;
			outvec.x = -v[1];
			outvec.y = -v[2];
			outvec.z = v[0];
			return outvec;
		};

	//gotta love conversions
	std::vector<std::string> conv_labels(OpenSim::Array<std::string> arg);
	
	template <class T> 
	geometry_msgs::Wrench get_as_ros_wrench(T oW)
	{
	
		geometry_msgs::Wrench w;

		w.force = get_as<geometry_msgs::Vector3>(oW.force);
		w.torque = get_as<geometry_msgs::Vector3>(oW.torque);
		return w;
	}
}

#endif
