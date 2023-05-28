/**
 * @author      : frekle (frekle@bml01.mech.kth.se)
 * @file        : utils
 * @created     : Saturday May 27, 2023 19:28:28 CEST
 */
#include "ros/ros.h"
#include "opensimrt_bridge/conversions/message_convs.h"
#include "opensimrt_msgs/CommonTimed.h"

//pattern : i have some opensim object and i pass it to the converter and it will return me the message i need for the publisher like
//
// ros_msg_obj = message_converter (opensim_obj A, args* )
//

namespace Osb
{
	opensimrt_msgs::PosVelAccTimed get_as_ik_filtered_msg(std_msgs::Header h, double t, SimTK::Vector q, SimTK::Vector qDot, SimTK::Vector qDDot)
	{
		opensimrt_msgs::PosVelAccTimed msg_filtered;
		msg_filtered.header = h;
		msg_filtered.time = t;
		//for loop to fill the data appropriately:
		for (int i=0;i<q.size();i++)
		{
			msg_filtered.d0_data.push_back(q[i]);
			msg_filtered.d1_data.push_back(qDot[i]);
			msg_filtered.d2_data.push_back(qDDot[i]);
		}
		return msg_filtered;
	}		

	opensimrt_msgs::CommonTimed get_as_ik_msg(std_msgs::Header h, double t, SimTK::Vector q)
	{
		opensimrt_msgs::CommonTimed msg;
		msg.header = h;
		msg.time = t;

		for (double joint_angle:q)
		{
			ROS_DEBUG_STREAM("some joint_angle:"<<joint_angle);
			msg.data.push_back(joint_angle);
		}
		return msg;
	}
	void update_pose(opensimrt_msgs::CommonTimed& msg, double t, SimTK::Vector q)
	{
		msg.time = t;

		for (double joint_angle:q)
		{
			ROS_DEBUG_STREAM("some joint_angle:"<<joint_angle);
			msg.data.push_back(joint_angle);
		}
	}

	opensimrt_msgs::CommonTimed get_GRFMs_as_common_msg(OpenSimRT::GRFMNonSmooth::Output grfmOutput, double t, std_msgs::Header h)
	{
		//TODO: NO LABELS FOR ORDER??
		//

		//OpenSim::TimeSeriesTable output;
		std::vector<double> p;
		auto a = grfmOutput.right.toVector() ;
		auto b = grfmOutput.left.toVector() ;

		p.insert(p.end(),a.begin(),a.end());
		p.insert(p.end(),b.begin(),b.end());
		//output.appendRow(grfmOutput.t,p);
		opensimrt_msgs::CommonTimed msg;
		if (false)
		{
			std_msgs::Header h;
			h.frame_id = "subject";
			h.stamp = ros::Time::now();
			msg.header = h;
		} else
		{
			msg.header = h; //will this break? it will be publishing messages in the past
		}
		msg.time = t;
		msg.data.insert(msg.data.end(), p.begin(),p.end());
		return msg;
	}
}
