/**
 * @author      : frekle (frekle@bml01.mech.kth.se)
 * @file        : utils
 * @created     : Saturday May 27, 2023 19:26:06 CEST
 */

#ifndef MESSAGE_CONVERTERS_UTILS_H
#define MESSAGE_CONVERTERS_UTILS_H

#include "MuscleOptimization.h"
#include "RealTime/include/experimental/GRFMNonSmooth.h"
#include "SignalProcessing.h"
#include "geometry_msgs/WrenchStamped.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Event.h"
#include "opensimrt_msgs/Dual.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "ros/message_traits.h"
#include "tf2_ros/buffer.h"
#include <SimTKcommon/internal/BigMatrix.h>

namespace Osb
{

	//IK
	opensimrt_msgs::PosVelAccTimed get_as_ik_filtered_msg(std_msgs::Header h, double t, SimTK::Vector q, SimTK::Vector qDot, SimTK::Vector qDDot);

	opensimrt_msgs::CommonTimed get_as_ik_msg(std_msgs::Header h, double t, SimTK::Vector q);
	void update_pose(opensimrt_msgs::CommonTimed& msg, double t, SimTK::Vector q);


	//Stuff from ID:

	opensimrt_msgs::CommonTimed get_GRFMs_as_common_msg(OpenSimRT::GRFMNonSmooth::Output grfmOutput, double t, std_msgs::Header h);

	std::vector<OpenSimRT::ExternalWrench::Input> get_wrench(const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr, std::string right_foot_tf_name, std::string left_foot_tf_name, tf2_ros::Buffer& tfBuffer, std::string grf_reference_frame);

//	std::vector<OpenSimRT::ExternalWrench::Input> get_wrench(const opensimrt_msgs::CommonTimedConstPtr& message_grf);
	std::vector<OpenSimRT::ExternalWrench::Input> get_wrench(const opensimrt_msgs::CommonTimedConstPtr& message_grf,boost::array<int,9> grfRightIndexes, boost::array<int,9> grfLeftIndexes );

	OpenSimRT::ExternalWrench::Input parse_message(const opensimrt_msgs::CommonTimedConstPtr& msg_grf, boost::array<int,9> grfIndexes);
	OpenSimRT::ExternalWrench::Input parse_message(const geometry_msgs::WrenchStampedConstPtr& w, std::string ref_frme, tf2_ros::Buffer& tfBuffer, std::string grf_reference_frame);
	std::vector<SimTK::Vector> parse_ik_message(const opensimrt_msgs::CommonTimedConstPtr& message_ik, double* filtered_t, OpenSimRT::LowPassSmoothFilter* ikfilter );
	std::vector<SimTK::Vector> parse_ik_message(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik);
	void print_wrench(OpenSimRT::ExternalWrench::Input w);

	boost::array<int,9> generateIndexes(std::vector<std::string> pick, std::vector<std::string> whole); 

	opensimrt_msgs::Dual get_SO_as_Dual(std_msgs::Header h, double t,SimTK::Vector q,OpenSimRT::MuscleOptimization::Output& soOutput);


}
#endif /* end of include guard  */

