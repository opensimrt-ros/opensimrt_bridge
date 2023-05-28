/**
 * @author      : frekle (frekle@bml01.mech.kth.se)
 * @file        : utils
 * @created     : Saturday May 27, 2023 19:26:06 CEST
 */

#ifndef MESSAGE_CONVERTERS_UTILS_H
#define MESSAGE_CONVERTERS_UTILS_H

#include "RealTime/include/experimental/GRFMNonSmooth.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include <SimTKcommon/internal/BigMatrix.h>

namespace Osb
{
	opensimrt_msgs::PosVelAccTimed get_as_ik_filtered_msg(std_msgs::Header h, double t, SimTK::Vector q, SimTK::Vector qDot, SimTK::Vector qDDot);

	opensimrt_msgs::CommonTimed get_as_ik_msg(std_msgs::Header h, double t, SimTK::Vector q);
	void update_pose(opensimrt_msgs::CommonTimed& msg, double t, SimTK::Vector q);
	opensimrt_msgs::CommonTimed get_GRFMs_as_common_msg(OpenSimRT::GRFMNonSmooth::Output grfmOutput, double t, std_msgs::Header h);
}
#endif /* end of include guard  */

