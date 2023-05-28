/**
 * @author      : frekle (frekle@bml01.mech.kth.se)
 * @file        : utils
 * @created     : Saturday May 27, 2023 19:28:28 CEST
 */
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "opensimrt_bridge/conversions/message_convs.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

//pattern : i have some opensim object and i pass it to the converter and it will return me the message i need for the publisher like
//
// ros_msg_obj = message_converter (opensim_obj A, args* )
//
using namespace OpenSimRT;

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



	ExternalWrench::Input parse_message(const opensimrt_msgs::CommonTimedConstPtr & msg_grf, boost::array<int,9> grfIndexes)
	{
		//ROS_DEBUG_STREAM("Parsing wrench from message");
		ExternalWrench::Input a;
		//for (auto ind:grfIndexes)
		//	ROS_INFO_STREAM("index:" << ind);
		a.point  = SimTK::Vec3(msg_grf->data[grfIndexes[0]],msg_grf->data[grfIndexes[1]],msg_grf->data[grfIndexes[2]]);
		a.force  = SimTK::Vec3(msg_grf->data[grfIndexes[3]],msg_grf->data[grfIndexes[4]],msg_grf->data[grfIndexes[5]]);
		a.torque = SimTK::Vec3(msg_grf->data[grfIndexes[6]],msg_grf->data[grfIndexes[7]],msg_grf->data[grfIndexes[8]]);

		//ROS_DEBUG_STREAM("wrench i got:");
		//print_wrench(a);

		return a;
	}

	ExternalWrench::Input parse_message(const geometry_msgs::WrenchStampedConstPtr& w, std::string ref_frame, tf2_ros::Buffer& tfBuffer, std::string grf_reference_frame)
	{
		ExternalWrench::Input wO;
		wO.force[0] = w->wrench.force.x;
		wO.force[1] = w->wrench.force.y;
		wO.force[2] = w->wrench.force.z;
		wO.torque[0] = w->wrench.torque.x;
		wO.torque[1] = w->wrench.torque.y;
		wO.torque[2] = w->wrench.torque.z;
		// now get the translations from the transform
		// for the untranslated version I just want to get the reference in their own coordinate reference frame
		wO.point[0] = 0;
		wO.point[1] = 0;
		wO.point[2] = 0;
		geometry_msgs::TransformStamped nulltransform, actualtransform,inv_t;
		try
		{
			//ATTENTION FUTURE FREDERICO:
			//this is actually already correct. what you need to do use this function is to have another fixed transform generating a "subject_opensim" frame of reference and everything should work

			nulltransform = tfBuffer.lookupTransform(grf_reference_frame, ref_frame, ros::Time(0));
			//nulltransform = tfBuffer.lookupTransform("subject_opensim", ref_frame, ros::Time(0));
			wO.point[0] = nulltransform.transform.translation.x;
			wO.point[1] = nulltransform.transform.translation.y;
			wO.point[2] = nulltransform.transform.translation.z;
			//actualtransform = tfBuffer.lookupTransform("map", ref_frame, ros::Time(0));
			//inv_t = tfBuffer.lookupTransform(ref_frame,"map", ros::Time(0));
			ROS_INFO_STREAM("null transform::\n" << nulltransform);
			//ROS_DEBUG_STREAM("actual transform" << actualtransform);
			//ROS_DEBUG_STREAM("inverse transform" << inv_t);
			//inv_t converts back to opensim

			//now convert it:

		}
		catch (tf2::TransformException &ex) {
			ROS_ERROR("transform exception: %s",ex.what());
			ros::Duration(1.0).sleep();
			return wO;
		}

		ROS_WARN_STREAM("TFs in wrench parsing of geometry_wrench messages not implemented! Rotated frames will fail!");
		return wO;

	}


	std::vector<SimTK::Vector> parse_ik_message(const opensimrt_msgs::CommonTimedConstPtr& message_ik, double* filtered_t, LowPassSmoothFilter* ikfilter)
	{
		std::vector<SimTK::Vector> qVec;
		SimTK::Vector qRaw(message_ik->data.size()); //cant find the right copy constructor syntax. will for loop it
		for (int j = 0;j < qRaw.size();j++)
		{
			qRaw[j] = message_ik->data[j];
		}

		double t = message_ik->time;
		// filter
		auto ikFiltered = ikfilter->filter({t, qRaw});
		auto q = ikFiltered.x;
		auto qDot = ikFiltered.xDot;
		auto qDDot = ikFiltered.xDDot;
		ROS_DEBUG_STREAM("Filter ran ok");
		if (!ikFiltered.isValid) {
			ROS_DEBUG_STREAM("filter results are NOT valid");
			return qVec; }
		ROS_DEBUG_STREAM("Filter results are valid");

		*filtered_t = ikFiltered.t;
		//Construct qVec
		qVec.push_back(q);
		qVec.push_back(qDot);
		qVec.push_back(qDDot);

		return qVec;
	}

	std::vector<SimTK::Vector> parse_ik_message(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik)
	{
		std::vector<SimTK::Vector> qVec;
		SimTK::Vector q(message_ik->d0_data.size()),qDot(message_ik->d0_data.size()),qDDot(message_ik->d0_data.size()); 
		for (int j = 0;j < q.size();j++)
		{
			q[j] = message_ik->d0_data[j];
			qDot[j] = message_ik->d1_data[j];
			qDDot[j] = message_ik->d2_data[j];
		}

		//Construct qVec :should be same as above
		qVec.push_back(q);
		qVec.push_back(qDot);
		qVec.push_back(qDDot);
		return qVec;

	}

	//TODO: MAKE THIS INTO A VECTOR OF WRENCHES FFS
	std::vector<ExternalWrench::Input> get_wrench(const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr, std::string right_foot_tf_name, std::string left_foot_tf_name, tf2_ros::Buffer& tfBuffer, std::string grf_reference_frame)
	{
		// TODO: get wrench message!!!!!!!!!!
		std::vector<ExternalWrench::Input> wrenches;
		OpenSimRT::ExternalWrench::Input grfRightWrench = parse_message(wr, right_foot_tf_name, tfBuffer, grf_reference_frame);
		//cout << "left wrench.";
		ROS_DEBUG_STREAM("rw");
		print_wrench(grfRightWrench);
		ExternalWrench::Input grfLeftWrench = parse_message(wl, left_foot_tf_name, tfBuffer, grf_reference_frame);
		ROS_DEBUG_STREAM("lw");
		print_wrench(grfLeftWrench);
		//	return;

		wrenches.push_back(grfLeftWrench);
		wrenches.push_back(grfRightWrench);
		return wrenches;
	}
	//TODO: MAKE INTO VECTOR OF WRENCHES
	std::vector<ExternalWrench::Input> get_wrench(const opensimrt_msgs::CommonTimedConstPtr& message_grf,boost::array<int,9> grfRightIndexes, boost::array<int,9> grfLeftIndexes )
	{
		std::vector<ExternalWrench::Input> wrenches;
		double t = message_grf->time; //TODO: if it isn't the same as in message ik, this will break!
		ExternalWrench::Input grfRightWrench = parse_message(message_grf, grfRightIndexes);
		//cout << "left wrench.";
		ROS_DEBUG_STREAM("rw");
		print_wrench(grfRightWrench);
		ExternalWrench::Input grfLeftWrench = parse_message(message_grf, grfLeftIndexes);
		ROS_DEBUG_STREAM("lw");
		print_wrench(grfLeftWrench);
		//	return;

		wrenches.push_back(grfLeftWrench);
		wrenches.push_back(grfRightWrench);
		return wrenches;
	}

	void print_wrench(ExternalWrench::Input w)
	{
		ROS_DEBUG_STREAM("POINT" << w.point[0] << ","<< w.point[1] << "," << w.point[2] );
		ROS_DEBUG_STREAM("FORCE" << w.force[0] << ","<< w.force[1] << "," << w.force[2] );
		ROS_DEBUG_STREAM("TORQUE" << w.torque[0] << ","<< w.torque[1] << "," << w.torque[2] );


	}

	boost::array<int,9> generateIndexes(std::vector<std::string> pick, std::vector<std::string> whole) // point,force, torque
	{
		boost::array<int,9> grfIndexes;
		/*ROS_INFO_STREAM("whole.size: " << whole.size());
		  for (auto label:whole)
		  ROS_INFO_STREAM("whole labels: " << label);
		  for (auto label:pick)
		  ROS_INFO_STREAM("pick labels: " << label);
		  */

		for (int i= 0; i<9 ; i++)
		{
			//im assuming this is in order. if it isnt this will break
			int j = 0;
			bool found_it = false;
			for (auto label:whole)
			{
				if(label.compare(pick[i])==0)
				{
					// the pick_label and the label are the same, so the index j is what we want
					grfIndexes[i] = j;
					found_it = true;
					break;
				}
				j++;
			}
			if (!found_it)
				ROS_FATAL_STREAM("Did not find label: [" << pick[i] << "]. Cannot proceed. Check if you loaded the correct MOT file or you set the correct Parameters in the launch file for this node.") ;
		}
		for (auto ind: grfIndexes)
			ROS_WARN_STREAM(ind);

		return grfIndexes;
	}




}
