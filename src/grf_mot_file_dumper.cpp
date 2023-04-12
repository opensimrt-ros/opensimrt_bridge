#include "InverseDynamics.h"
#include "OpenSimUtils.h"
#include "SignalProcessing.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "opensimrt_msgs/PointWrenchTimed.h"
#include "opensimrt_msgs/LabelsSrv.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "ros/subscriber.h"
#include "std_msgs/Header.h"
#include <SimTKcommon/SmallMatrix.h>
#include <cstdlib>
#include <opensimrt_bridge/conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "Ros/include/common_node.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;
using namespace Osb;

//TODO: maybe think about making ik_file_dumper a base class instead of copy-pasting everything

//! oh, yeah, this guy.
class Gfrm: public Ros::CommonNode
{
	//some important variables should be here
	LowPassSmoothFilter* grfRightFilter, *grfLeftFilter;
	Storage* grfMotion;
	bool use_filter, tf_apply_rotation, publish_tf;
	bool read_4_force_plates = false; //DIRTY HORRIBLE HACK
	vector<string> grfRightLabels, grfLeftLabels; 
	//publishers
	ros::Publisher r_pub, l_pub;	
	ros::Publisher rw_pub, lw_pub;	
	ros::Publisher rp_pub, lp_pub;	
	ros::Publisher common_pub;
	tf2_ros::TransformBroadcaster br;

	string tf_left_foot_frame, tf_right_foot_frame, tf_ref_frame;

	public:
	Gfrm()
	{
		// subject data
		//auto section = "TEST_ID_FROM_FILE";
		ros::NodeHandle nh("~");
		string grf_mot_file;
		nh.param<std::string>("grf_mot_file", grf_mot_file, "");

		//TODO: This is the incorrect way of doing this. Ideally I want to be able to read many external forces from the MOT file, not just 2, so there should be a sort of vector for this, or a yaml file or something. 
		//everything else that follows is kind of a hack
		//
		nh.param<bool>("read_4_force_plates", read_4_force_plates, false);
		//GFRM params
		string grfRightApplyBody;
		nh.param<string>("grf_right_apply_to_body", grfRightApplyBody, "");
		string grfRightForceExpressed;
		nh.param<string>("grf_right_force_expressed_in_body", grfRightForceExpressed, "");
		string grfRightPointExpressed;
		nh.param<string>("grf_right_point_expressed_in_body", grfRightPointExpressed, "");
		string grfRightPointIdentifier;
		nh.param<string>("grf_right_point_identifier", grfRightPointIdentifier, "");
		string grfRightForceIdentifier;
		nh.param<string>("grf_right_force_identifier", grfRightForceIdentifier, "");
		string grfRightTorqueIdentifier;
		nh.param<string>("grf_right_torque_identifier", grfRightTorqueIdentifier, "");

		string grfLeftApplyBody;
		nh.param<string>("grf_left_apply_to_body", grfLeftApplyBody, "");
		string grfLeftForceExpressed;
		nh.param<string>("grf_left_force_expressed_in_body", grfLeftForceExpressed, "");
		string grfLeftPointExpressed;
		nh.param<string>("grf_left_point_expressed_in_body", grfLeftPointExpressed, "");
		string grfLeftPointIdentifier;
		nh.param<string>("grf_left_point_identifier", grfLeftPointIdentifier, "");
		string grfLeftForceIdentifier;
		nh.param<string>("grf_left_force_identifier", grfLeftForceIdentifier, "");
		string grfLeftTorqueIdentifier;
		nh.param<string>("grf_left_torque_identifier", grfLeftTorqueIdentifier, "");
		

		nh.param<bool>("tf_apply_rotation", tf_apply_rotation, true);
		nh.param<bool>("publish_tf", publish_tf, true);
		nh.param<string>("tf_left_foot_frame", tf_left_foot_frame, "left_foot_forceplate");
		nh.param<string>("tf_right_foot_frame", tf_right_foot_frame, "right_foot_forceplate");
		nh.param<string>("tf_ref_frame", tf_ref_frame, "map");

		//Params for GFRM filter
		nh.param<bool>("use_filter", use_filter, true);
		int memory;
		nh.param<int>("memory", memory, 0);
		double cutoffFreq; 
		nh.param<double>("cutoff_freq", cutoffFreq, 0);
		int delay;
		nh.param<int>("delay", delay, 0);
		int splineOrder;
		nh.param<int>("spline_order", splineOrder, 0);

		// setup external forces
		grfMotion = new Storage(grf_mot_file);

		auto labels_ = grfMotion->getColumnLabels();
		output_labels = conv_labels(labels_); // am I messing the order here?
					       

		ExternalWrench::Parameters grfRightFootPar{
			grfRightApplyBody, grfRightForceExpressed, grfRightPointExpressed};
		grfRightLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
				grfRightPointIdentifier, grfRightForceIdentifier,
				grfRightTorqueIdentifier);
		auto grfRightLogger = ExternalWrench::initializeLogger();

		ExternalWrench::Parameters grfLeftFootPar{
			grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
		grfLeftLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
				grfLeftPointIdentifier, grfLeftForceIdentifier,
				grfLeftTorqueIdentifier);
		auto grfLeftLogger = ExternalWrench::initializeLogger();

		vector<ExternalWrench::Parameters> wrenchParameters;
		wrenchParameters.push_back(grfRightFootPar);
		wrenchParameters.push_back(grfLeftFootPar);

		LowPassSmoothFilter::Parameters grfFilterParam;
		grfFilterParam.numSignals = 9;
		grfFilterParam.memory = memory;
		grfFilterParam.delay = delay;
		grfFilterParam.cutoffFrequency = cutoffFreq;
		grfFilterParam.splineOrder = splineOrder;
		grfFilterParam.calculateDerivatives = false;
		grfRightFilter = new LowPassSmoothFilter (grfFilterParam);
		grfLeftFilter = new LowPassSmoothFilter (grfFilterParam);

		ros::NodeHandle n;
		l_pub = n.advertise<opensimrt_msgs::PointWrenchTimed>("grf_left/wrench_point", 1000);
		r_pub = n.advertise<opensimrt_msgs::PointWrenchTimed>("grf_right/wrench_point", 1000);

		lw_pub = n.advertise<geometry_msgs::WrenchStamped>("grf_left/wrench", 1000);
		rw_pub = n.advertise<geometry_msgs::WrenchStamped>("grf_right/wrench", 1000);
		lp_pub = n.advertise<geometry_msgs::PointStamped>("grf_left/point", 1000);
		rp_pub = n.advertise<geometry_msgs::PointStamped>("grf_right/point", 1000);

		common_pub = n.advertise<opensimrt_msgs::CommonTimed>("grf_both", 1000);


		double initial_time = ros::Time::now().toSec();

	}
	void onInit()
	{
		if (read_4_force_plates)
		{
			output.desired_label_order = {"ground_force1_px","ground_force1_py","ground_force1_pz","ground_force1_vx","ground_force1_vy","ground_force1_vz","ground_torque1_x","ground_torque1_y","ground_torque1_z","ground_force4_px","ground_force4_py","ground_force4_pz","ground_force4_vx","ground_force4_vy","ground_force4_vz","ground_torque4_x","ground_torque4_y","ground_torque4_z"};

		}
		else
		{
			output.desired_label_order = {"ground_force_px","ground_force_py","ground_force_pz","ground_force_vx","ground_force_vy","ground_force_vz","ground_torque_x","ground_torque_y","ground_torque_z","1_ground_force_px","1_ground_force_py","1_ground_force_pz","1_ground_force_vx","1_ground_force_vy","1_ground_force_vz","1_ground_torque_x","1_ground_torque_y","1_ground_torque_z"};
		}
		output.set(output_labels); // important is that this is done after I read the tableseries so that I have the columnnames set on the labels variable 	
		output_labels = output.desired_label_order;
		//since now I will have the correct output order, I need to make sure when someone asks me for the labels that I say the corrected order. 
		//alright, I remembered the code wrong. we actually unpack the table into a grfm type and pack it again, it was saying the wrong order, but the order it was publishing was correct all along. 
		Ros::CommonNode::onInit();
	}
	void pub_wrench_combined(const ros::Publisher& pub, const ExternalWrench::Input& wrench, double t, const std_msgs::Header h)
	{
		//double t = qTable.getIndependentColumn()[i] + time_offset;
		opensimrt_msgs::PointWrenchTimed msg;	

		//msg.wrench.force = get_as<geometry_msgs::Vector3>(wrench.force);
		//msg.wrench.torque = get_as<geometry_msgs::Vector3>(wrench.torque);
		msg.wrench = get_as_ros_wrench<ExternalWrench::Input>(wrench);
		msg.point = get_as<geometry_msgs::Point>(wrench.point);

		msg.time = t;

		ROS_DEBUG("created PointWrenchTimed msg ok.");
		msg.header = h;
		pub.publish(msg);
	}

	void pub_wrench(const ros::Publisher& pub, const ExternalWrench::Input& wrench, const std_msgs::Header h)
	{
		//double t = qTable.getIndependentColumn()[i] + time_offset;
		geometry_msgs::WrenchStamped w;

		//w.wrench.force = get_as<geometry_msgs::Vector3>(wrench.force);
		//w.wrench.torque = get_as<geometry_msgs::Vector3>(wrench.torque);
		w.wrench = get_as_ros_wrench(wrench);

		ROS_DEBUG("created Wrench msg ok.");
		w.header = h;
		pub.publish(w);
	}

	void pub_tf(string parent_frame_id, string child_frame_id, ExternalWrench::Input& wrench, const std_msgs::Header h)
	{
		geometry_msgs::TransformStamped transformStamped;
		transformStamped.header = h;
		transformStamped.header.frame_id = parent_frame_id;
		transformStamped.child_frame_id = child_frame_id;
		transformStamped.transform.translation = get_as<geometry_msgs::Vector3>(wrench.point);
		if (tf_apply_rotation)
			transformStamped.transform.rotation = Trial_and_error_G;
			//transformStamped.transform.rotation = TO_ROS_G;
		else
			transformStamped.transform.rotation = EYE_G; 
		br.sendTransform(transformStamped);

	}

	opensimrt_msgs::CommonTimed get_GRFMs_as_common_msg(ExternalWrench::Input grfmRight, ExternalWrench::Input grfmLeft, double t, double offsettime, std_msgs::Header h)
	{
		//TODO: NO LABELS FOR ORDER??
		//

		//OpenSim::TimeSeriesTable output;
		std::vector<double> p;
		//auto a = grfmOutput.right.toVector() ;
		//auto b = grfmOutput.left.toVector() ;
		auto a = grfmRight.toVector();
		auto b = grfmLeft.toVector();

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
		msg.offsettime = offsettime;
		msg.data.insert(msg.data.end(), p.begin(),p.end());
		return msg;
	}

	void pub_both_wrenches(double t, double offsettime, std_msgs::Header h)
	{
		// receives time:
		// get grf forces
		ROS_DEBUG_STREAM("T:" << t);
		auto grfRightWrench = ExternalWrench::getWrenchFromStorage(
				t-offsettime, grfRightLabels, *grfMotion);
		auto grfLeftWrench = ExternalWrench::getWrenchFromStorage(
				t-offsettime, grfLeftLabels, *grfMotion);

		//not sure why we need the filtered version since we are not really using it. 
		if (use_filter) {
			auto grfRightFiltered =
				grfRightFilter->filter({t, grfRightWrench.toVector()});
			grfRightWrench.fromVector(grfRightFiltered.x);
			auto grfLeftFiltered =
				grfLeftFilter->filter({t, grfLeftWrench.toVector()});
			grfLeftWrench.fromVector(grfLeftFiltered.x);
			if (!grfRightFiltered.isValid || !grfLeftFiltered.isValid) {
				return;
			}
		}

		//std_msgs::Header h;
		// I also need to publish the correct frame transformations!!!
		//ros::Time frameTime = ros::Time::now();
		//h.stamp = frameTime;

		h.frame_id = tf_right_foot_frame;
		pub_wrench_combined(r_pub, grfRightWrench,t, h);
		pub_wrench(rw_pub, grfRightWrench, h);
		h.frame_id = tf_left_foot_frame;
		pub_wrench_combined(l_pub, grfLeftWrench,t, h);
		pub_wrench(lw_pub, grfLeftWrench, h);
		if (publish_tf)
		{
			ROS_DEBUG_STREAM("Publishing tfs: " << tf_right_foot_frame << " " << tf_left_foot_frame << " using reference frame:" << tf_ref_frame);
			pub_tf(tf_ref_frame,tf_right_foot_frame,grfRightWrench,h);
			pub_tf(tf_ref_frame,tf_left_foot_frame,grfLeftWrench,h);
		}
		h.frame_id = "subject";
		common_pub.publish(get_GRFMs_as_common_msg(grfRightWrench,grfLeftWrench,t, offsettime,h));
		// now publish the tfs
	}
	void callback(const opensimrt_msgs::CommonTimedConstPtr& message){

		//IMPORTANT: gotta keep the header, of the messages won't synchronize properly.
		//LOOP needs to be implemented like functor
		double t = message->time;
		double offsettime = message->offsettime;
		ROS_DEBUG_STREAM("Am I getting the offset? "<< offsettime);
		pub_both_wrenches(t, offsettime, message->header);
	}
};
int main(int argc, char** argv) {
	try { //should be avoided.
		ros::init(argc, argv, "grf_mot_file_bridge");
		ros::NodeHandle nh("~");
		ROS_INFO_STREAM("Starting GRFM node");
		//this is completely wrong. I should be able to just play it. but then how do I know that these things are synchronized?
		Gfrm grfm;
		grfm.onInit();
		ros::spin();
	} 
	catch ( ros::Exception &e ) {
		ROS_ERROR("ROS Error occured: %s ", e.what());
	}
	catch (exception& e) {
		cout << e.what() << endl;
		ROS_ERROR_STREAM("nope." << e.what());
		return -1;
	}
	return 0;
}
