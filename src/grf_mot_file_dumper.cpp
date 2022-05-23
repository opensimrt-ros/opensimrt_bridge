#include "InverseDynamics.h"
#include "OpenSimUtils.h"
#include "SignalProcessing.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include "geometry_msgs/Vector3.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "opensimrt_msgs/PointWrenchTimed.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "ros/subscriber.h"
#include "std_msgs/Header.h"
#include <SimTKcommon/SmallMatrix.h>
#include <cstdlib>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

//! oh, yeah, this guy.
template <class T>
T get_as(const SimTK::Vec3& v)
{
	//geometry_msgs::Vector3 outvec;
	T outvec;
	outvec.x = v[0];
	outvec.y = v[1];
	outvec.z = v[2];
	return outvec;
}


class Gfrm
{
	//some important variables should be here
	LowPassSmoothFilter* grfRightFilter, *grfLeftFilter;
    	Storage* grfMotion;
        bool use_filter;
	vector<string> grfRightLabels, grfLeftLabels; 
//publishers
	ros::Publisher r_pub, l_pub;	
	public:
		Gfrm()
		{
    // subject data
    //auto section = "TEST_ID_FROM_FILE";
    ros::NodeHandle nh("~");
    string grf_mot_file;
    nh.param<std::string>("grf_mot_file", grf_mot_file, "");
    
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
    l_pub = n.advertise<opensimrt_msgs::PointWrenchTimed>("grf_left", 1000);
    r_pub = n.advertise<opensimrt_msgs::PointWrenchTimed>("grf_right", 1000);
    
    double initial_time = ros::Time::now().toSec();
       
		}
void pub_wrench(const ros::Publisher& pub, const ExternalWrench::Input& wrench, double t, const std_msgs::Header h)
{
		//double t = qTable.getIndependentColumn()[i] + time_offset;
	        opensimrt_msgs::PointWrenchTimed msg;	
		
		msg.wrench.force = get_as<geometry_msgs::Vector3>(wrench.force);
		msg.wrench.torque = get_as<geometry_msgs::Vector3>(wrench.torque);
		msg.point = get_as<geometry_msgs::Point>(wrench.point);

		msg.time = t;

		ROS_DEBUG("created PointWrenchTimed msg ok.");
		msg.header = h;
		pub.publish(msg);
}

void pub_both_wrenches(double t)
{
    // receives time:
    // get grf forces
    	ROS_INFO_STREAM("T" << t);
        auto grfRightWrench = ExternalWrench::getWrenchFromStorage(
                t, grfRightLabels, *grfMotion);
        auto grfLeftWrench = ExternalWrench::getWrenchFromStorage(
                t, grfLeftLabels, *grfMotion);

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

	std_msgs::Header h;
	h.frame_id = "subject";
	ros::Time frameTime = ros::Time::now();
	h.stamp = frameTime;

	pub_wrench(r_pub, grfRightWrench,t, h);
	pub_wrench(l_pub, grfLeftWrench,t, h);

}
void operator() (const opensimrt_msgs::CommonTimedConstPtr& message){

    //LOOP needs to be implemented like functor
    double t = message->time;
    pub_both_wrenches(t);
}
};
int main(int argc, char** argv) {
    try { //should be avoided.
	ros::init(argc, argv, "grf_mot_file_bridge");
	ros::NodeHandle n;
	ROS_INFO_STREAM("Starting GRFM node");
	//this is completely wrong. I should be able to just play it. but then how do I know that these things are synchronized?
	ros::Subscriber sub = n.subscribe<opensimrt_msgs::CommonTimed>("r_data", 1, Gfrm());
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
