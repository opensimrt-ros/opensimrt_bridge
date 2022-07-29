#include "OpenSimUtils.h"
#include "SignalProcessing.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/LabelsSrv.h"
#include "ros/service_server.h"
#include "std_msgs/Header.h"
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyRequest.h"
#include "std_srvs/EmptyResponse.h"
#include <cstdlib>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;
std::vector<std::string> labels;

bool pudlished=false;
bool start=false;

bool update_labels(opensimrt_msgs::LabelsSrv::Request & req, opensimrt_msgs::LabelsSrv::Response& res )
{
	res.data = labels;
	pudlished = true;
	ROS_INFO_STREAM("CALLED LABELS SRV");
	return true;
}

bool start_me(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & res)
{
	start = true;
	return true;

}

void run() {
    // subject data
    //auto section = "TEST_ACCELERATION_GRFM_PREDICTION_FROM_FILE";
    ros::NodeHandle nh("~");
    string subject_dir, model_file, ik_file;
    nh.param<std::string>("model_file", model_file, "");
    nh.param<std::string>("ik_file", ik_file, "");
    
    // repeat cyclic motion X times
    int simulation_loops;
    nh.param<int>("simulation_loops", simulation_loops, 0);
    // remove last N samples in motion for smooth transition between loops
    int remove_n_last_rows;
    nh.param<int>("remove_n_last_rows", remove_n_last_rows, 0);

    //cheat
    double resample_period;
    nh.param<double>("resample_period", resample_period, 0.01);
    // setup model
    Object::RegisterType(Thelen2003Muscle());
    Model model(model_file);
    model.initSystem();

    // get kinematics as a table with ordered coordinates
    auto qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
            model, ik_file, resample_period); ///this is resampling!!!!!
    ROS_WARN_STREAM("The precision of this algorithm and data downstream is inflated. IK data was resampled! ");

    ROS_INFO_STREAM("Number of variables measured: " << qTable.getRowAtIndex(0).size());
    // remove last rows in qTable
    for (int i = 0; i < remove_n_last_rows; ++i)
        qTable.removeRow(qTable.getIndependentColumn().back());

    ros::NodeHandle n;
    ros::Publisher re_pub = nh.advertise<opensimrt_msgs::CommonTimed>("r_data", 1000);
    ros::Publisher re_pub2 = nh.advertise<opensimrt_msgs::CommonTimed>("r_data2", 1000);
    //ros::Publisher labels_pub = n.advertise<opensimrt_msgs::Labels>("r_labels", 1000, true); //latching topic
    ros::ServiceServer gets_labels = nh.advertiseService("out_labels", update_labels);
    ros::ServiceServer start_publishing = nh.advertiseService("start", start_me);
    ros::Rate r(1);
    labels = qTable.getColumnLabels();
    while(!(pudlished && start))
    {
	ros::spinOnce();
	r.sleep();
   	ROS_INFO_STREAM("stuck"); 
    }
   
    // this is wrong.
    /*opensimrt_msgs::Labels l_msg;
    l_msg.data = labels;
    std_msgs::Header h;
    h.stamp = ros::Time::now();
    h.frame_id = "subject";
    l_msg.header = h;	
    labels_pub.publish(l_msg); */
    //ros::spinOnce();
    
    ros::Rate rate(1/resample_period);
    // repeat the simulation `simulationLoops` times
    int simulationLimit = qTable.getNumRows() * simulation_loops;

    
    ROS_INFO_STREAM("Simulation limit:" << simulationLimit );

    double time_error_threshold = 10*resample_period; // in seconds
    double initial_time = ros::Time::now().toSec();

    for (int j = 0; j < simulation_loops; j++) {
	    double time_offset = j * (qTable.getIndependentColumn().back()+resample_period); // we need resample period or 
	    ROS_INFO_STREAM("Time offset:" << time_offset );
	    for (int i = 0; i < qTable.getNumRows(); i++) {
		// get raw pose from table
		ROS_DEBUG("Get raw pose from table");
		auto qqqqq = qTable.getRowAtIndex(i);
		opensimrt_msgs::CommonTimed msg;
		std_msgs::Header h;
		
		double t = qTable.getIndependentColumn()[i] + time_offset;
			
		//msg.data.push_back(t);
		msg.time = t;

		for (auto ele: qqqqq)
		{
			ROS_DEBUG_STREAM("Element: " << ele);
			msg.data.push_back(ele);
		}
		ROS_DEBUG("finished reading table row into msg.");


    		//labels_pub.publish(l_msg); 


		//auto qRaw = qTable.getRowAtIndex(i).getAsVector();
		//ROS_DEBUG("DD");

		h.frame_id = "subject";
		ros::Time frameTime = ros::Time::now();
		double time_difference = frameTime.toSec() - initial_time;
		if ( std::abs(time_difference - t) > time_error_threshold )
		{
			
			ROS_WARN_STREAM("time difference" << time_difference -t << " exceeds threshold: " << time_error_threshold);
			ROS_INFO_STREAM("Frame time - initial_time " << time_difference << " Simulation time" << t );
		}
		h.stamp = frameTime;
		msg.header = h;
		re_pub.publish(msg);
		re_pub2.publish(msg);
		ros::spinOnce();
		rate.sleep();
	    }
    }

}

int main(int argc, char** argv) {
    try { //should be avoided.
	ros::init(argc, argv, "ik_file_bridge");

        run();
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
