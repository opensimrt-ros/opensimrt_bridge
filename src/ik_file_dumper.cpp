#include "OpenSimUtils.h"
#include "SignalProcessing.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/LabelsSrv.h"
#include "ros/service_server.h"
#include "ros/time.h"
#include "std_msgs/Header.h"
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyRequest.h"
#include "std_srvs/EmptyResponse.h"
#include <cstddef>
#include <cstdlib>
#include <string>
#include <vector>
#include "opensimrt_bridge/IkPublisher.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

class TablePublisher
{
	public:
		std::vector<std::string> labels;
		bool pudlished=false;
		bool first_run=true;
		TimeSeriesTable qTable;
		int simulation_loops, executed_loops = 0;
		double resample_period;
		ros::Rate *rate;
		double rate_divider;
		double time_error_threshold; // in seconds
		int i = 0, j = 0;
		double time_offset = 0;
		ros::Publisher re_pub, re_pub2;
		ros::ServiceServer gets_labels;
		double initial_time = 0;
		//uint32_t start_secs, start_nsecs;
		//uint32_t stop_secs, stop_nsecs;
		int start_secs, start_nsecs;
		int stop_secs, stop_nsecs;
		ros::Time start_time, stop_time;
		double table_initial_time, table_final_time;
		IkPublisher ik_pub;
		bool async_run, running;
		std::vector<bool> include_list;
		TablePublisher()
		{
			ros::NodeHandle nh("~");
			string subject_dir, model_file, ik_file;
			nh.param<std::string>("model_file", model_file, "");
			nh.param<std::string>("ik_file", ik_file, "");
			// repeat cyclic motion X times
			nh.param<int>("simulation_loops", simulation_loops, 0);
			// remove last N samples in motion for smooth transition between loops
			int remove_n_last_rows;
			nh.param<int>("remove_n_last_rows", remove_n_last_rows, 0);

			nh.param<int>("start_at_secs", start_secs, 1668695814);
			nh.param<int>("start_at_nsecs", start_nsecs, 643890142);

			start_time = ros::Time{static_cast<uint32_t>(start_secs),static_cast<uint32_t>(start_nsecs)};

			nh.param<int>("stop_at_secs", stop_secs, 1668695818);
			nh.param<int>("stop_at_nsecs", stop_nsecs, 643890142);

			stop_time = ros::Time {static_cast<uint32_t>(stop_secs),static_cast<uint32_t>(stop_nsecs)};
			//cheat
			nh.param<double>("resample_period", resample_period, 0.01);

			nh.param<double>("rate_divider", rate_divider, 1);
			nh.param<bool>("async_run", async_run, false);
			running = false;
			// setup model
			Object::RegisterType(Thelen2003Muscle());
			Model model(model_file);
			model.initSystem();

			// get kinematics as a table with ordered coordinates
			qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
					model, ik_file, resample_period); ///this is resampling!!!!!
			table_initial_time = qTable.getIndependentColumn().front();
			table_final_time = qTable.getIndependentColumn().back();
			ROS_WARN_STREAM("The precision of this algorithm and data downstream is inflated. IK data was resampled! ");

			ROS_INFO_STREAM("Number of variables measured: " << qTable.getRowAtIndex(0).size());
			// remove last rows in qTable
			for (int i = 0; i < remove_n_last_rows; ++i)
				qTable.removeRow(qTable.getIndependentColumn().back());

			ros::NodeHandle n;
			//ros::Publisher re_pub = nh.advertise<opensimrt_msgs::CommonTimed>("r_data", 1000);
			//ros::Publisher re_pub2 = nh.advertise<opensimrt_msgs::CommonTimed>("r_data2", 1000);
			//re_pub = nh.advertise<opensimrt_msgs::CommonTimed>("r_data", 1000);
			re_pub2 = nh.advertise<opensimrt_msgs::CommonTimed>("r_data2", 1000);
			//ros::Publisher labels_pub = n.advertise<opensimrt_msgs::Labels>("r_labels", 1000, true); //latching topic
			//gets_labels = nh.advertiseService("out_labels", &TablePublisher::update_labels, this);
			ik_pub.output_labels = qTable.getColumnLabels();
			ik_pub.numSignals= ik_pub.output_labels.size();
			ik_pub.onInit();
			// this is wrong.
			/*opensimrt_msgs::Labels l_msg;
			  l_msg.data = labels;
			  std_msgs::Header h;
			  h.stamp = ros::Time::now();
			  h.frame_id = "subject";
			  l_msg.header = h;	
			  labels_pub.publish(l_msg); */
			//ros::spinOnce();
			double rate_frequency = 1/rate_divider/resample_period;
			ROS_INFO_STREAM("Rate rate_frequency set to: "<< rate_frequency);
			//ros::Rate rate(rate_frequency);
			rate = new ros::Rate(rate_frequency);
			// repeat the simulation `simulationLoops` times
			int simulationLimit = qTable.getNumRows() * simulation_loops;


			ROS_INFO_STREAM("Simulation limit:" << simulationLimit );

			time_error_threshold = 10*resample_period; // in seconds



			// publishes zeros in some columns
			//
			
			std::vector<std::string> columns = qTable.getColumnLabels();

			// needs to be a param list

			//std::vector<std::string> exclude_list{"hip","adomen"};
			std::vector<std::string> exclude_list;
			nh.getParam("exclude_list", exclude_list);
			if (exclude_list.size() == 0)
			{
				ROS_WARN("publishing everything");
			}
			else
			{
				for (auto excl:exclude_list)
					ROS_WARN_STREAM("I will replace joint" << excl << " for zeros");
			}

			include_list.reserve(qTable.getNumColumns()); // why not

			for (std::size_t i=0; i< columns.size();i++)
			{
				std::string dof_name= columns[i];
				include_list[i] = true;
				for (auto exclude_dof:exclude_list)
				{
					if (exclude_dof.compare(dof_name) == 0 )
					{	
						ROS_WARN_STREAM("will exclude" << columns[i]);
						include_list[i]= false;
					}
				}

			}


		}
		bool update_labels(opensimrt_msgs::LabelsSrv::Request & req, opensimrt_msgs::LabelsSrv::Response& res )
		{
			res.data = labels;
			pudlished = true;
			ROS_INFO_STREAM("CALLED LABELS SRV");
			return true;
		}

		bool start_me(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & res)
		{
			if (async_run)
				running = true;
			else
				run();
			return true;

		}

		bool start_at(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & res)
		{
			//This is severely untested stuff. it may work by coincidence alone
			auto time_now = ros::Time::now();
			auto start_time_nsecs = start_time.toNSec();
			auto stop_time_nsecs = stop_time.toNSec();
			bool started = false;
			while(ros::ok())
			{
				time_now = ros::Time::now();
				auto time_now_nsecs = time_now.toNSec();
				if ((time_now_nsecs>start_time_nsecs && time_now_nsecs<stop_time_nsecs) && !started)
				{
					initial_time = time_now_nsecs;
					started = true;
				}
				if ((time_now_nsecs>stop_time_nsecs || time_now_nsecs<start_time_nsecs) && started)
				{
					started = false;
				}
				if (started)
				{
					double publishing_time = (time_now_nsecs - initial_time)/1000000000.0;
					publish_once();
					i++;
					if (!ros::ok())
						return false;
				}
				ros::spinOnce();
				rate->sleep();
			}
			return true;

		}
		bool publish_one_frame(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & res)
		{
			if (qTable.getNumRows() == i)
			{
				j++;
				initial_time = ros::Time::now().toNSec();
				time_offset = j * (qTable.getIndependentColumn().back()+resample_period); // we need resample period or 
				i= 0;
				publish_once();
			}
			else
			{
				publish_once();

				i++;
			}
			return true;

		}
		void publish_table(RowVector qqqqq, double t)
		{
			opensimrt_msgs::CommonTimed msg;
			std_msgs::Header h;
			//msg.data.push_back(t);
			// Possibly breaking change!!!!! I will make t the table time, instead of simulation time. Because it loops and then when trying to get the accurate grf from the table, I don't knwo what it the offset time. if things break, change common message to include the offset as well and then remove this offset in grf_mot_file_dumper from opensimrt_bridge to get the same results.
			msg.time = t;
			msg.offsettime = time_offset;


			//for (auto ele: qqqqq)
			for (size_t i=0;i<qqqqq.size(); i++)
			{
				auto ele = qqqqq[i];
				if (include_list[i])
				{
					ROS_DEBUG_STREAM("Element: " << ele);
					msg.data.push_back(ele);}
				else
				{
					//ROS_DEBUG_STREAM("skipped" << i);
					//I am using qqqqq directly in the ik_pub ....
					qqqqq[i] = 0;
					msg.data.push_back(0);
				}
			}
			ROS_DEBUG("finished reading table row into msg.");


			//labels_pub.publish(l_msg); 


			//auto qRaw = qTable.getRowAtIndex(i).getAsVector();
			//ROS_DEBUG("DD");

			h.frame_id = "subject";
			ros::Time frameTime = ros::Time::now();
			double time_difference = (frameTime.toNSec() - initial_time )/rate_divider/1E9 + table_initial_time + time_offset;
			if ( std::abs(time_difference - t) > time_error_threshold )
			{
				ROS_INFO_STREAM("Time offset:" << time_offset );

				ROS_WARN_STREAM("time difference" << time_difference -t << " exceeds threshold: " << time_error_threshold);
				ROS_INFO_STREAM("Frame time - initial_time " << time_difference << " Simulation time" << t );
			}
			h.stamp = frameTime;
			msg.header = h;
			//re_pub.publish(msg);
			//ROS_INFO_STREAM(qqqqq);
			ik_pub.publish(t,~qqqqq,time_offset);
			re_pub2.publish(msg);

		}
		void publish_once(double time)
		{
			double table_time =time+table_initial_time;
			if (table_time > table_final_time)
			{
				ROS_WARN_STREAM_ONCE("Time: " << table_time << " exceeds maximum time in source table:" << table_final_time << endl << ". Publishing nothing" );
			}
			else
			{
				int k = qTable.getRowIndexAfterTime(table_time);
				ROS_DEBUG_STREAM("table_time" << table_time << "time:" << time << "k" << k);
				RowVector qqqqq = qTable.getRowAtIndex(k);
				publish_table(qqqqq, table_time);
			}
		}
		void publish_once()
		{
			// get raw pose from table
			ROS_DEBUG("Get raw pose from table");
			RowVector qqqqq = qTable.getRowAtIndex(i);

			double t = qTable.getIndependentColumn()[i] + time_offset;
			publish_table(qqqqq,t);


		}
		void run() 
		{
			// subject data
			//auto section = "TEST_ACCELERATION_GRFM_PREDICTION_FROM_FILE";
			ros::Rate r(1);
			while(!ik_pub.published_labels_at_least_once)
			{
				ros::spinOnce();
				r.sleep();
				ROS_INFO_STREAM("stuck"); 
			}
			inner_loop();

		}
		void inner_loop()
		{
			for (j = executed_loops; j < simulation_loops + executed_loops; j++) {
				initial_time = ros::Time::now().toNSec();
				time_offset = j * (qTable.getIndependentColumn().back()+resample_period); // we need resample period or 
				ROS_INFO_STREAM("Time offset:" << time_offset );
				for (i = 0; i < qTable.getNumRows(); i++) {
					publish_once();
					ros::spinOnce();
					if (!ros::ok())
						return;
					rate->sleep();
					// will execute only once!
					//return;
				}
			}
			executed_loops += simulation_loops;

		}
};

int main(int argc, char** argv) {
	try { //should be avoided.
		ros::init(argc, argv, "ik_file_bridge");
		bool debug = false;
		if( debug && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
			ros::console::notifyLoggerLevelsChanged();
		}
		TablePublisher myTablePub;
		ros::NodeHandle nh("~");
		myTablePub.initial_time = ros::Time::now().toNSec(); 
		ros::ServiceServer start_at = nh.advertiseService("start_at", &TablePublisher::start_at, &myTablePub);
		ros::ServiceServer start_publishing = nh.advertiseService("start", &TablePublisher::start_me, &myTablePub);
		ros::ServiceServer publishing_one = nh.advertiseService("step", &TablePublisher::publish_one_frame, &myTablePub);
		if (myTablePub.async_run)
		{
			ros::Rate async_rate(10);
			while(ros::ok())
			{
				if (myTablePub.running)
				{
					myTablePub.inner_loop();
					myTablePub.running= false;
				}
				ros::spinOnce();
				async_rate.sleep();
			}
		}
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
