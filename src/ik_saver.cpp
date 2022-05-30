#include "OpenSimUtils.h"
#include "SignalProcessing.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
//#include "opensimrt_msgs/Labels.h"
#include "opensimrt_msgs/LabelsSrv.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "std_msgs/Header.h"
#include <cstdlib>
#include <string>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

TimeSeriesTable myTable;

void callback(const opensimrt_msgs::CommonTimedConstPtr& msg)
{
	myTable.appendRow(msg->time, msg->data.begin(), msg->data.end());
}

/*void update_labels(const opensimrt_msgs::LabelsPtr& msg)
{
	std::vector<double> empty{};
	for(string label: msg->data)
		myTable.appendColumn(label,empty);

}*/

int main(int argc, char** argv) {
    try { //should be avoided.
	ros::init(argc, argv, "ik_file_bridge");
    ros::NodeHandle nh("~");

    ros::NodeHandle n;
  //  ros::Subscriber labels_sub = n.subscribe<opensimrt_msgs::Labels>("r_labels", 1000, update_labels); 
    ros::ServiceClient labels_cli = n.serviceClient<opensimrt_msgs::LabelsSrv>("/gets_labels");
    //std::vector<std::string> labels = qTable.getColumnLabels();

    //calls service and receives answer which is labels
    opensimrt_msgs::LabelsSrv srv;
    if(labels_cli.call(srv))
    {
	//
	//answer is in srv.response
	std::vector<std::string> labels = srv.response.data;
	// I can create the table with an empty number of observations, if that's possible
	vector<double> empty{};
	for (string label:labels)
	{
		myTable.appendColumn(label,empty);
	}
	// now i need to create the subscriber so i can start reading the matrix
        ros::Subscriber re_sub = n.subscribe<opensimrt_msgs::CommonTimed>("r_data", 1000, callback);
    }
    else
    {
	ROS_WARN_STREAM("Couldnt get label srv response");
    }

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
