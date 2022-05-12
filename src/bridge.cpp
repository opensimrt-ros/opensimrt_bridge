/**
 * -----------------------------------------------------------------------------
 * Copyright 2019-2021 OpenSimRT developers.
 *
 * This file is part of OpenSimRT.
 *
 * OpenSimRT is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * OpenSimRT is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * OpenSimRT. If not, see <https://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------------
 *
 * @file TestAccelerationGRFMPredictionFromFile.cpp
 *
 * @brief Test the GRF&M prediction method with the AccelerationBased
 * PhaseDetector. Increases the simulation time by repeating the recorded motion
 * X times, in order to provide enough time for the detector to adapt.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#include "experimental/AccelerationBasedPhaseDetector.h"
#include "experimental/GRFMPrediction.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "SignalProcessing.h"
#include "Utils.h"
#include "Visualization.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Labels.h"
#include "std_msgs/Header.h"
#include <cstdlib>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_ACCELERATION_GRFM_PREDICTION_FROM_FILE";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");

    // repeat cyclic motion X times
    auto simulationLoops = ini.getInteger(section, "SIMULATION_LOOPS", 0);
    // remove last N samples in motion for smooth transition between loops
    auto removeNLastRows =
            ini.getInteger(section, "REMOVE_N_LAST_TABLE_ROWS", 0);
    // setup model
    Object::RegisterType(Thelen2003Muscle());
    Model model(modelFile);
    model.initSystem();

    // get kinematics as a table with ordered coordinates
    auto qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
            model, ikFile, 0.01);

    ROS_INFO_STREAM("UUUUU" << qTable.getRowAtIndex(0).size());
    // remove last rows in qTable
    for (int i = 0; i < removeNLastRows; ++i)
        qTable.removeRow(qTable.getIndependentColumn().back());

    // mean delay
    int sumDelayMS = 0;
    int sumDelayMSCounter = 0;

    ros::NodeHandle n;
    ros::Publisher re_pub = n.advertise<opensimrt_msgs::CommonTimed>("r_data", 1000);
    ros::Publisher labels_pub = n.advertise<opensimrt_msgs::Labels>("r_labels", 1000, true); //latching topic

    std::vector<std::string> labels = qTable.getColumnLabels();
    opensimrt_msgs::Labels l_msg;
    l_msg.data = labels;
    std_msgs::Header h;
    h.stamp = ros::Time::now();
    h.frame_id = "subject";
    l_msg.header = h;	
    labels_pub.publish(l_msg); 
    //ros::spinOnce();
    
    ros::Rate rate(100);
    // repeat the simulation `simulationLoops` times
    int simulationLimit = qTable.getNumRows() * simulationLoops;

    
    ROS_INFO_STREAM("Simulation limit:" << simulationLimit );

    double time_error_threshold = 0.1; // in seconds
    double initial_time = ros::Time::now().toSec();

    for (int j = 0; j < simulationLoops; j++) {
	    double time_offset = j * qTable.getIndependentColumn().back(); 
	    ROS_INFO_STREAM("Time offset:" << time_offset );
	    for (int i = 0; i < qTable.getNumRows(); i++) {
		// get raw pose from table
		ROS_DEBUG("get raw pose from table");
		auto qqqqq = qTable.getRowAtIndex(i);
		opensimrt_msgs::CommonTimed msg;
		std_msgs::Header h;
		
		double t = qTable.getIndependentColumn()[i] + time_offset;
			
		//msg.data.push_back(t);
		msg.time = t;

		for (auto ele: qqqqq)
		{
			ROS_DEBUG_STREAM("ELE"<< ele);
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
			
			ROS_WARN_STREAM("time difference" <<time_difference -t << " exceeds threshold: " << time_error_threshold);
			ROS_INFO_STREAM("Frame time - initial_time " << time_difference << " Simulation time" << t );
		}
		h.stamp = frameTime;
		msg.header = h;
		re_pub.publish(msg);

		rate.sleep();
	    }
    }
    cout << "Mean delay: " << double(sumDelayMS) / sumDelayMSCounter << " ms"
         << endl;

}

int main(int argc, char** argv) {
    try {
	ros::init(argc, argv, "osrtbridge");

        run();
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}
