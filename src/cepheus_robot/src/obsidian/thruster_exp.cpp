#include <fstream>
#include <signal.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>
// #include "digital_filter.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;

sig_atomic_t volatile g_request_shutdown = 0;
// Replacement SIGINT handler
void ctrl_c_handler(int sig) {
	g_request_shutdown = 1;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "erros_control_node", ros::init_options::NoSigintHandler);
	signal(SIGINT, ctrl_c_handler);
	ros::NodeHandle nh;
	ros::CallbackQueue main_queue;
	nh.setCallbackQueue(&main_queue);

	ros::Publisher thrust_pub = nh.advertise<geometry_msgs::Vector3Stamped>("cmd_thrust", 1);

	double rate;
	ros::param::param<double>("~loop_rate", rate, 200);
	ros::Rate loop_rate(rate);

	std_msgs::Float64 temp_msg;

	geometry_msgs::Vector3Stamped thrust_vector;

    ifstream data("/home/cepheus/cepheus_ws/src/cepheus_robot/src/csvs/identification.csv");
    string line;
    vector<vector<string> > parsedCsv;
    while(getline(data, line)) {
        stringstream lineStream(line);
        string cell;
        vector<string> parsedRow;
        while(getline(lineStream, cell, ',')) {
            parsedRow.push_back(cell);
        }
        parsedCsv.push_back(parsedRow);
    }

    int i = 0;
	ros::Time curr_time, t_beg = ros::Time::now();
	ros::Duration all_time;
    double secs;

	while (!g_request_shutdown) {
		curr_time = ros::Time::now();
		all_time = curr_time - t_beg;
        secs = all_time.sec + all_time.nsec * pow(10, -9);
        
        if (i < parsedCsv.size() && secs < 23.365) {
            thrust_vector.vector.x = stof(parsedCsv[i][0]);
            thrust_vector.vector.y = stof(parsedCsv[i][1]);
            thrust_vector.vector.z = stof(parsedCsv[i][2]);
            cout << secs << endl;
        } else {
            if (i < parsedCsv.size())
                cout << "over: " << secs << endl;
            thrust_vector.vector.x = 0.0;
            thrust_vector.vector.y = 0.0;
            thrust_vector.vector.z = 0.0;
        }
        i++;

		thrust_pub.publish(thrust_vector);

		main_queue.callAvailable(ros::WallDuration(0.0));
		loop_rate.sleep();
	}

	return 0;
}
