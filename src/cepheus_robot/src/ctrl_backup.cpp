#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <string>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>

#include "cepheus_hardware.h"

bool flag = false;

//send messages to cepheus_ctl node to load-start controllers
void timerCallback(const ros::TimerEvent& even, ros::Publisher &ctl_pub){

	std_msgs::String msg;
	msg.data = "OK";

	ctl_pub.publish(msg);
}

// res: nothing to return because failure stops 'ctrl' and interface hangs
void startCtrl(const std_msgs::StringConstPtr &msg, 
		ros::NodeHandle &n, 
		std::vector<std::string> &ctrl_names)
{
	if(flag == false && ((msg->data).compare("START_CTL") == 0)){
		flag = true;

		//reload controllers libraries
		ros::service::waitForService("/controller_manager/reload_controller_libraries", -1);
		ros::ServiceClient reload_controller = n.serviceClient<controller_manager_msgs::ReloadControllerLibraries>(
				"/controller_manager/reload_controller_libraries");	

		controller_manager_msgs::ReloadControllerLibraries reload_controllers_libraries_msg;
		reload_controllers_libraries_msg.request.force_kill = true;
		reload_controller.call(reload_controllers_libraries_msg);
		assert(reload_controllers_libraries_msg.response.ok);

		ROS_INFO("SUCCEDED_TO_RELOAD_CONTROLLERS_LIBRARIES");


		// try to load controllers
		ros::service::waitForService("/controller_manager/load_controller", -1);
		ros::ServiceClient load_controller = n.serviceClient<controller_manager_msgs::LoadController>(
				"/controller_manager/load_controller");

		std::vector<std::string>::iterator it;
		for (it = ctrl_names.begin(); it != ctrl_names.end(); it++) {
			ROS_WARN("Loading %s...",(*it).c_str());

			controller_manager_msgs::LoadController load_controller_msg;
			load_controller_msg.request.name = *it;
			load_controller.call(load_controller_msg);
			assert(load_controller_msg.response.ok);
		}

		ROS_WARN("Loaded controllers!");

		// try to unload controllers
		ros::service::waitForService("/controller_manager/unload_controller", -1);
		ros::ServiceClient unload_controller = n.serviceClient<controller_manager_msgs::UnloadController>(
				"/controller_manager/unload_controller");

		for (it = ctrl_names.begin(); it != ctrl_names.end(); it++) {
			ROS_WARN("UnLoading %s...",(*it).c_str());

			controller_manager_msgs::UnloadController unload_controller_msg;
			unload_controller_msg.request.name = *it;
			unload_controller.call(unload_controller_msg);
			assert(unload_controller_msg.response.ok);
		}

		ROS_WARN("Unloaded controllers!");
// try to load controllers
		for (it = ctrl_names.begin(); it != ctrl_names.end(); it++) {
			ROS_WARN("Loading %s...",(*it).c_str());

			controller_manager_msgs::LoadController load_controller_msg;
			load_controller_msg.request.name = *it;
			load_controller.call(load_controller_msg);
			assert(load_controller_msg.response.ok);
		}

		ROS_WARN("Loaded controllers!");



		// try to start controllers
		ros::service::waitForService("/controller_manager/switch_controller", -1);
		ros::ServiceClient start_controller = n.serviceClient<controller_manager_msgs::SwitchController>(
				"/controller_manager/switch_controller");
		controller_manager_msgs::SwitchController start_controller_msg;

		//std::vector<std::string> strVec;
		//strVec.push_back("joint_state_publisher");
		//strVec.push_back("reaction_wheel_effort_controller");
		//strVec.push_back("left_shoulder_position_controller");
		//strVec.push_back("/left_elbow_position_controller");


		start_controller_msg.request.start_controllers = ctrl_names;
		start_controller_msg.request.strictness = start_controller_msg.request.STRICT;

		ROS_WARN("About to call switch");	
		if(start_controller.call(start_controller_msg))
			ROS_WARN("CALLED");
		else
			ROS_WARN("CALL FAILED");
		ROS_WARN("Called switch!");

		assert(start_controller_msg.response.ok);

		ROS_WARN("Started controllers");

		//send OK response to interface
		ros::Publisher ctl_pub = n.advertise<std_msgs::String>("load_start_controllers_response",10);
		ros::Timer timer = n.createTimer(ros::Duration(1), boost::bind(&timerCallback, _1, boost::ref(ctl_pub)));

		ROS_INFO("I SENT OK TO INTERFACE");
	}	


}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cepheus_ctrl");
	ros::NodeHandle n;

	//PANOS NKWSTAS effort to load/start controllers with topic

	std::vector<std::string> ctrl_names;
	for (int i=1;i<argc;++i) ctrl_names.push_back(std::string(argv[i]));

	//have to declare message type cause I pass const pointer to the callback
	ros::Subscriber ctl_sub = n.subscribe <std_msgs::String> ("load_start_controllers", 10, boost::bind(&startCtrl, _1, boost::ref(n), boost::ref(ctrl_names)) );


	//in order to process the message from the topic coming from cepheus_ctl node
	ROS_INFO("Ready to load, start controllers");

	/*
	 *  ros::spin() will not return until the node has been shutdown, 
	 *  either through a call to ros::shutdown() or a Ctrl-C. 
	 */
	ros::spin();

	/*ros::ServiceServer service = n.advertiseService<std_srvs::Empty::Request,std_srvs::Empty::Response>
	  ("start_ctrl", 
	  boost::bind(&startCtrl,
	  _1,_2,
	  boost::ref(n),
	  boost::ref(ctrl_names)));*/


	ros::ServiceClient stop_controller = n.serviceClient<controller_manager_msgs::SwitchController>(
			"/controller_manager/switch_controller");
	controller_manager_msgs::SwitchController stop_controller_msg;
	stop_controller_msg.request.stop_controllers = ctrl_names;
	stop_controller_msg.request.start_controllers = std::vector<std::string>();
	stop_controller_msg.request.strictness = 2; // strictness
	stop_controller.call(stop_controller_msg);
	assert(stop_controller_msg.response.ok);

	ROS_INFO("Stopped controllers");

	ros::service::waitForService("/controller_manager/unload_controller", -1);
	ros::ServiceClient unload_controller = n.serviceClient<controller_manager_msgs::UnloadController>(
			"/controller_manager/unload_controller");
	std::vector<std::string>::iterator it;
	for (it = ctrl_names.begin(); it != ctrl_names.end(); it++) {
		controller_manager_msgs::UnloadController unload_controller_msg;
		unload_controller_msg.request.name = *it;
		unload_controller.call(unload_controller_msg);
		assert(unload_controller_msg.response.ok);
	}

	ROS_INFO("Unloaded controllers");

	return 0;
}
