#include <string>
#include <boost/bind.hpp>
#include <std_msgs/String.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ControllerState.h>

#include "cepheus_hardware.h"
#include "cepheus_ctrl.h"

#define MSG_NUM 100

bool standard_ctrls_started = false;
bool left_shoulder_ctrl_started = false;
bool left_elbow_ctrl_started = false;
bool right_shoulder_ctrl_started = false;
bool right_elbow_ctrl_started = false;


int num_of_ctrls = 0;
pid_t pid;
std::vector<std::string>controllers_to_start;

std::vector<std::string> split(std::string strToSplit, char delimeter)
{
	std::stringstream ss(strToSplit);
	std::string item;
	std::vector<std::string> splittedStrings;
	while (std::getline(ss, item, delimeter))
	{
		splittedStrings.push_back(item);
	}
	return splittedStrings;
}

bool reloadControllerLibraries(ros::NodeHandle &n){

	ros::service::waitForService("/controller_manager/reload_controller_libraries", -1);
	ros::ServiceClient reload_controller = n.serviceClient<controller_manager_msgs::ReloadControllerLibraries>(
			"/controller_manager/reload_controller_libraries");

	controller_manager_msgs::ReloadControllerLibraries reload_controllers_libraries_msg;
	reload_controllers_libraries_msg.request.force_kill = true;
	reload_controller.call(reload_controllers_libraries_msg);
	return reload_controllers_libraries_msg.response.ok;

}


bool loadController(ros::NodeHandle &n, std::string c_name){

	ROS_WARN(c_name.c_str());

	ros::service::waitForService("/controller_manager/load_controller", -1);
	ros::ServiceClient load_controller = n.serviceClient<controller_manager_msgs::LoadController>(
			"/controller_manager/load_controller");

	controller_manager_msgs::LoadController load_controller_msg;
	load_controller_msg.request.name = c_name;
	load_controller.call(load_controller_msg);
	return load_controller_msg.response.ok;
}

bool unloadController(ros::NodeHandle &n, std::string c_name){

	ROS_WARN(c_name.c_str());

	ros::service::waitForService("/controller_manager/load_controller", -1);
	ros::ServiceClient load_controller = n.serviceClient<controller_manager_msgs::UnloadController>(
			"/controller_manager/unload_controller");

	controller_manager_msgs::UnloadController unload_controller_msg;
	unload_controller_msg.request.name = c_name;
	load_controller.call(unload_controller_msg);
	return unload_controller_msg.response.ok;
}

bool startControllers(ros::NodeHandle &n, std::vector<std::string>& controllers_to_start){

	ros::service::waitForService("/controller_manager/switch_controller", -1);
	ros::ServiceClient switch_controller = n.serviceClient<controller_manager_msgs::SwitchController>(
			"/controller_manager/switch_controller");

	controller_manager_msgs::SwitchController switch_controller_msg;
	switch_controller_msg.request.start_controllers = controllers_to_start;
	switch_controller_msg.request.strictness =  switch_controller_msg.request.STRICT;

	switch_controller.call(switch_controller_msg);

	return switch_controller_msg.response.ok;
}

bool stopControllers(ros::NodeHandle &n, std::vector<std::string>& controllers_to_stop){

	ros::service::waitForService("/controller_manager/switch_controller", -1);
	ros::ServiceClient switch_controller = n.serviceClient<controller_manager_msgs::SwitchController>(
			"/controller_manager/switch_controller");

	controller_manager_msgs::SwitchController switch_controller_msg;
	switch_controller_msg.request.stop_controllers = controllers_to_stop;
	switch_controller_msg.request.strictness =  switch_controller_msg.request.STRICT;

	switch_controller.call(switch_controller_msg);

	return switch_controller_msg.response.ok;
}


// res: nothing to return because failure stops 'ctrl' and interface hangs
void startCtrl(const std_msgs::StringConstPtr &msg, ros::NodeHandle &n, ros::Publisher &ctl_pub)
{


	if(standard_ctrls_started && left_shoulder_ctrl_started && left_elbow_ctrl_started && right_shoulder_ctrl_started && right_elbow_ctrl_started)
		return;

	if(!standard_ctrls_started && ((msg->data).compare("START_STANDARD_CTRLS")== 0)){


		std::string cnames;
		ros::param::get("~controllers_to_spawn",cnames);
		ROS_WARN(cnames.c_str());

		controllers_to_start = split(cnames, ' ');

		//reload controllers libraries
		assert(reloadControllerLibraries(n));
		ROS_INFO_STREAM("RELOADED CONTROLLERS LIBRARIES");


		pid = fork();

		//CHILD
		if (pid == 0) {

			char command[248];

			sprintf(command, "controllers_to_spawn:=%s",cnames.c_str());
			ROS_INFO("command: %s",command);

			execl("/opt/ros/kinetic/bin/roslaunch", "/opt/ros/kinetic/bin/roslaunch", "cepheus_robot","spawner.launch", command, (char *)0);
		}
		//PARENT
		else {
			int num_of_ctrls = controllers_to_start.size();

			int count_started_ctrls;
			//sleep for 2 seconds
			sleep(2);

			ros::ServiceClient ctrls_list = n.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");
			controller_manager_msgs::ListControllers msg;

			std::vector<controller_manager_msgs::ControllerState> cs;
			std::vector<controller_manager_msgs::ControllerState>::iterator it;

			do{
				if (ctrls_list.call(msg)){
					cs = msg.response.controller;
				}
				else{
					ROS_ERROR("Failed to call service add_two_ints");
				}

				count_started_ctrls = 0;
				for (it = cs.begin() ; it != cs.end(); ++it){

					ROS_INFO("%s",(it->name).c_str());
					if((it->state).compare("running") == 0){
						count_started_ctrls++;
					}
				}

				sleep(1);

			} while(count_started_ctrls < num_of_ctrls);

			//send OK response to interface
			ros::Rate loop_rate(200);
			int count = 0;
			std_msgs::String res_msg;
			res_msg.data = "STANDARD_CTRLS_OK";
			while (count < MSG_NUM) {

				ctl_pub.publish(res_msg);

				loop_rate.sleep();
				++count;
			}

			standard_ctrls_started = true;
		}

	}
	else if(!left_shoulder_ctrl_started && ((msg->data).compare(CMD_START_LEFT_SHOULDER)== 0)){

		bool rv;

		rv = loadController(n, std::string(LEFT_SHOULDER_CONTROLLER));
		if(rv)
			ROS_WARN("LOADED");
		else
			ROS_WARN("could not load");

		std::vector<std::string> v;
		v.push_back(std::string(LEFT_SHOULDER_CONTROLLER));

		rv = startControllers(n, v);
		if(rv)
			ROS_WARN("STARTED");
		else
			ROS_WARN("could not start");

		//send OK response to interface
		ros::Rate loop_rate(200);
		int count = 0;
		std_msgs::String res_msg;
		res_msg.data = std::string(RESPONSE_LEFT_SHOULDER);
		while (count < MSG_NUM) {

			ctl_pub.publish(res_msg);

			loop_rate.sleep();
			++count;
		}

		left_shoulder_ctrl_started = true;
	}
	else if(!left_elbow_ctrl_started && ((msg->data).compare(CMD_START_LEFT_ELBOW)== 0)){

		bool rv;

		rv = loadController(n, std::string(LEFT_ELBOW_CONTROLLER));
		if(rv)
			ROS_WARN("LOADED");
		else
			ROS_WARN("could not load");

		std::vector<std::string> v;
		v.push_back(std::string(LEFT_ELBOW_CONTROLLER));
		rv = startControllers(n, v);

		if(rv)
			ROS_WARN("STARTED");
		else
			ROS_WARN("could not start");


		//send OK response to interface
		ros::Rate loop_rate(200);
		int count = 0;
		std_msgs::String res_msg;
		res_msg.data = std::string(RESPONSE_LEFT_ELBOW);

		while (count < MSG_NUM) {

			ctl_pub.publish(res_msg);
			loop_rate.sleep();
			++count;
		}

		left_elbow_ctrl_started = true;

	}
	else if(!right_shoulder_ctrl_started && ((msg->data).compare(CMD_START_RIGHT_SHOULDER)== 0)){

		bool rv;

		rv = loadController(n, std::string(RIGHT_SHOULDER_CONTROLLER));
		if(rv)
			ROS_WARN("LOADED");
		else
			ROS_WARN("could not load");

		std::vector<std::string> v;
		v.push_back(std::string(RIGHT_SHOULDER_CONTROLLER));

		rv = startControllers(n, v);
		if(rv)
			ROS_WARN("STARTED");
		else
			ROS_WARN("could not start");

		//send OK response to interface
		ros::Rate loop_rate(200);
		int count = 0;
		std_msgs::String res_msg;
		res_msg.data = std::string(RESPONSE_RIGHT_SHOULDER);
		while (count < MSG_NUM) {

			ctl_pub.publish(res_msg);
			loop_rate.sleep();
			++count;
		}

		right_shoulder_ctrl_started = true;

	}
	else if(!right_elbow_ctrl_started && ((msg->data).compare(CMD_START_RIGHT_ELBOW)== 0)){

		bool rv;

		rv = loadController(n, std::string(RIGHT_ELBOW_CONTROLLER));
		if(rv)
			ROS_WARN("LOADED");
		else
			ROS_WARN("could not load");

		std::vector<std::string> v;
		v.push_back(std::string(RIGHT_ELBOW_CONTROLLER));

		rv = startControllers(n, v);
		if(rv)
			ROS_WARN("STARTED");
		else
			ROS_WARN("could not start");

		//send OK response to interface
		ros::Rate loop_rate(200);
		int count = 0;
		std_msgs::String res_msg;
		res_msg.data = std::string(RESPONSE_RIGHT_ELBOW);
		while (count < MSG_NUM) {

			ctl_pub.publish(res_msg);
			loop_rate.sleep();
			++count;
		}

		right_elbow_ctrl_started = true;
	}
	else if((msg->data).compare(CMD_SWITCH_TO_EFFORT) == 0) {

		bool rv;

		// stop position controllers
		std::vector<std::string> to_stop;
		to_stop.push_back(std::string(LEFT_SHOULDER_CONTROLLER));
		to_stop.push_back(std::string(LEFT_ELBOW_CONTROLLER));
		to_stop.push_back(std::string(RIGHT_ELBOW_CONTROLLER));

		rv = stopControllers(n, to_stop);
		if(rv)
			ROS_WARN("STOPPED");
		else
			ROS_WARN("could not stop");

		// unload position controllers
		rv = unloadController(n, std::string(LEFT_SHOULDER_CONTROLLER));
		if(rv)
			ROS_WARN("LOADED");
		else
			ROS_WARN("could not load");

		rv = unloadController(n, std::string(LEFT_ELBOW_CONTROLLER));
		if(rv)
			ROS_WARN("LOADED");
		else
			ROS_WARN("could not load");

		rv = unloadController(n, std::string(RIGHT_ELBOW_CONTROLLER));
		if(rv)
			ROS_WARN("LOADED");
		else
			ROS_WARN("could not load");

		// load effort controllers
		rv = loadController(n, std::string(LEFT_SHOULDER_EFFORT_CONTROLLER));
		if(rv)
			ROS_WARN("LOADED");
		else
			ROS_WARN("could not load");

		rv = loadController(n, std::string(LEFT_ELBOW_EFFORT_CONTROLLER));
		if(rv)
			ROS_WARN("LOADED");
		else
			ROS_WARN("could not load");

		rv = loadController(n, std::string(RIGHT_ELBOW_EFFORT_CONTROLLER));
		if(rv)
			ROS_WARN("LOADED");
		else
			ROS_WARN("could not load");

		// start effort controllers
		std::vector<std::string> to_start;
		to_start.push_back(std::string(LEFT_SHOULDER_EFFORT_CONTROLLER));
		to_start.push_back(std::string(LEFT_ELBOW_EFFORT_CONTROLLER));
		to_start.push_back(std::string(RIGHT_ELBOW_EFFORT_CONTROLLER));

		rv = startControllers(n, to_start);
		if(rv)
			ROS_WARN("STARTED");
		else
			ROS_WARN("could not start");

		// //send OK response to interface
		// ros::Rate loop_rate(200);
		// int count = 0;
		// std_msgs::String res_msg;
		// res_msg.data = std::string(RESPONSE_LEFT_SHOULDER);
		// while (count < MSG_NUM) {

		// 	ctl_pub.publish(res_msg);

		// 	loop_rate.sleep();
		// 	++count;
		// }

		// left_shoulder_ctrl_started = true;
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cepheus_ctrl");
	ros::NodeHandle n;

	//have to declare message type cause I pass const pointer to the callback
	ros::Publisher ctl_pub = n.advertise<std_msgs::String>("load_start_controllers_response",10);
	ros::Subscriber ctl_sub = n.subscribe<std_msgs::String> ("load_start_controllers", 10, boost::bind(&startCtrl, _1, boost::ref(n), boost::ref(ctl_pub)));

	/*
	 *  ros::spin() will not return until the node has been shutdown, 
	 *  either through a call to ros::shutdown() or a Ctrl-C. 
	 */
	ros::spin();

	return 0;
}
