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
#include <sys/types.h>
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
#include <controller_manager_msgs/ControllerState.h>

#include "cepheus_hardware.h"

#define MSG_NUM 100

bool started_ctrls = false;
int num_of_ctrls = 0;
pid_t pid;

// res: nothing to return because failure stops 'ctrl' and interface hangs
void startCtrl(const std_msgs::StringConstPtr &msg, ros::NodeHandle &n, ros::Publisher &ctl_pub)
{
	if (started_ctrls) return;
	else started_ctrls = true;

	if(((msg->data).compare("START_CTL") == 0)){
		//reload controllers libraries
		ros::service::waitForService("/controller_manager/reload_controller_libraries", -1);
		ros::ServiceClient reload_controller = n.serviceClient<controller_manager_msgs::ReloadControllerLibraries>(
				"/controller_manager/reload_controller_libraries");

		controller_manager_msgs::ReloadControllerLibraries reload_controllers_libraries_msg;
		reload_controllers_libraries_msg.request.force_kill = true;
		reload_controller.call(reload_controllers_libraries_msg);
		assert(reload_controllers_libraries_msg.response.ok);

		ROS_INFO("SUCCEDED_TO_RELOAD_CONTROLLERS_LIBRARIES");



		std::string cnames;
		ros::param::get("~controllers_to_spawn",cnames);
		ROS_WARN(cnames.c_str());

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
			// loop through list of controllers
			// check if controllers_to_spawn started
			// if all send message


			//Calculate how many controllers are expected to start
			char ctrl_names[248];

			sprintf(ctrl_names, "%s",cnames.c_str());

			char *space = NULL;

			space = strchr(ctrl_names, ' ');

			do{
				num_of_ctrls++;
				if(space != NULL){

					space ++;
					space = strchr(space, ' ');
				}

			} while(space != NULL);
			num_of_ctrls++;

			//Check if the number of started controllers is the same with the number you expected
			//if yes then send "OK" to "cepheus interface"
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

					//ROS_INFO("%s",(it->state).c_str());
					if((it->state).compare("running") == 0){
						count_started_ctrls++;
					}
				}

				sleep(1);

			} while(count_started_ctrls < num_of_ctrls);

			//send OK response to interface
			ros::Rate loop_rate(500);
			int count = 0;
			std_msgs::String res_msg;
			res_msg.data = "OK";
			while (count < MSG_NUM) {

				ctl_pub.publish(res_msg);

				loop_rate.sleep();
				++count;
			}

		}
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

	/*
	   int count_stopped_ctrls;
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
	   ROS_ERROR("Failed to call service");
	   }	

	   sleep(1);
	   } while(!cs.empty());
	 */

	int status = 0;
	while (wait(&status) > 0) ;
	ROS_WARN("Just picked up my dead offspring");


	/*
	   ros::Rate loop_rate(500);
	   int count = 0;
	   std_msgs::String res_msg;
	   res_msg.data = "END";
	   while (count < MSG_NUM) {
	   ctl_pub.publish(res_msg);

	   loop_rate.sleep();
	   ++count;
	   }
	 */

	return 0;
}
