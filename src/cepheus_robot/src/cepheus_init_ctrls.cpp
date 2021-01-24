#include <controller_manager/controller_manager.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include "cepheus_hardware.h"
#include "cepheus_ctrl.h"


const int MSG_NUM = 100;

extern bool standard_controllers_started;
extern bool standard_controllers_started;
extern bool left_shoulder_ctrl_started;
extern bool left_elbow_ctrl_started;
extern bool right_shoulder_ctrl_started;
extern bool right_elbow_ctrl_started;

void ctlNodeReport(const std_msgs::StringConstPtr &msg){

	if(standard_controllers_started &&
		left_shoulder_ctrl_started &&
		left_elbow_ctrl_started &&
		right_elbow_ctrl_started &&
		right_shoulder_ctrl_started)
		return;

	if(!standard_controllers_started && (msg->data).compare("STANDARD_CTRLS_OK") == 0){
		standard_controllers_started = true;
		//ROS_WARN("STARTEEEEDD");
	}

	if(standard_controllers_started && (msg->data).compare(std::string(RESPONSE_LEFT_ELBOW))==0){
		left_elbow_ctrl_started = true;
	}

	if(standard_controllers_started && left_elbow_ctrl_started && (msg->data).compare(std::string(RESPONSE_LEFT_SHOULDER))==0){
		left_shoulder_ctrl_started = true;
	}

	if(standard_controllers_started && (msg->data).compare(std::string(RESPONSE_RIGHT_ELBOW))==0){
		right_elbow_ctrl_started = true;
	}

	if(standard_controllers_started && right_elbow_ctrl_started && (msg->data).compare(std::string(RESPONSE_RIGHT_SHOULDER))==0){
		right_shoulder_ctrl_started = true;
	}

}


void start_standard_controllers(ros::NodeHandle& nh, controller_manager::ControllerManager& cm, ros::Rate& loop_rate){

	ros::Publisher ctl_pub = nh.advertise<std_msgs::String>("load_start_controllers", 10);
	ros::Subscriber ctl_sub = nh.subscribe<std_msgs::String>("load_start_controllers_response", 10, &ctlNodeReport);


	std_msgs::Float64 set_point_msg;

	//IN ORDER TO PUBLISH THE MESSAGE MORE THAN ONE TIME
	int count = 0 ;
	std_msgs::String msg;
	msg.data = "START_STANDARD_CTRLS";
	while (count < MSG_NUM) {

		ctl_pub.publish(msg);

		loop_rate.sleep();
		++count;
	}


	ROS_WARN("ORDER TO START STANDARD CTLS!");

	ros::AsyncSpinner init_spinner(2);
	init_spinner.start();


	ros::Time update_time = ros::Time::now();
	ros::Time prev_time = update_time;

	while(!standard_controllers_started)
	{
		ros::Duration time_step = update_time - prev_time;
		prev_time = update_time;

		cm.update(update_time, time_step);

		loop_rate.sleep();
	}
	init_spinner.stop();

	ROS_WARN("STANDARD CONTROLLERS HAVE STARTED!");

}

// 2020 keep it simple -- start //

void init_left_elbow_and_start_controller(ros::NodeHandle& nh,
										controller_manager::ControllerManager& cm,
										CepheusHW& robot,
										ros::Publisher left_elbow_pub,
										ros::Rate& loop_rate)
{

	ros::Publisher ctl_pub = nh.advertise<std_msgs::String>("load_start_controllers", 10);
	ros::Subscriber ctl_sub = nh.subscribe<std_msgs::String>("load_start_controllers_response", 10, &ctlNodeReport);
	//ros::Publisher go_to_zero_service = nh.advertise<std_msgs::Bool>("left_elbow_go_to",1);

	std_msgs::String msg;
	//std_msgs::Bool empty_msg;

	robot.init_left_elbow();

	msg.data = std::string(CMD_START_LEFT_ELBOW);
	//ctl_pub.publish(msg);
	//go_to_zero_service.publish(empty_msg);
}


void init_left_shoulder_and_start_controller(ros::NodeHandle& nh,
										controller_manager::ControllerManager& cm,
										CepheusHW& robot,
										ros::Publisher left_shoulder_pub,
										ros::Rate& loop_rate)
{

	ros::Publisher ctl_pub = nh.advertise<std_msgs::String>("load_start_controllers", 10);
	ros::Subscriber ctl_sub = nh.subscribe<std_msgs::String>("load_start_controllers_response", 10, &ctlNodeReport);

	std_msgs::String msg;

	robot.init_left_shoulder();

	msg.data = std::string(CMD_START_LEFT_SHOULDER);
	//ctl_pub.publish(msg);
}


void init_right_elbow_and_start_controller(ros::NodeHandle& nh,
										controller_manager::ControllerManager& cm,
										CepheusHW& robot,
										ros::Publisher right_elbow_pub,
										ros::Rate& loop_rate)
{

	ros::Publisher ctl_pub = nh.advertise<std_msgs::String>("load_start_controllers", 10);
	ros::Subscriber ctl_sub = nh.subscribe<std_msgs::String>("load_start_controllers_response", 10, &ctlNodeReport);

	std_msgs::String msg;

	robot.init_right_elbow();

	msg.data = std::string(CMD_START_RIGHT_ELBOW);
	//ctl_pub.publish(msg);
}



// 2020 keep it simple -- end //

void init_left_arm_and_start_controllers(ros::NodeHandle& nh,
										controller_manager::ControllerManager& cm,
										CepheusHW& robot,
										ros::Publisher left_shoulder_pub,
										ros::Publisher left_elbow_pub,
										ros::Rate& loop_rate)
{

	ros::Publisher ctl_pub = nh.advertise<std_msgs::String>("load_start_controllers", 10);
	ros::Subscriber ctl_sub = nh.subscribe<std_msgs::String>("load_start_controllers_response", 10, &ctlNodeReport);

//	std_msgs::Float64 set_point_msg;

	//IN ORDER TO PUBLISH THE MESSAGE MORE THAN ONE TIME
//	int count = 0 ;
	std_msgs::String msg;

//	ros::AsyncSpinner init_spinner(2);
//	init_spinner.start();


//	ros::Time update_time = ros::Time::now();
//	ros::Time prev_time = update_time;


	//INITIALIZE THE LEFT ELBOW
	robot.init_left_elbow();

	msg.data = std::string(CMD_START_LEFT_ELBOW);
	ctl_pub.publish(msg);

/*	count = 0;
	while (count < MSG_NUM) {

		ctl_pub.publish(msg);

		loop_rate.sleep();
		++count;
	}

*/
/*
	init_spinner.start();
	while(!left_elbow_ctrl_started)
	{
		//ros::Time curr_time = ros::Time::now();
		ros::Duration time_step = update_time - prev_time;
		prev_time = update_time;

		robot.readEncoders(time_step);
		cm.update(update_time, time_step);

		loop_rate.sleep();
	}
	init_spinner.stop();
*/

	//INITIALIZE THE LEFT SHOULDER
//	robot.init_left_shoulder();
/*	msg.data = std::string(CMD_START_LEFT_SHOULDER);

	count = 0;
	while (count < MSG_NUM) {

		ctl_pub.publish(msg);

		loop_rate.sleep();
		++count;
	}
*/
/*
	init_spinner.start();

	while(!left_shoulder_ctrl_started)
	{
		//ros::Time curr_time = ros::Time::now();

		set_point_msg.data = robot.getPos(LEFT_ELBOW);

		left_elbow_pub.publish(set_point_msg);

		ros::Duration time_step = update_time - prev_time;
		prev_time = update_time;

		robot.readEncoders(time_step);
		cm.update(update_time, time_step);

		loop_rate.sleep();
	}

	init_spinner.stop();
*/
	//Initialize the left finger and the wrist
	//robot.init_left_finger();
	//robot.init_left_wrist();

}


void init_right_arm_and_start_controllers(ros::NodeHandle& nh,
										controller_manager::ControllerManager& cm,
										CepheusHW& robot,
										ros::Publisher right_shoulder_pub,
										ros::Publisher right_elbow_pub,
										ros::Rate& loop_rate)
{

/*	ros::Publisher ctl_pub = nh.advertise<std_msgs::String>("load_start_controllers", 10);
	ros::Subscriber ctl_sub = nh.subscribe<std_msgs::String>("load_start_controllers_response", 10, &ctlNodeReport);

	int count = 0 ;
	std_msgs::Float64 set_point_msg;
	std_msgs::String msg;

	ros::AsyncSpinner init_spinner(2);


	ros::Time update_time = ros::Time::now();
	ros::Time prev_time = update_time;
*/
	//INITIALIZE THE RIGHT ELBOW
	robot.init_right_elbow();
	/*msg.data = std::string(CMD_START_RIGHT_ELBOW);
	ctl_pub.publish(msg);

	count = 0;
	while (count < MSG_NUM) {

		ctl_pub.publish(msg);

		loop_rate.sleep();
		++count;
	}

	init_spinner.start();

	while(!right_elbow_ctrl_started)
	{
		//ros::Time curr_time = ros::Time::now();
		ros::Duration time_step = update_time - prev_time;
		prev_time = update_time;

		robot.readEncoders(time_step);
		cm.update(update_time, time_step);

		loop_rate.sleep();
	}






	init_spinner.stop();
*/
/*
	//INITIALIZE THE RIGHT SHOULDER
	//robot.init_right_shoulder();
	msg.data = std::string(CMD_START_RIGHT_SHOULDER);
	ctl_pub.publish(msg);

	count = 0;
	while (count < MSG_NUM) {

		ctl_pub.publish(msg);

		loop_rate.sleep();
		++count;
	}

	init_spinner.start();

	while(!right_shoulder_ctrl_started)
	{
		//ros::Time curr_time = ros::Time::now();

		set_point_msg.data = robot.getPos(RIGHT_ELBOW);

		right_elbow_pub.publish(set_point_msg);

		ros::Duration time_step = update_time - prev_time;
		prev_time = update_time;

		robot.readEncoders(time_step);
		cm.update(update_time, time_step);

		loop_rate.sleep();
	}
*/
/*	init_spinner.stop();

	robot.init_right_finger();
	robot.init_right_wrist();
*/
}
