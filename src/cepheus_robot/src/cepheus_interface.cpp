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

//for clock_gettime and monotonic clocks
#include <time.h>
#include <sys/time.h>
#include <sys/resource.h>
#define NANO_TO_MICRO_DIVISOR 1000
FILE *latency_fp;
#include <boost/bind.hpp>
#define RT_PRIORITY 95


#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <controller_manager/controller_manager.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>


#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>

#include "cepheus_hardware.h"

#define MSG_NUM 100

#define SH_DUR 2
#define ELB_DUR 2

//Panagiotis Mavridis
//---Constants for fsr force controller
#define F_DES 3
#define KP 0.01
#define KI 1
#define F_HIGH 4
#define F_LOW 2


//------------------------------------


CepheusHW robot;
controller_manager::ControllerManager cm(&robot);


double rw_torque = 0.0;
double rw_cur_vel = 0.0;
double rw_last_vel = 0.0;

bool standard_controllers_started = false;
bool left_shoulder_ctrl_started = false;
bool left_elbow_ctrl_started = false;

bool ready_to_grip_left = false;
bool left_gripper_first_time = true;

//In order to be used from callbacks
ros::Publisher  left_shoulder_pub, left_elbow_pub;

int readErr()
{
	FILE *f;
	int i;
	f = fopen("/home/mrrobot/nerr.txt","r");
	if (f == NULL) return 0;

	fscanf(f,"%d",&i);
	ROS_INFO("Errors number = %d",i);

	fclose(f);
	return i;
}

void fixJointPos()
{
	int sh=0;
	int elb=0;
	char c;

	for (;;) {
		ROS_WARN("Give arm: s(houlder) or e(lbow) or q(uit)");
		fflush(stdout);

		c = getchar();
		getchar();

		if (c == 's') 
		{
			for(;;) {
				ROS_WARN("Give direction (-/+) or q(uit)");
				fflush(stdout);

				c = getchar();
				while (getchar() != '\n') ;

				if (c == 'q') break;
				else if (c == '-') sh = -1;
				else if (c == '+') sh = 1;
				else continue;

				robot.setJointTorque(sh,0);

				robot.writeMotors();
				ros::Time init_time = ros::Time::now();
				ros::Duration timer;
				while(timer.toSec()<SH_DUR) {
					robot.heartbeat();				
					timer = ros::Time::now() - init_time;
				}

				robot.setJointTorque(0,0);
				robot.writeMotors();
				//robot.disable();
			}
		} else if (c == 'e') {
			for(;;) {
				ROS_WARN("Give direction (-/+) or q(uit)");
				fflush(stdout);

				c = getchar();
				while (getchar() != '\n') ;

				if (c == 'q') break;
				else if (c == '-') elb = -1;
				else if (c == '+') elb = 1;
				else continue;

				robot.setJointTorque(0,elb);
				robot.writeMotors();

				ros::Time init_time = ros::Time::now();
				ros::Duration timer;
				while(timer.toSec()<ELB_DUR) {
					robot.heartbeat();
					timer = ros::Time::now() - init_time;

				}

				robot.setJointTorque(0,0);
				robot.writeMotors();
				//robot.disable();
			}
		} else if (c == 'q') {
			break;
		} else {
			ROS_WARN("Not valid input!");
		}
	}
}


void ctlNodeReport(const std_msgs::StringConstPtr &msg){

	if(standard_controllers_started && left_shoulder_ctrl_started && left_elbow_ctrl_started)
		return;

	if(!standard_controllers_started && (msg->data).compare("STANDARD_CTRLS_OK") == 0){
		standard_controllers_started = true;
		//ROS_WARN("STARTEEEEDD");
	}

	if(standard_controllers_started && (msg->data).compare("LEFT_ELBOW_CTRL_OK")==0){
		left_elbow_ctrl_started = true;
	}

	if(standard_controllers_started && left_elbow_ctrl_started && (msg->data).compare("LEFT_SHOULDER_CTRL_OK")==0){
		left_shoulder_ctrl_started = true;
	}

}

//--------------------ORIGINAL---------------------------------
/*
   void thrusterCallback(const geometry_msgs::Vector3::ConstPtr& cmd)
   {
   double thrust[4];
   thrust[0] = (double)cmd->x;
   thrust[1] = (double)cmd->y;
   thrust[2] = (double)cmd->z;
   thrust[3] = (double)0;
   robot.setThrustPwm(thrust, 0.001, 0.9);
   return;
   }
 */
//-----------------------------------------------------------------


//----------TO MEASURE LATENCY-REPLACE WITH ORIGINAL IF  NEEDED AND IN PLANNER AND CONTROLLER-----
//----------Panagiotis Mavridis 24/04/2018---------------------

void thrusterCallback(const geometry_msgs::Vector3Stamped::ConstPtr& cmd)
{
	/*static int m_counter=0;
	//CALCULATING MONOTONIC CLOCK TIME DIFFERENCE 
	struct timespec ts_arrived;
	clock_gettime(CLOCK_MONOTONIC_RAW, &ts_arrived);

	long latency = ((ts_arrived.tv_sec - cmd->header.stamp.sec) * 1000000000 + (ts_arrived.tv_nsec - cmd->header.stamp.nsec))/NANO_TO_MICRO_DIVISOR;
	//std::cout << " LATENCY IN CONTROLLER->INTERFACE (THRUSTERS) : "<< latency;
	m_counter++;
	fprintf(latency_fp,"%ld\n",latency);
	 */
	double thrust[4];
	thrust[0] = (double)cmd->vector.x;
	thrust[1] = (double)cmd->vector.y;
	thrust[2] = (double)cmd->vector.z;
	thrust[3] = (double)0;
	robot.setThrustPwm(thrust, 0.001, 0.9);
	return;
}

//----------------------------------------------------------------------------------------------

void torqueCallback(const std_msgs::Float64::ConstPtr& cmd)
{
	rw_torque = (double)cmd->data;
}

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
// Replacement SIGINT handler
void ctrl_C_Handler(int sig)
{
	g_request_shutdown = 1;
}


void leftFsrCallback(const std_msgs::UInt8::ConstPtr& cmd)
{
	//ROS_WARN("fsr val : %d",cmd->data);
	robot.set_left_fsr_value(cmd->data);
}


void leftWristCallback(const std_msgs::Float64::ConstPtr& cmd)
{
	//ROS_WARN("CMD WRIST %lf",cmd->data);
	if(cmd->data >=0 && cmd->data <= LEFT_WRIST_MAX_ANGLE){
		robot.setCmd(8, (double)cmd->data);
	}
	else
		ROS_WARN("Cmd to left wrist out of bounds!");
}

void leftGripperCallback(const std_msgs::Float64::ConstPtr& cmd)
{
	//ROS_WARN("CMD GRIP %lf",cmd->data);
	if(cmd->data >=0 && cmd->data <= LEFT_FINGER_MAX_ANGLE)
		robot.setCmd(10, (double)cmd->data);
	else
		ROS_WARN("Cmd to left gripper out of bounds!");

}

void leftGripperActionCallback(const std_msgs::Bool::ConstPtr& cmd)
{
	if(cmd->data){
		ROS_WARN("ORDERED TO CLOSE LEFT_GRIPPER");
		ready_to_grip_left = true;
	}
	else{
		ROS_WARN("ORDERED TO OPEN LEFT_GRIPPER");
		ready_to_grip_left = false;
		left_gripper_first_time = true;

		robot.init_left_finger();
	}

}


//Trend line transforming force to d_theta for gripper
double fsr_trend_line(bool positive ,double pi_out){

	double rv = 0;
	rv = 0.25 * (double)pow(pi_out,3) - (double)4 * (double)pow(pi_out,2) + 20.4 * pi_out + 0.04;

	if(positive)
		return rv;
	else
		return -rv;
}

//Pnagiotis Mavridis
//PI controller for left gripper (force controll)
void left_fsr_update(){


	static int8_t error_sum = 0;
	static uint16_t count = 0;
	static double gripper_last_angle = 0.0;
	static double new_angle = 0.0;
	double d_theta = 0.0;
	uint16_t width_val = 0;	
	double div = 0.0;
	double pi_out = 0.0;
	static ros::Time init_time;
	static ros::Duration dur;

	if(left_gripper_first_time){
		gripper_last_angle = (double)LEFT_FINGER_MAX_ANGLE;
		error_sum = 0;
		count = 0;
		init_time = ros::Time::now();
		left_gripper_first_time = false;
	}


	int8_t fsr_val = robot.get_left_fsr_val();
	int8_t error = fsr_val - (int8_t)F_DES ;

	if(error < -1 || error > 1){

		//PI out (Kp * error + Ki * sum(error))
		error_sum += error;
		count ++;
		double avg = (double)error_sum/(double)count; 
		ROS_WARN("AVG = %lf error_sum %d count %d",avg,error_sum, count);	
		//For protection from overheating, if the fsr does not sense the target as expected ,so the error is not decreasing
		dur =  ros::Time::now() - init_time;
		if(dur.toSec() >= 10 && avg <= -2.0 ){
			//open and try again
			robot.init_left_finger();
			sleep(2);
			init_time = ros::Time::now();
			error_sum = 0;
			count = 0;
			left_gripper_first_time = true;
		}


		//uint8_t pi_out = KP * error + KI * error_sum;
		pi_out = (double)KP * (double)error;

		//tranpose force to width in order to give the command
		if(fsr_val == 0){
			d_theta = 1;
		}
		else if(fsr_val > 0 && fsr_val < F_DES){
			d_theta = fsr_trend_line(true, abs(pi_out));
		}	
		else if(fsr_val >= F_DES){
			d_theta = fsr_trend_line(false, abs(pi_out));
		}

		//holding the last position of the gripper
		gripper_last_angle -= d_theta;
		new_angle = gripper_last_angle;

		robot.setCmd(LEFT_GRIPPER, new_angle);
		ROS_WARN("FSR: %d, error: %d, PI_OUT: %lf, d_theta: %lf, new_angle: %lf", fsr_val, error, pi_out, d_theta, new_angle);
	}
}

//-------------------------------------------
//----------------------------------------------------------------

double produce_trajectory_point_wrist(double time, double movement_duration, double init_pos, double set_point){

	double tj_p;

	double a0 = init_pos;
	double a1 = 0.0;
	double a2 = (3.0/(double)pow(movement_duration,2)) * (double)(set_point - init_pos);
	double a3 = - (2.0/(double)pow(movement_duration,3)) * (double)(set_point - init_pos);

	tj_p = a0 + a1 * time + a2 * (double)pow(time,2) + a3 * (double)pow(time,3);

	return tj_p;

}


double produce_trajectory_point(double time, double movement_duration, double init_pos, double set_point){

	if(set_point >= LEFT_WRIST_MIN_ANGLE && set_point <= LEFT_WRIST_MAX_ANGLE){

		double tj_p;

		double a0 = init_pos;
		double a1 = 0.0;
		double a2 = (3.0/(double)pow(movement_duration,2)) * (double)(set_point - init_pos);
		double a3 = - (2.0/(double)pow(movement_duration,3)) * (double)(set_point - init_pos);

		tj_p = a0 + a1 * time + a2 * (double)pow(time,2) + a3 * (double)pow(time,3);

		return tj_p;

	}
	else
		ROS_WARN("Command to left wrist out of range");
}


//----Create trajectory (given a set_point) for the left arm
void moveLeftArmCallback(const std_msgs::Float64MultiArray::ConstPtr& cmd_array){

	ROS_WARN("Received sp_sh :%lf, sp_el: %lf, sp_wr: %lf, dur: %lf", cmd_array->data[0],cmd_array->data[1],cmd_array->data[2],cmd_array->data[3]);

	double set_point_shoulder = cmd_array->data[0];
	double set_point_elbow = cmd_array->data[1];
	double set_point_wrist = cmd_array->data[2];
	double movement_duration = cmd_array->data[3];

	ros::Time init_time = ros::Time::now();
	ros::Duration timer;
	timer = ros::Time::now() - init_time;

	robot.readEncoders(timer);

	double curr_pos_shoulder, curr_pos_elbow, curr_pos_wrist;
	double init_pos_shoulder = robot.getPos(LEFT_SHOULDER);
	double init_pos_elbow = robot.getPos(LEFT_ELBOW);
	double init_pos_wrist = robot.getCmd(LEFT_WRIST);

	ros::Rate loop_rate(200);

	std_msgs::Float64 cmd_pos;
	double wrist_cmd;

	while(timer.toSec() <= movement_duration){

		robot.readEncoders(timer);

		curr_pos_shoulder = robot.getPos(LEFT_SHOULDER);
		curr_pos_elbow = robot.getPos(LEFT_ELBOW);
		curr_pos_wrist = robot.getCmd(LEFT_WRIST);

		cmd_pos.data = produce_trajectory_point(timer.toSec(), movement_duration, init_pos_shoulder, set_point_shoulder);
		//ROS_WARN("pos : %lf",cmd_pos.data);
		left_shoulder_pub.publish(cmd_pos);

		cmd_pos.data = produce_trajectory_point(timer.toSec(), movement_duration, init_pos_elbow, set_point_elbow);
		left_elbow_pub.publish(cmd_pos);

		wrist_cmd = produce_trajectory_point_wrist(timer.toSec(), movement_duration, init_pos_wrist, set_point_wrist);
		robot.setCmd(LEFT_WRIST, wrist_cmd);

		timer = ros::Time::now() - init_time;
		loop_rate.sleep();
	}

}
//---------------------------------------------------------------

void init_left_arm_and_start_controllers(ros::NodeHandle& nh, controller_manager::ControllerManager& cm, ros::Rate& loop_rate){

	ros::Publisher ctl_pub = nh.advertise<std_msgs::String>("load_start_controllers",10);
	ros::Subscriber ctl_sub = nh.subscribe<std_msgs::String>("load_start_controllers_response",10,&ctlNodeReport);

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

	/*
	//INITIALIZE THE LEFT ELBOW
	robot.init_left_elbow();

	msg.data = "START_LEFT_ELBOW_CTRL";
	ctl_pub.publish(msg);

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

	//INITIALIZE THE LEFT SHOULDER
	robot.init_left_shoulder();
	msg.data = "START_LEFT_SHOULDER_CTRL";

	count = 0;
	while (count < MSG_NUM) {

	ctl_pub.publish(msg);

	loop_rate.sleep();
	++count;
	}



	init_spinner.start();

	std_msgs::Float64 set_point_msg;


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
	robot.init_left_finger();
	robot.init_left_wrist();

}

double produce_sin_trajectory(double width, double period, double t){

	double rv = width * sin((2.0 * M_PI) / period * t);
	return rv;
}

double produce_sin_trajectory_wrist(double width, double period, double t){

	double rv = 60 + width * sin((2.0 * M_PI) / period * t);
	return rv;
}


void moveLeftArmSin(controller_manager::ControllerManager& cm){

	double movement_duration = 12;

	ros::Time init_time = ros::Time::now();
	ros::Duration timer;
	timer = ros::Time::now() - init_time;

	robot.readEncoders(timer);

	double curr_pos_shoulder, curr_pos_elbow, curr_pos_wrist;
	double init_pos_shoulder = robot.getPos(LEFT_SHOULDER);
	double init_pos_elbow = robot.getPos(LEFT_ELBOW);
	double init_pos_wrist = robot.getCmd(LEFT_WRIST);

	ros::Rate loop_rate(200);

	std_msgs::Float64 cmd_pos;
	double wrist_cmd;

	while(timer.toSec() <= movement_duration){

		robot.readEncoders(timer);

		curr_pos_shoulder = robot.getPos(LEFT_SHOULDER);
		curr_pos_elbow = robot.getPos(LEFT_ELBOW);
		curr_pos_wrist = robot.getCmd(LEFT_WRIST);

		cmd_pos.data = produce_sin_trajectory(0.78, movement_duration, timer.toSec());
		left_shoulder_pub.publish(cmd_pos);

		cmd_pos.data = produce_sin_trajectory(-1.22, movement_duration, timer.toSec());
		left_elbow_pub.publish(cmd_pos);

		wrist_cmd = produce_sin_trajectory_wrist(-30, movement_duration, timer.toSec());
		robot.setCmd(LEFT_WRIST, wrist_cmd);

		robot.readEncoders(timer);
		cm.update(ros::Time::now(), timer);
		robot.writeMotors();
		robot.heartbeat();

		ros::spinOnce();

		timer = ros::Time::now() - init_time;
		loop_rate.sleep();
	}

}


void move_left_arm(double set_point_shoulder,
		double set_point_elbow,
		double set_point_wrist,
		double movement_duration,
		controller_manager::ControllerManager& cm){



	ros::Time init_time = ros::Time::now();
	ros::Duration timer;
	timer = ros::Time::now() - init_time;

	robot.readEncoders(timer);

	double curr_pos_shoulder, curr_pos_elbow, curr_pos_wrist;
	double init_pos_shoulder = robot.getPos(LEFT_SHOULDER);
	double init_pos_elbow = robot.getPos(LEFT_ELBOW);
	double init_pos_wrist = robot.getCmd(LEFT_WRIST);

	ros::Rate loop_rate(200);

	std_msgs::Float64 cmd_pos;
	double wrist_cmd;

	while(timer.toSec() <= movement_duration){

		robot.readEncoders(timer);

		curr_pos_shoulder = robot.getPos(LEFT_SHOULDER);
		curr_pos_elbow = robot.getPos(LEFT_ELBOW);
		curr_pos_wrist = robot.getCmd(LEFT_WRIST);

		cmd_pos.data = produce_trajectory_point(timer.toSec(), movement_duration, init_pos_shoulder, set_point_shoulder);
		//ROS_WARN("pos : %lf",cmd_pos.data);
		left_shoulder_pub.publish(cmd_pos);

		cmd_pos.data = produce_trajectory_point(timer.toSec(), movement_duration, init_pos_elbow, set_point_elbow);
		left_elbow_pub.publish(cmd_pos);

		wrist_cmd = produce_trajectory_point_wrist(timer.toSec(), movement_duration, init_pos_wrist, set_point_wrist);
		robot.setCmd(LEFT_WRIST, wrist_cmd);

		robot.readEncoders(timer);
		cm.update(ros::Time::now(), timer);
		robot.writeMotors();
		robot.heartbeat();

		ros::spinOnce();


		timer = ros::Time::now() - init_time;
		loop_rate.sleep();
	}

}


// Inverse kinematics...calculate the set points of the joints of the left arm given an angle that you want ot catch a target
void catchObjectCallback(const std_msgs::Float64& cmd_angle_to_catch){

	//theta2 needs to be from -90 deg to 0 deg


	//TO DO...ADD COMMMEEEEENTS

	//The lengths of the joint of the left arm
	double l1 = 0.181004;	//shoulder
	double l2 = 0.16005;	//elbow
	double l3 = 0.0499;	//wrist

	//Used in order to track the frame of the target
	tf::TransformListener target_listener;
	tf::StampedTransform transform;

	try{
		target_listener.lookupTransform("cepheus","gripper_target",
				ros::Time(0), transform);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
	}

	//position of target
	/*
	   double x = transform.getOrigin().x();
	   double y = transform.getOrigin().y();
	 */
	double x = 0.1;
	double y = 0.21;

	//the desired angle of the wrist translated to cepheus coordinates
	double phi = (90.0/180.0) * M_PI;

	double xn = x - l3  * cos(phi);
	double yn = y - l3 * sin(phi);

	//calculating 2 results for each angle

	//For q2
	double q21 = acos(pow(xn, 2) + pow(yn, 2) - pow(l2, 2) - pow(l1, 2) / (2 * l1 * l2));
	double q22 = -q21;

	//For q1

	//q11
	double a = -l1 - l2 * cos(q21); 
	double b = -l2 * sin(q21);
	double c = -xn;
	double d = -l2 * sin(q21);
	double e = l1 + l2 * cos(q21);
	double f = -yn;

	double s11 = ((c/a) - (f/d)) / ((e/d) - (b/a));
	double c11 = (b*s11/a)+ (c/a);

	double q11 = atan2(s11, c11);

	//q12
	a = -l1 - l2 * cos(q22);
	b = -l2 * sin(q22);
	c = -xn;
	d = -l2 * sin(q22);
	e = l1 + l2 * cos(q22);
	f = -yn;

	double s12 = ((c/a) - (f/d))/((e/d) - (b/a));
	double c12 = (b * s12/a) + (c/a);

	double q12 = atan2(s12, c12);

	double q31 = phi - q11 - q21;
	double q32 = phi - q12 - q22;	

	double tuple1[3];
	double tuple2[3];

	tuple1[0] = q11;
	tuple1[1] = q21;
	tuple1[2] = q31;

	tuple2[0] = q12;
        tuple2[1] = q22;
        tuple2[2] = q32;

	//TO DO>>>>qs to degereees

	//checking the results in order to discard odd angles
	if( (-60.0 <= q31 && q31 <= 60.0) && (-90.0 <= q21 && q21 <= 120.0) && (90.0 <= q11 && q11 <= 180.0) ){
		//solution is tuple1
	}
	else if((-60.0 <= q32 && q32 <= 60.0) && (-90.0 <= q22 && q22 <= 120.0) && (90.0 <= q12 && q12 <= 180.0)){
		//solution is tuple2
	}
	else{
		//we see.....
	}


}


int main(int argc, char** argv) 
{

	ros::init(argc, argv, "cepheus_interface_node", ros::init_options::NoSigintHandler);
	signal(SIGINT, ctrl_C_Handler);
	ros::NodeHandle nh;


	setpriority(PRIO_PROCESS, 0, 19);

	double rate;
	ros::param::param<double>("~loop_rate", rate, 500); 
	ros::Rate loop_rate(rate);

	double l1_limit_pos, l2_limit_pos;
	double max_thrust;
	double rw_max_torque, rw_max_speed, rw_max_power, rw_total_inertia;
	ros::param::param<double>("~thruster_force", max_thrust, 1.5); //the thrust of an open thruster in Newtons
	//ros::param::param<double>("~max_motor_current", max_cur, 1.72); //the max current of the motor
	ros::param::param<double>("~rw_max_torque", rw_max_torque, 0.5); 
	ros::param::param<double>("~rw_max_speed",  rw_max_speed, 100); 
	ros::param::param<double>("~rw_max_power",  rw_max_power, 60); 
	ros::param::param<double>("~rw_total_inertia", rw_total_inertia, 0.00197265); 
	ros::param::param<double>("~left_shoulder_limit_pos", l1_limit_pos, 2.4); 
	ros::param::param<double>("~left_elbow_limit_pos", l2_limit_pos, 1.4); 

	//--------Panagiotis Mavridis 25/04/2018----------------

	struct sched_param schedParam;
	schedParam.sched_priority = RT_PRIORITY;
	int sched_policy = SCHED_RR;
	sched_setscheduler(0, sched_policy, &schedParam);



	double max_cur[8];
	max_cur[0] = 1.72;
	max_cur[1] = 3.1;
	max_cur[2] = 3.1;
	max_cur[3] = 3.1;
	max_cur[4] = 3.1;
	max_cur[5] = 3.1;
	max_cur[6] = 3.1;
	max_cur[7] = 3.1;

	robot.setParam(max_cur, max_thrust);



	ros::Subscriber thrust_sub =  nh.subscribe("cmd_thrust", 1, thrusterCallback);
	ros::Subscriber torque_sub =  nh.subscribe("cmd_torque", 1, torqueCallback);

	//For reading the fsr from the gripper
	ros::Subscriber fsr_sub =  nh.subscribe("left_fsr", 1, leftFsrCallback);

	//For giving cmdsto the left wrist and gripper if nesessary
	//ros::Subscriber left_wrist_sub =  nh.subscribe("left_wrist_cmd", 1, leftWristCallback);
	//ros::Subscriber left_gripper_sub =  nh.subscribe("left_gripper_cmd", 1, leftGripperCallback);

	ros::Subscriber move_left_arm_sub =  nh.subscribe("move_left_arm", 1, moveLeftArmCallback);
	ros::Subscriber left_gripper_action_sub =  nh.subscribe("left_gripper_action", 1, leftGripperActionCallback);
	ros::Subscriber catch_object_sub =  nh.subscribe("catch_object", 1, catchObjectCallback);	

	//ros::Publisher  torque_pub =  nh.advertise<std_msgs::Float64>("reaction_wheel_velocity_controller/command", 1);
	ros::Publisher  torque_pub =  nh.advertise<std_msgs::Float64>("reaction_wheel_effort_controller/command", 1);

	//Puplishers to ROS-contol topics
	left_shoulder_pub =  nh.advertise<std_msgs::Float64>("left_shoulder_position_controller/command", 1000);
	left_elbow_pub =  nh.advertise<std_msgs::Float64>("left_elbow_position_controller/command", 1000);


	ros::Time prev_time = ros::Time::now();

	robot.setHomePos(4, l1_limit_pos); 
	robot.setHomePos(5, l2_limit_pos);


	//int err = readErr();
	//if (err) {
	//fixJointPos();
	//ROS_INFO("Fixed, now init...\n");
	//}

	//Initialize the left arm and start the ros controllers
	init_left_arm_and_start_controllers(nh, cm, loop_rate);

	//move_left_arm(0.0, 0.0, 60.0, 12.0, cm);


	/*ROS_WARN("STARTING SIN...");
	  moveLeftArmSin(cm);	
	  sleep(3);
	  moveLeftArmSin(cm);
	 */
	ROS_WARN("About to enter normal spinning...");
	ros::AsyncSpinner spinner(3);
	spinner.start();

	ros::Time curr_time;
	ros::Duration time_step;	

	bool first_time = true;

	while(!g_request_shutdown)
	{

		curr_time = ros::Time::now();

		if(first_time){
			prev_time = curr_time;
			first_time = false;
		}

		time_step = curr_time - prev_time;
		prev_time = curr_time;


		robot.readEncoders(time_step);
		cm.update(curr_time, time_step);

		if(ready_to_grip_left)
			left_fsr_update();


		robot.writeMotors();
		robot.heartbeat();

		//ros::spinOnce();

		if(rw_torque!=0.0) {
			std_msgs::Float64 cmd;
			cmd.data = rw_torque;
			torque_pub.publish(cmd);
			rw_last_vel = robot.getVel(0);
			rw_cur_vel = rw_last_vel;
		}
		else {
			rw_cur_vel = robot.getVel(0);
			double error = rw_last_vel - rw_cur_vel;
			std_msgs::Float64 cmd;
			cmd.data = -10*error;
			torque_pub.publish(cmd);
		}

		loop_rate.sleep();
	}



	robot.safeClose();

	//fclose(latency_fp);
	return 0;
}
