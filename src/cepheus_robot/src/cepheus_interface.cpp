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
#include <std_msgs/String.h>

#include "cepheus_hardware.h"

#define MSG_NUM 100

#define SH_DUR 2
#define ELB_DUR 2

CepheusHW robot;

double rw_torque = 0.0;
double rw_cur_vel = 0.0;
double rw_last_vel = 0.0;

bool controllers_started = false;

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
				robot.disable();
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
				robot.disable();
			}
		} else if (c == 'q') {
			break;
		} else {
			ROS_WARN("Not valid input!");
		}
	}
}


void ctlNodeReport(const std_msgs::StringConstPtr &msg){

	if(!controllers_started && (msg->data).compare("OK") == 0){
		controllers_started = true;
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
	//--------Panagiotis Mavridis 25/04/2018----------------
	//----Create file with Thrust message latencies in order to plot---


	//latency_fp = fopen("/home/mrrobot/Desktop/cmd_thrust_laytencies.txt","w");

	//------------------------------------------------------ 



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
	// ros::Publisher  torque_pub =  nh.advertise<std_msgs::Float64>("reaction_wheel_velocity_controller/command", 1);
	ros::Publisher  torque_pub =  nh.advertise<std_msgs::Float64>("reaction_wheel_effort_controller/command", 1);

	//PANOS_KWSTAS
	ros::Publisher  left_shoulder_pub =  nh.advertise<std_msgs::Float64>("left_shoulder_position_controller/command", 1);
	ros::Publisher  left_elbow_pub =  nh.advertise<std_msgs::Float64>("left_elbow_position_controller/command", 1);


	controller_manager::ControllerManager cm(&robot);
	ros::Time prev_time = ros::Time::now();

	robot.setHomePos(4, l1_limit_pos); 
	robot.setHomePos(5, l2_limit_pos);

	/*
	   int err = readErr();
	   if (err) {
	   fixJointPos();
	   ROS_INFO("Fixed, now init...\n");
	   }
	 */


	//INITIALIZE THE
//	robot.init();
	//robot.init_2();


	//--------PANOS NKWSTAS effort to load/start controllers with topic----------------------

	ros::Publisher ctl_pub = nh.advertise<std_msgs::String>("load_start_controllers",10);
	ros::Subscriber ctl_sub = nh.subscribe<std_msgs::String>("load_start_controllers_response",10,&ctlNodeReport);

	//IN ORDER TO PUBLISH THE MESSAGE MORE THAN ONE TIME

	int count = 0;
	std_msgs::String msg;
	msg.data = "START_CTL";
	while (count < MSG_NUM) {

		ctl_pub.publish(msg);

		loop_rate.sleep();
		++count;
	}

	//ROS_WARN("ORDER TO START CTLS!");

	//in order to process the message from the topic coming from cepheus_ctl node
	// while(1){
	// 	ros::spinOnce();
	// }	

	//----------------------------------------------------------------------------------------

	//PANOS_KWSTAS service to load/start controllers
	/*ros::ServiceClient client = nh.serviceClient<std_srvs::Empty> ("start_ctrl");
	  std_srvs::Empty::Request req;
	  std_srvs::Empty::Response res;

	  ROS_INFO("Waiting for controllers to load,start");
	  if (client.call(req,res))
	  {
	  ROS_INFO("Service ends: Controllers started!");
	  }
	  else
	  {
	  ROS_ERROR("Failed to call service ctrl");
	  return 1;
	  }*/

	//PANOS_KWSTAS
	/*	std_msgs::Float64 cmd;
		cmd.data = 0;
		left_shoulder_pub.publish(cmd);
		cmd.data = 0;
		left_elbow_pub.publish(cmd);
	 */
	ros::AsyncSpinner init_spinner(2);
	init_spinner.start();

	while(!controllers_started)
	{
		ros::Time curr_time = ros::Time::now();
		ros::Duration time_step = curr_time - prev_time;
		prev_time = curr_time;

		ros::spinOnce();

		robot.readEncoders(time_step);
		cm.update(curr_time, time_step);

		robot.setJointTorque(0,0);
		robot.writeMotors();
		robot.heartbeat();

		loop_rate.sleep();
	}
	init_spinner.stop();

	ROS_WARN("About to enter normal spinning...");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	while(!g_request_shutdown)
	{
		ros::Time curr_time = ros::Time::now();
		ros::Duration time_step = curr_time - prev_time;
		prev_time = curr_time;

		ros::spinOnce();

		robot.readEncoders(time_step);
		cm.update(curr_time, time_step);


		//robot.testGripperFinger();
		robot.writeMotors();

		robot.heartbeat();

		// //saturate acceleration og reaction wheel
		// double rw_available_torque;
		// double rw_real_vel = robot.getVel(0);
		// if(fabs(rw_real_vel)<=0.01) {
		//     rw_available_torque = rw_max_torque;
		// }
		// else {
		//     rw_available_torque = rw_max_power/fabs(rw_real_vel);
		// }

		// // rw_available_torque = rw_max_torque;
		// if (rw_torque > rw_available_torque) {
		//     rw_torque = rw_available_torque;
		// }
		// else if (rw_torque < -rw_available_torque) {
		//     rw_torque = -rw_available_torque;
		// }
		// //calculate velocity cmd for reaction wheel
		// double rw_cmd_vel;
		// rw_cmd_vel = (rw_torque/rw_total_inertia) * time_step.toSec() + rw_cur_vel;
		// if(fabs(rw_cmd_vel) > rw_max_speed)
		// {
		//     rw_cmd_vel = rw_cur_vel;
		// }
		// rw_cur_vel = rw_cmd_vel;

		// // need rifinement
		// std_msgs::Float64 cmd_rw_vel;
		// cmd_rw_vel.data = -1*rw_cmd_vel;
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
		// ROS_INFO("cmd: %f", cmd_rw_vel.data);

		loop_rate.sleep();
	}

	robot.safeClose();

	//fclose(latency_fp);
	return 0;
}
