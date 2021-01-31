#include <signal.h>
#include <stdio.h>
#include <math.h>
#include <std_msgs/Float64.h>

#include <ros/ros.h>

// #include <sys/resource.h>

#include "cepheus_hardware.h"
#include "cepheus_interface.h"


extern CepheusHW robot;


double ls_position;
double le_position;
double re_position;

double ls_velocity;
double le_velocity;
double re_velocity;

double ls_error;
double le_error;
double re_error;

int main(int argc, char** argv) {
	ros::init(argc, argv, "alex_gripper_control_node", ros::init_options::NoSigintHandler);

	ros::NodeHandle nh;

	// ros::Subscriber phase_space_sub =  nh.subscribe("joint_states", 1, statesCallback);
	ros::Publisher ls_torque = nh.advertise<std_msgs::Float64>("set_left_shoulder_effort", 1);
	ros::Publisher le_torque = nh.advertise<std_msgs::Float64>("set_left_elbow_effort", 1);
	ros::Publisher re_torque = nh.advertise<std_msgs::Float64>("set_right_elbow_effort", 1);

	double rate;
	ros::param::param<double>("~loop_rate", rate, 200);
	ros::Rate loop_rate(rate);

	double errorq[3];
	double error_qdot[3];
	double torq[3];

	double Kp = 0.0002;
	double Kd = 0.00002;
	std_msgs::Float64 torque;

	while (ros::ok()) {
		// robot.readEncoders()
		ls_position = robot.getPos(LEFT_SHOULDER);
		le_position = robot.getPos(LEFT_ELBOW);
		re_position = robot.getPos(RIGHT_ELBOW);

		ls_velocity = robot.getVel(LEFT_SHOULDER);
		le_velocity = robot.getVel(LEFT_ELBOW);
		re_velocity = robot.getVel(RIGHT_ELBOW);

		errorq[0] = 0.1 - ls_position;
		errorq[1] = 0.1 - le_position;
		errorq[2] = 0.1 - re_position;

		error_qdot[0] = 0 - ls_velocity;
		error_qdot[1] = 0 - le_velocity;
		error_qdot[2] = 0 - re_velocity;

		torq[0] = (Kp * errorq[0]) + (Kd * error_qdot[0]);
		torq[1] = (Kp * errorq[1]) + (Kd * error_qdot[1]);
		torq[2] = (Kp * errorq[2]) + (Kd * error_qdot[2]);

		torque.data = torq[0];
		ls_torque.publish(torque);

		torque.data = torq[1];
		le_torque.publish(torque);

		torque.data = torq[2];
		re_torque.publish(torque);

		loop_rate.sleep();
	}

	return 0;
}
