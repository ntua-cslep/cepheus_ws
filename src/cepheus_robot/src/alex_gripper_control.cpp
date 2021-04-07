#include <signal.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

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


#include "digital_filter.h"

#define POS_FILTER 0.005
#define VEL_FILTER 0.05
#define TORQUE_LIMIT 0.00000001

DigitalFilter *f_x1;
DigitalFilter *f_x2;
DigitalFilter *f_x3;
DigitalFilter *f_y1;
DigitalFilter *f_y2;
DigitalFilter *f_y3;
DigitalFilter *f_z1;
DigitalFilter *f_z2;
DigitalFilter *f_z3;

enum PhaseSpaceEnumeration{
	CEPHEUS = 0,
	ALX_GRIPPER,
	ASSIST
};

double ls_position = 0.0, le_position = 0.0, re_position = 0.0;
// double ls_position_prev, le_position_prev, re_position_prev;
double ls_velocity = 0.0, le_velocity = 0.0, re_velocity = 0.0;
// double ls_velocity_prev, le_velocity_prev, re_velocity_prev;
bool ls_initialized = false;
bool le_initialized = false;
bool re_initialized = false;
bool ls_first_time = true;
bool le_first_time = true;
bool re_first_time = true;
bool start_moving = false;
bool first_time_movement = true;
bool start_docking = false;
bool first_time_before_docking = true;
bool first_time_docking = true;
bool first_time_penetrating = true;
bool first_time_after_docking = true;

double ps_x[3];
double ps_y[3];
double ps_th[3];

double q1_init = -60 * (M_PI / 180);
double q2_init = 105 * (M_PI / 180);
double q3_init = 45 * (M_PI / 180);

double Kp = 0.06;
double Kd = 0.006;


void PSAlxCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
	geometry_msgs::TransformStamped temp;
	temp = *msg;

	double x = temp.transform.rotation.x;
	double y = temp.transform.rotation.y;
	double z = temp.transform.rotation.z;
	double w = temp.transform.rotation.w;
	double roll,pitch,yaw;

	tf::Quaternion q(x, y, z, w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll,pitch,yaw);

	// pose_stamp = temp.header.stamp;
	ps_x[ALX_GRIPPER] = f_x1->filter(temp.transform.translation.x);
	ps_y[ALX_GRIPPER] = f_y1->filter(temp.transform.translation.y);
	ps_th[ALX_GRIPPER] = f_z1->filter(yaw);
	return;
}


void PSAssistCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
	geometry_msgs::TransformStamped temp;
	temp = *msg;

	double x = temp.transform.rotation.x;
	double y = temp.transform.rotation.y;
	double z = temp.transform.rotation.z;
	double w = temp.transform.rotation.w;
	double roll,pitch,yaw;

	tf::Quaternion q(x, y, z, w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

	// pose_stamp = temp.header.stamp;
	ps_x[ASSIST] = f_x2->filter(temp.transform.translation.x);
	ps_y[ASSIST] = f_y2->filter(temp.transform.translation.y);
	ps_th[ASSIST] = f_z2->filter(yaw);
	return;
}


void PSCepheusCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
	geometry_msgs::TransformStamped temp;
	temp = *msg;

	double x = temp.transform.rotation.x;
	double y = temp.transform.rotation.y;
	double z = temp.transform.rotation.z;
	double w = temp.transform.rotation.w;
	double roll, pitch, yaw;

	tf::Quaternion q(x, y, z, w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

	// pose_stamp = temp.header.stamp;
	ps_x[CEPHEUS] = f_x3->filter(temp.transform.translation.x);
	ps_y[CEPHEUS] = f_y3->filter(temp.transform.translation.y);
	ps_th[CEPHEUS] = f_z3->filter(yaw);
	return;
}


void lsPosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - ls_position) > POS_FILTER)
		return;
	else
		ls_position = cmd->data;
}


void lePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - le_position) > POS_FILTER)
		return;
	else
		le_position = cmd->data;
}

void rePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - re_position) > POS_FILTER)
		return;
	else
		re_position = cmd->data;
}


void lsVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - ls_velocity) > VEL_FILTER)
		return;
	else
		ls_velocity = cmd->data;
}


void leVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - le_velocity) > VEL_FILTER)
		return;
	else
		le_velocity = cmd->data;
}


void reVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - re_velocity) > VEL_FILTER)
		return;
	else
		re_velocity = cmd->data;
}


void lsLimitCallback(const std_msgs::UInt8::ConstPtr& cmd) {
	if (cmd->data == 1)
		ls_initialized = true;
}


void leLimitCallback(const std_msgs::UInt8::ConstPtr& cmd) {
	if (cmd->data == 1)
		le_initialized = true;
}


void reLimitCallback(const std_msgs::UInt8::ConstPtr& cmd) {
	if (cmd->data == 1)
		re_initialized = true;
}


void startMovingCallback(const std_msgs::Bool::ConstPtr& msg) {
	start_moving = true;
}


void startDockingCallback(const std_msgs::Bool::ConstPtr& msg) {
	start_docking = true;
}


void resetMovementCallback(const std_msgs::Bool::ConstPtr& msg) {
	start_moving = false;
	first_time_movement = true;
	start_docking = false;
	first_time_before_docking = true;
	first_time_docking = true;
	first_time_penetrating = true;
	first_time_after_docking = true;
	q1_init = -60 * (M_PI / 180);
	q2_init = 105 * (M_PI / 180);
	q3_init = 45 * (M_PI / 180);
}


void resetDockingCallback(const std_msgs::Bool::ConstPtr& msg) {
	start_docking = false;
	first_time_before_docking = true;
	first_time_docking = true;
	first_time_penetrating = true;
	first_time_after_docking = true;
}


void setGainsCallback(const std_msgs::Float64::ConstPtr& cmd) {
	Kp = cmd->data;
	Kd = Kp / 10;
	ROS_INFO("SET: Kp:  %f   Kd:  %f", Kp, Kd);
}


double filter_torque(double torq, double prev) {
	if (torq == 0.0){
		// torq = 0.00001;
		torq = 0.00001;
		if (prev < 0.0)
			torq = torq * -1;
		printf("CHANGED ZERO TORQUE\n");
	}
	return torq;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "alex_gripper_control_node", ros::init_options::NoSigintHandler);

	ros::NodeHandle nh;
	f_x1 = new DigitalFilter(10, 0.0);
	f_x2 = new DigitalFilter(10, 0.0);
	f_x3 = new DigitalFilter(10, 0.0);
	f_y1 = new DigitalFilter(10, 0.0);
	f_y2 = new DigitalFilter(10, 0.0);
	f_y3 = new DigitalFilter(10, 0.0);
	f_z1 = new DigitalFilter(10, 0.0);
	f_z2 = new DigitalFilter(10, 0.0);
	f_z3 = new DigitalFilter(10, 0.0);
	
	// ros::Subscriber ps_alx_sub =  nh.subscribe("map_to_alxgripper", 1, PSAlxCallback);
	// ros::Subscriber ps_assist_sub =  nh.subscribe("map_to_assist_robot", 1, PSAssistCallback);
	ros::Subscriber ps_cepheus_sub =  nh.subscribe("map_to_cepheus", 1, PSCepheusCallback);

	// ros::Subscriber phase_space_sub =  nh.subscribe("joint_states", 1, statesCallback);
	ros::Publisher ls_torque_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_effort", 1);
	ros::Publisher le_torque_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_effort", 1);
	ros::Publisher re_torque_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_effort", 1);
	ros::Publisher ls_offset_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_offset", 1);
	ros::Publisher le_offset_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_offset", 1);
	ros::Publisher re_offset_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_offset", 1);
	ros::Publisher ls_pos_pub = nh.advertise<std_msgs::Float64>("left_shoulder_pos", 1);
	ros::Publisher le_pos_pub = nh.advertise<std_msgs::Float64>("left_elbow_pos", 1);
	ros::Publisher re_pos_pub = nh.advertise<std_msgs::Float64>("right_elbow_pos", 1);
	ros::Publisher ls_vel_pub = nh.advertise<std_msgs::Float64>("left_shoulder_vel", 1);
	ros::Publisher le_vel_pub = nh.advertise<std_msgs::Float64>("left_elbow_vel", 1);
	ros::Publisher re_vel_pub = nh.advertise<std_msgs::Float64>("right_elbow_vel", 1);
	ros::Publisher ls_qd_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_qd", 1);
	ros::Publisher le_qd_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_qd", 1);
	ros::Publisher re_qd_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_qd", 1);
	ros::Publisher ls_error_pub = nh.advertise<std_msgs::Float64>("ls_error", 1);
	ros::Publisher le_error_pub = nh.advertise<std_msgs::Float64>("le_error", 1);
	ros::Publisher re_error_pub = nh.advertise<std_msgs::Float64>("re_error", 1);
	ros::Publisher gripper_x_pub = nh.advertise<std_msgs::Float64>("g_x", 1);
	ros::Publisher gripper_y_pub = nh.advertise<std_msgs::Float64>("g_y", 1);
	ros::Publisher gripper_th_pub = nh.advertise<std_msgs::Float64>("g_th", 1);
	ros::Publisher gripper_x_dot_pub = nh.advertise<std_msgs::Float64>("g_x_dot", 1);
	ros::Publisher gripper_y_dot_pub = nh.advertise<std_msgs::Float64>("g_y_dot", 1);
	ros::Publisher gripper_th_dot_pub = nh.advertise<std_msgs::Float64>("g_th_dot", 1);
	ros::Publisher gripper_x_desdot_pub = nh.advertise<std_msgs::Float64>("g_x_desdot", 1);
	ros::Publisher cepheus_x_pub = nh.advertise<std_msgs::Float64>("c_x", 1);
	ros::Publisher cepheus_y_pub = nh.advertise<std_msgs::Float64>("c_y", 1);
	ros::Publisher cepheus_th_pub = nh.advertise<std_msgs::Float64>("c_th", 1);
	ros::Publisher gripper_error_x_pub = nh.advertise<std_msgs::Float64>("g_error_x", 1);
	ros::Publisher gripper_error_y_pub = nh.advertise<std_msgs::Float64>("g_error_y", 1);
	ros::Publisher gripper_error_th_pub = nh.advertise<std_msgs::Float64>("g_error_th", 1);
	ros::Publisher det_pub = nh.advertise<std_msgs::Float64>("det", 1);

	ros::Publisher lar_pub = nh.advertise<std_msgs::UInt16>("LAR", 1);
	ros::Publisher secs_pub = nh.advertise<std_msgs::Float64>("secs", 1);

	ros::Subscriber ls_pos_sub = nh.subscribe("read_left_shoulder_position", 1, lsPosCallback);
	ros::Subscriber le_pos_sub = nh.subscribe("read_left_elbow_position", 1, lePosCallback);
	ros::Subscriber re_pos_sub = nh.subscribe("read_right_elbow_position", 1, rePosCallback);
	ros::Subscriber ls_vel_sub = nh.subscribe("read_left_shoulder_velocity", 1, lsVelCallback);
	ros::Subscriber le_vel_sub = nh.subscribe("read_left_elbow_velocity", 1, leVelCallback);
	ros::Subscriber re_vel_sub = nh.subscribe("read_right_elbow_velocity", 1, reVelCallback);
	ros::Subscriber ls_limit_sub = nh.subscribe("read_left_shoulder_limit", 1, lsLimitCallback);
	ros::Subscriber le_limit_sub = nh.subscribe("read_left_elbow_limit", 1, leLimitCallback);
	ros::Subscriber re_limit_sub = nh.subscribe("read_right_elbow_limit", 1, reLimitCallback);

	ros::Subscriber start_moving_sub = nh.subscribe("start_moving", 1, startMovingCallback);
	ros::Subscriber start_docking_sub = nh.subscribe("start_docking", 1, startDockingCallback);

	ros::Subscriber reset_movement_sub = nh.subscribe("reset_movement", 1, resetMovementCallback);
	ros::Subscriber reset_docking_sub = nh.subscribe("reset_docking", 1, resetDockingCallback);

	ros::Subscriber set_gains_sub = nh.subscribe("set_gains", 1, setGainsCallback);

	double rate;
	ros::param::param<double>("~loop_rate", rate, 200);
	ros::Rate loop_rate(rate);

	double errorq[3];
	double error_qdot[3];
	double torq[3];
	double prev_torq[3];
	double qd[3];
	double qd_dot[3];

	for (int i = 0; i < 3; i++) {
		errorq[i] = 0.0;
		error_qdot[i] = 0.0;
		torq[i] = 0.0;
		prev_torq[i] = 0.0;
		qd[i] = 0.0;
		qd_dot[i] = 0.0;
	}

	double theta0_des = 90 * (M_PI / 180);
	double theta0_desdot = 0.0;

	double a00 = 2.328090369367525;
	double a01 = 0.331762469609965;
	double a02 = 0.247736831332797;
	double a03 = 0.152479037019120;
	double a11 = 0.768198737850671;
	double a21 = 0.573237053631319;
	double a31 = 0.352820504933170;
	double a22 = 0.443678774166903;
	double a32 = 0.273792231852408;
	double a33 = 0.269608750820881;
	double a12 = 0.573237053631319;
	double a23 = 0.273792231852408;
	double a13 = 0.352820504933170;
	double a, b, c, d, d0, d1, d2, d3;
	double d00, d10, d20, d30, d01, d02, d03, d11, d21, d31, d12, d22, d32, d13, d23, d33;
	double s, s_dot, S;
	double xe_desdot;
	double determinant = 0.0;
	// values from launch file
	double qd1_init = 1.328828;
	double qd2_init = 2.22;
	double qd3_init = 1.777316;
	// docking final joins' angles
	double q1_fin_docking = -0.16;
	double q2_fin_docking = 1.84;
	double q3_fin_docking = 1.04;
	// double q1_fin_pen = -0.12966;
	double q1_fin_pen = -0.179;
	double q2_fin_pen = 1.8707;
	double q3_fin_pen = 1.193;

	ros::Time curr_time, t_beg = ros::Time::now();
	ros::Duration all_time;
	double prev_secs = 0.0;
	bool ls_moved = false;
	bool re_moved = false;
	double le_position_after_ls_init, re_position_after_ls_init;

	// phase space movement
	double gripper_x_init, gripper_y_init, gripper_th_init;
	double gripper_x = 0.0, gripper_y = 0.0, gripper_th = 0.0;
	double lar_x_fin, lar_y_fin, lar_th_fin;
	double gripper_x_dot = 0.0, gripper_y_dot = 0.0, gripper_th_dot = 0.0;
	double gripper_x_des, gripper_y_des, gripper_th_des;
	double gripper_x_desdot = 0.0, gripper_y_desdot, gripper_th_desdot;
	double cepheus_x, cepheus_y, cepheus_th;
	double gripper_error_x = 0.0, gripper_error_y = 0.0, gripper_error_th = 0.0;
	double gripper_error_x_dot = 0.0, gripper_error_y_dot = 0.0, gripper_error_th_dot = 0.0;
	double q1, q2, q3;
	double x_prev, y_prev, th_prev;

	std_msgs::Float64 torque;
	std_msgs::Float64 offset;
	std_msgs::Float64 temp_msg;
	std_msgs::Float64 _secs;
	std_msgs::UInt16 lar;
	bool initialized = false;
	sleep(4);
	double secs, ls_time, le_time, re_time, move_time, docking_time, penetration_time;

	while (ros::ok()) {

		for (int i = 0; i < 3; i++) {
			prev_torq[i] = torq[i];
		}

		curr_time = ros::Time::now();
		all_time = curr_time - t_beg;

		secs = all_time.sec + all_time.nsec * pow(10, -9);

		if (!initialized) {
			if (!ls_initialized) {

				error_qdot[0] = 0.08 + ls_velocity;
				error_qdot[1] = -0.0001 - le_velocity;
				error_qdot[2] = -0.0001 + re_velocity;

				errorq[0] = error_qdot[0] * (secs - prev_secs) + errorq[0];
				errorq[1] = error_qdot[1] * (secs - prev_secs) + errorq[1];
				errorq[2] = error_qdot[2] * (secs - prev_secs) + errorq[2];

				torq[0] = - (Kp/5 * error_qdot[0] + Kd/5 * errorq[0]);
				torq[1] = Kp/6 * error_qdot[1] + Kd/6 * errorq[1];
				torq[2] = - (Kp/5 * error_qdot[2] + Kd/5 * errorq[2]);

			} else if (!re_initialized) {
				if (!ls_moved) {
					// move ls to pose position
					if (ls_first_time) {
						//calculate and set offset
						offset.data = qd1_init + ls_position;
						ls_offset_pub.publish(offset);

						ls_first_time = false;
						ls_time = secs;
						ls_position = qd1_init;
						le_position_after_ls_init = le_position;
						re_position_after_ls_init = re_position;
					}

					s = (0.000001875 * pow(secs - ls_time, 5)) + (-0.00009375000000000002 * pow(secs - ls_time, 4)) + (0.00125 * pow(secs - ls_time, 3));
					s_dot = (5 * 0.000001875 * pow(secs - ls_time, 4)) + (4 * -0.00009375000000000002 * pow(secs - ls_time, 3)) + (3 * 0.00125  * pow(secs - ls_time, 2));

					qd[0] = qd1_init + (q1_init - qd1_init) * s;
					qd_dot[0] = (q1_init - qd1_init) * s_dot;

					error_qdot[0] = qd_dot[0] - ls_velocity;
					error_qdot[1] = 0 - le_velocity;
					error_qdot[2] = 0 + re_velocity;
					
					errorq[0] = qd[0] - ls_position;
					errorq[1] = le_position_after_ls_init - le_position;
					errorq[2] = re_position_after_ls_init + re_position;

					torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
					torq[1] = Kp/2 * errorq[1] + Kd/2 * error_qdot[1];
					torq[2] = - (Kp/8 * errorq[2] + Kd/8 * error_qdot[2]);

					// this movement is 20 secs
					if (secs - ls_time >= 20.0)
						ls_moved = true;
						
				} else {
  
					error_qdot[0] = 0 - ls_velocity;
					error_qdot[1] = 0 - le_velocity;
					error_qdot[2] = 0.08 + re_velocity;

					errorq[0] = q1_init - ls_position;
					errorq[1] = le_position_after_ls_init - le_position;
					errorq[2] = error_qdot[2] * (secs - prev_secs) + errorq[2];

					torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
					torq[1] = Kp/2 * errorq[1] + Kd/2 * error_qdot[1];
					torq[2] = - (Kp/5 * error_qdot[2] + Kd/5 * errorq[2]);
				}
			} else if (!le_initialized) {
				if (!re_moved) {
					// move re to pose position
					if (re_first_time) {
						//calculate and set offset
						offset.data = qd3_init + re_position;
						re_offset_pub.publish(offset);

						re_first_time = false;
						re_time = secs;
						re_position = qd3_init;
					}

					s = (0.00006 * pow(secs - re_time, 5)) + (-0.0015 * pow(secs - re_time, 4)) + (0.01 * pow(secs - re_time, 3));
					s_dot = (5 * 0.00006 * pow(secs - re_time, 4)) + (4 * -0.0015 * pow(secs - re_time, 3)) + (3 * 0.01 * pow(secs - re_time, 2));

					qd[2] = qd3_init + (q3_init - qd3_init) * s;
					qd_dot[2] = (q3_init - qd3_init) * s_dot;

					error_qdot[0] = 0 - ls_velocity;
					error_qdot[1] = 0 - le_velocity;
					error_qdot[2] = qd_dot[2] - re_velocity;

					errorq[0] = q1_init - ls_position;
					errorq[1] = le_position_after_ls_init - le_position;
					errorq[2] = qd[2] - re_position;

					torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
					torq[1] = Kp * errorq[1] + Kd * error_qdot[1];
					torq[2] = - (1.5*Kp * errorq[2] + 1.5*Kd * error_qdot[2]);

					// this movement is 10 secs
					if (secs - re_time >= 10.0)
						re_moved = true;

				} else {
						
					error_qdot[0] = 0 - ls_velocity;
					error_qdot[1] = 0.08 - le_velocity;
					error_qdot[2] = 0 - re_velocity;

					errorq[0] = q1_init - ls_position;
					errorq[1] = error_qdot[1] * (secs - prev_secs) + errorq[1];
					errorq[2] = q3_init - re_position;

					torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
					torq[1] = Kp/6 * error_qdot[1] + Kd/6 * errorq[1];
					torq[2] = - (1.5*Kp * errorq[2] + 1.5*Kd * error_qdot[2]);
				}
			} else {
				// move re to pose position
				if (le_first_time) {
					//calculate and set offset
					offset.data = qd2_init - le_position;
					le_offset_pub.publish(offset);

					le_first_time = false;
					le_time = secs;
					le_position = qd2_init;
				}

				s = (0.00006 * pow(secs - le_time, 5)) + (-0.0015 * pow(secs - le_time, 4)) + (0.01 * pow(secs - le_time, 3));
				s_dot = (5 * 0.00006 * pow(secs - le_time, 4)) + (4 * -0.0015 * pow(secs - le_time, 3)) + (3 * 0.01 * pow(secs - le_time, 2));

				qd[1] = qd2_init + (q2_init - qd2_init) * s;
				qd_dot[1] = (q2_init - qd2_init) * s_dot;

				error_qdot[0] = 0 - ls_velocity;
				error_qdot[1] = qd_dot[1] - le_velocity;
				error_qdot[2] = 0 - re_velocity;

				errorq[0] = q1_init - ls_position;
				errorq[1] = qd[1] - le_position;
				errorq[2] = q3_init - re_position;

				torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
				torq[1] = 2*Kp * errorq[1] + 2*Kd * error_qdot[1];
				torq[2] = - (1.5*Kp * errorq[2] + 1.5*Kd * error_qdot[2]);

				// this movement is 10 secs
				if (secs - le_time >= 10.0)
					initialized = true;
			}
		} else if (!start_moving) {
			// stay there until further notice
			// ROS_INFO("WAITING FOR START MOVING");
			error_qdot[0] = 0 - ls_velocity;
			error_qdot[1] = 0 - le_velocity;
			error_qdot[2] = 0 - re_velocity;

			errorq[0] = q1_init - ls_position;
			errorq[1] = q2_init - le_position;
			errorq[2] = q3_init - re_position;
			// ROS_INFO("ERRORQ:  %f   %f   %f", errorq[0], errorq[1], errorq[2]);
			// ROS_INFO("pos:  %f   %f   %f", ls_position, le_position,re_position);

			torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
			torq[1] = 1.5*Kp * errorq[1] + 1.5*Kd * error_qdot[1];
			torq[2] = - (1.5*Kp * errorq[2] + 1.5*Kd * error_qdot[2]);

		} else {
			// calculate secs
			if (first_time_movement) {
				move_time = secs;
				// q1 = ls_position;
				// q2 = le_position;
				// q3 = re_position;
				// cepheus_x = ps_x[CEPHEUS];
				// cepheus_y = ps_y[CEPHEUS];
				// cepheus_th = ps_th[CEPHEUS];
				// gripper_x_init = cepheus_x + 0.1645 * cos(cepheus_th) + 0.370 * cos(cepheus_th + q1) + 0.286 * cos(cepheus_th + q1 + q2) + 0.28971 * cos(cepheus_th + q1 + q2 + q3);
				// gripper_y_init = cepheus_y + 0.1645 * sin(cepheus_th) + 0.370 * sin(cepheus_th + q1) + 0.286 * sin(cepheus_th + q1 + q2) + 0.28971 * sin(cepheus_th + q1 + q2 + q3);
				// gripper_th_init = cepheus_th + q1 + q2 + q3;
				// gripper_x = gripper_x_init;
				// gripper_y = gripper_y_init;
				// gripper_th = gripper_th_init;
				// x_prev = gripper_x_init;
				// y_prev = gripper_y_init;
				// th_prev = gripper_th_init;
				// lar_x_fin = gripper_x_init + 0.1;
				// lar_y_fin = gripper_y_init;
				// lar_th_fin = gripper_th_init;
				// gripper_x_dot = 0.0;
				// gripper_y_dot = 0.0;
				// gripper_th_dot = 0.0;
				first_time_movement = false;
				a = 0.143318141419581;
				b = 0.331623152993806;
				c = 0.257342875265441;
				d = 0.257361884747802;
				qd[0] = q1_init;
				qd[1] = q2_init;
				qd[2] = q3_init;
				Kp = 0.06;
				Kd = 0.0006;
			}
			if ((secs - move_time) <= 20.0) {
				// move 10 cm
				d00 = a00;
				d10 = a01 * cos(qd[0]);
				d20 = a02 * cos(qd[0] + qd[1]);
				d30 = a03 * cos(qd[0] + qd[1] + qd[2]);
				d01 = d10;
				d11 = a11;
				d21 = a21 * cos(qd[1]);
				d31 = a31 * cos(qd[1] + qd[2]);
				d02 = d20;
				d12 = d21;
				d22 = a22;
				d32 = a32 * cos(qd[2]);
				d03 = d30;
				d13 = d31;
				d23 = d32;
				d33 = a33;
				d0 = d00 + d10 + d20 + d30;
				d1 = d01 + d11 + d21 + d31;
				d2 = d02 + d12 + d22 + d32;
				d3 = d03 + d13 + d23 + d33;

				S = a * b * d2 *sin(qd[0]) + b * c * d0 * sin(qd[1]) - a * c * d1 *sin(qd[0] + qd[1]);
				s_dot = (5 * 0.000001875 * pow(secs - move_time, 4)) + (4 * -0.00009375000000000002 * pow(secs - move_time, 3)) + (3 * 0.00125  * pow(secs - move_time, 2));
				xe_desdot = - 0.1 * s_dot;

				theta0_desdot = ((b*d2*cos(theta0_des+qd[0])-c*d1*cos(theta0_des+qd[0]+qd[1]))*xe_desdot)/S;
				qd_dot[0] = ((-d2*(a*cos(theta0_des)+b*cos(theta0_des+qd[0]))+c*(d0+d1)*cos(theta0_des+qd[0]+qd[1]))*xe_desdot)/S; 
				qd_dot[1] = ((a*(d1+d2)*cos(theta0_des)-d0*(b*cos(theta0_des+qd[0])+c*cos(theta0_des+qd[0]+qd[1])))*xe_desdot)/S;
				qd_dot[2] = ((-a*d1*cos(theta0_des)+b*d0*cos(theta0_des+qd[0]))*xe_desdot)/S;

				qd[0] = qd_dot[0] * (secs - prev_secs) + qd[0];
				qd[1] = qd_dot[1] * (secs - prev_secs) + qd[1];
				qd[2] = qd_dot[2] * (secs - prev_secs) + qd[2];
				theta0_des = theta0_desdot * (secs - prev_secs) + theta0_des;

				error_qdot[0] = qd_dot[0] - ls_velocity;
				error_qdot[1] = qd_dot[1] - le_velocity;
				error_qdot[2] = qd_dot[2] - re_velocity;

				errorq[0] = qd[0] - ls_position;
				errorq[1] = qd[1] - le_position;
				errorq[2] = qd[2] - re_position;

				// torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
				// torq[1] = 1.5*Kp * errorq[1] + 1.5*Kd * error_qdot[1];
				// torq[2] = - (2.3*Kp * errorq[2] + 2.3*Kd * error_qdot[2]);

				torq[0] = - (58.42645738* errorq[0] +  1.047501073* error_qdot[0])/186;
				torq[1] = (40.54481 * errorq[1] + 1.23743613 * error_qdot[1])/186;
				torq[2] = - (30.90236545*errorq[2] + 1.089994475 * error_qdot[2])/186;
	
				// gripper_x_des = gripper_x_init + (lar_x_fin - gripper_x_init) * s;
				// gripper_y_des = gripper_y_init;
				// gripper_th_des = gripper_th_init;
				// gripper_x_desdot = s_dot * (lar_x_fin - gripper_x_init);
				// gripper_y_desdot = 0.0;
				// gripper_th_desdot = 0.0;

				// cepheus_x = ps_x[CEPHEUS];
				// cepheus_y = ps_y[CEPHEUS];
				// cepheus_th = ps_th[CEPHEUS];

				// q1 = ls_position;
				// q2 = le_position;
				// q3 = re_position;

				// gripper_x = cepheus_x + 0.1645 * cos(cepheus_th) + 0.370 * cos(cepheus_th + q1) + 0.286 * cos(cepheus_th + q1 + q2) + 0.28971 * cos(cepheus_th + q1 + q2 + q3);
				// gripper_y = cepheus_y + 0.1645 * sin(cepheus_th) + 0.370 * sin(cepheus_th + q1) + 0.286 * sin(cepheus_th + q1 + q2) + 0.28971 * sin(cepheus_th + q1 + q2 + q3);
				// gripper_th = cepheus_th + q1 + q2 + q3;

				// gripper_x_dot = (gripper_x - x_prev) / (secs - prev_secs);
				// gripper_y_dot = (gripper_y - y_prev) / (secs - prev_secs);
				// gripper_th_dot = (gripper_th - th_prev) / (secs - prev_secs);

				// gripper_error_x = gripper_x_des - gripper_x;
				// gripper_error_y = gripper_y_des - gripper_y;
				// gripper_error_th = gripper_th_des - gripper_th;

				// Eigen::Vector3d gripper_error(gripper_error_x, gripper_error_y, gripper_error_th);

				// gripper_error_x_dot = gripper_x_desdot - gripper_x_dot;
				// gripper_error_y_dot = gripper_y_desdot - gripper_y_dot;
				// gripper_error_th_dot = gripper_th_desdot - gripper_th_dot;

				// Eigen::Vector3d gripper_error_dot(gripper_error_x_dot, gripper_error_y_dot, gripper_error_th_dot);

				// Eigen::Matrix3d R0;
				// R0 << cos(cepheus_th), -sin(cepheus_th), 	0.0,
				//       sin(cepheus_th), cos(cepheus_th), 	0.0,
				//       0.0,  	0.0, 		1.0;
				
				
				// Eigen::Matrix3d Kpp;
				// Kpp << Kp, 0.0, 0.0,
				// 	  0.0, Kp, 0.0,
				// 	  0.0, 0.0, Kp;

				// Eigen::Matrix3d Kdd;
				// Kdd << Kd, 0.0, 0.0,
				// 		0.0, Kd, 0.0,
				// 		0.0, 0.0, Kd;

				// Eigen::Matrix3d Jq;
				// Jq	<<	(-1)*(a00+a01*cos(q1)+a02*cos(q1+q2)+a03*cos(q1+q2+q3))*
				// 		pow((a00+a11+a22+a33+2*a01*cos(q1)+2*a12*cos(q2)+2*a02*cos(q1+q2)
				// 		+2*a23*cos(q3)+2*a13*cos(q2+q3)+2*a03*cos(q1+q2+q3)),(-1))*
				// 		(b*sin(q1)+c*sin(q1+q2)+d*sin(q1+q2+q3)),
				// 		(-1)*c*sin(q1+q2)+(-1)*d*sin(q1+q2+q3)+(a22+a33+a12*cos(q2)+a02*cos(q1+q2)+2*
				// 		a23*cos(q3)+a13*cos(q2+q3)+a03*cos(q1+q2+q3))*pow((a00+a11+a22+
				// 		a33+2*a01*cos(q1)+2*a12*cos(q2)+2*a02*cos(q1+q2)+2*a23*
				// 		cos(q3)+2*a13*cos(q2+q3)+2*a03*cos(q1+q2+q3)),(-1))*(b*sin(
				// 		q1)+c*sin(q1+q2)+d*sin(q1+q2+q3)),
				// 		(-1)*d*sin(q1+q2+q3)+(a33+a23*cos(q3)+a13*cos(q2+q3)+a03*cos(q1+q2+q3))*pow((a00+a11+a22+
				// 		a33+2*a01*cos(q1)+2*a12*cos(q2)+2*a02*cos(q1+q2)+2*a23*
				// 		cos(q3)+2*a13*cos(q2+q3)+2*a03*cos(q1+q2+q3)),(-1))*(b*sin(
				// 		q1)+c*sin(q1+q2)+d*sin(q1+q2+q3)),
				// 		b*cos(q1)+c*cos(q1+q2)+d*cos(q1+q2+q3)+(-1)*(a11+a22+a33+a01*cos(q1)+2*a12*cos(q2)+
				// 		a02*cos(q1+q2)+2*a23*cos(q3)+2*a13*cos(q2+q3)+a03*cos(q1+q2+
				// 		q3))*pow((a00+a11+a22+a33+2*a01*cos(q1)+2*a12*cos(q2)+2*a02*
				// 		cos(q1+q2)+2*a23*cos(q3)+2*a13*cos(q2+q3)+2*a03*cos(q1+q2+
				// 		q3)),(-1))*(a+b*cos(q1)+c*cos(q1+q2)+d*cos(q1+q2+q3)),
				// 		c*cos(q1+q2)+d*cos(q1+q2+q3)+(-1)*(a22+a33+a12*cos(q2)+a02*cos(q1+
				// 		q2)+2*a23*cos(q3)+a13*cos(q2+q3)+a03*cos(q1+q2+q3))*pow((a00+a11+
				// 		a22+a33+2*a01*cos(q1)+2*a12*cos(q2)+2*a02*cos(q1+q2)+2*
				// 		a23*cos(q3)+2*a13*cos(q2+q3)+2*a03*cos(q1+q2+q3)),(-1))*(a+
				// 		b*cos(q1)+c*cos(q1+q2)+d*cos(q1+q2+q3)),
				// 		d*cos(q1+q2+q3)+(-1)*(a33+a23*cos(q3)+a13*cos(q2+q3)+a03*cos(q1+q2+q3))*pow((a00+a11+
				// 		a22+a33+2*a01*cos(q1)+2*a12*cos(q2)+2*a02*cos(q1+q2)+2*
				// 		a23*cos(q3)+2*a13*cos(q2+q3)+2*a03*cos(q1+q2+q3)),(-1))*(a+
				// 		b*cos(q1)+c*cos(q1+q2)+d*cos(q1+q2+q3)),
				// 		(a00+a01*cos(q1)+a02*
				// 		cos(q1+q2)+a03*cos(q1+q2+q3))*pow((a00+a11+a22+a33+2*a01*cos(q1)+
				// 		2*a12*cos(q2)+2*a02*cos(q1+q2)+2*a23*cos(q3)+2*a13*cos(q2+
				// 		q3)+2*a03*cos(q1+q2+q3)),(-1)),
				// 		(a00+a11+2*a01*cos(q1)+a12*cos(q2)+a02*cos(q1+q2)+a13*cos(q2+q3)+a03*cos(q1+q2+q3))*pow((a00+
				// 		a11+a22+a33+2*a01*cos(q1)+2*a12*cos(q2)+2*a02*cos(q1+q2)+2*
				// 		a23*cos(q3)+2*a13*cos(q2+q3)+2*a03*cos(q1+q2+q3)),(-1)),
				// 		1+(-1)*(a33+a23*cos(q3)+a13*cos(q2+q3)+a03*cos(q1+q2+q3))*pow((a00+
				// 		a11+a22+a33+2*a01*cos(q1)+2*a12*cos(q2)+2*a02*cos(q1+q2)+2*
				// 		a23*cos(q3)+2*a13*cos(q2+q3)+2*a03*cos(q1+q2+q3)),(-1));
				
				// determinant = Jq.determinant();
				// Eigen::Vector3d torque_vector;
				// Eigen::Matrix3d xxx = R0*Jq;
				// Eigen::Matrix3d xxx1 = xxx.transpose();
				// torque_vector = xxx1 * ((Kpp * gripper_error) + (Kdd * gripper_error_dot));

				// torq[0] = - torque_vector(0);
				// torq[1] = torque_vector(1);
				// torq[2] = - torque_vector(2);


				// x_prev = gripper_x;
				// y_prev = gripper_y;
				// th_prev = gripper_th;
			} 
			else {
				if (!start_docking) {
					if (first_time_before_docking) {
						Kp = 0.06;
						Kd = 0.0006;
						errorq[0] = 0.0;
						errorq[1] = 0.0;
						errorq[2] = 0.0;
						first_time_before_docking = false;
					}

					qd_dot[0] = 0.0001;
					qd_dot[1] = 0.0001;
					qd_dot[2] = 0.0001;

					error_qdot[0] = qd_dot[0] - ls_velocity;
					error_qdot[1] = qd_dot[1] - le_velocity;
					error_qdot[2] = qd_dot[2] - re_velocity;

					errorq[0] = error_qdot[0] * (secs - prev_secs) + errorq[0];
					errorq[1] = error_qdot[1] * (secs - prev_secs) + errorq[1];
					errorq[2] = error_qdot[2] * (secs - prev_secs) + errorq[2];

					torq[0] = - (Kp/16 * errorq[0] + Kd/16 * error_qdot[0]);
					torq[1] = Kp/16 * errorq[1] + Kd/16 * error_qdot[1];
					torq[2] = - (Kp/16 * errorq[2] + Kd/16 * error_qdot[2]);

					// ROS_INFO("ERRORQ:  %f   %f   %f", errorq[0], errorq[1], errorq[2]);
					// ROS_INFO("pos:  %f   %f   %f", ls_position, le_position,re_position);

					ROS_INFO("ls(-0.577): %lf	    le(1.89): %lf       re(0.40): %lf", ls_position, le_position, re_position);
					// PUBLISH LAR
					// lar.data = 1;
					// lar_pub.publish(lar);
				} else {
					if (first_time_docking) {
						docking_time = secs;
						q1_init = ls_position;
						q2_init = le_position;
						q3_init = re_position;
						first_time_docking = false;
					}
					if ((secs - docking_time) <= 20.0) {
						s = (0.000001875 * pow(secs - docking_time, 5)) + (-0.00009375000000000002 * pow(secs - docking_time, 4)) + (0.00125 * pow(secs - docking_time, 3));
						s_dot = (5 * 0.000001875 * pow(secs - docking_time, 4)) + (4 * -0.00009375000000000002 * pow(secs - docking_time, 3)) + (3 * 0.00125  * pow(secs - docking_time, 2));

						qd[0] = q1_init + (q1_fin_docking - q1_init) * s;
						qd_dot[0] = (q1_fin_docking - q1_init) * s_dot;

						qd[1] = q2_init + (q2_fin_docking - q2_init) * s;
						qd_dot[1] = (q2_fin_docking - q2_init) * s_dot;

						qd[2] = q3_init + (q3_fin_docking - q3_init) * s;
						qd_dot[2] = (q3_fin_docking - q3_init) * s_dot;

						error_qdot[0] = qd_dot[0] - ls_velocity;
						error_qdot[1] = qd_dot[1] - le_velocity;
						error_qdot[2] = qd_dot[2] - re_velocity;

						errorq[0] = qd[0] - ls_position;
						errorq[1] = qd[1] - le_position;
						errorq[2] = qd[2] - re_position;

						// torq[0] = - (3.0*Kp * errorq[0] + 3.0*Kd * error_qdot[0]);
						// torq[1] = 2.0*Kp * errorq[1] + 2.0*Kd * error_qdot[1];
						// torq[2] = - (4.0*Kp * errorq[2] + 4.0*Kd * error_qdot[2]);

						// torq[0] = - (152.1982533* errorq[0] +  2.152083302* error_qdot[0])/186;
						// torq[1] = (271.5 * errorq[1] + 7.679437641 * error_qdot[1])/186;
						// torq[2] = - (254.5740711*errorq[2] + 35.99677365 * error_qdot[2])/186;

						torq[0] = - (40.42645738* errorq[0] +  1.647501073* error_qdot[0])/186;
						torq[1] = (40.54481 * errorq[1] + 1.63743613 * error_qdot[1])/186;
						torq[2] = - (30.5740711*errorq[2] + 1.99677365 * error_qdot[2])/186;

					} else {
						if (first_time_penetrating) {
							penetration_time = secs;
							q1_init = ls_position;
							q2_init = le_position;
							q3_init = re_position;
							first_time_penetrating = false;
						}
						if ((secs - penetration_time) <= 10.0) {
							s = (0.00006 * pow(secs - penetration_time, 5)) + (-0.0015 * pow(secs - penetration_time, 4)) + (0.01 * pow(secs - penetration_time, 3));
							s_dot = (5 * 0.00006 * pow(secs - penetration_time, 4)) + (4 * -0.0015 * pow(secs - penetration_time, 3)) + (3 * 0.01 * pow(secs - penetration_time, 2));

							qd[0] = q1_init + (q1_fin_pen - q1_init) * s;
							qd_dot[0] = (q1_fin_pen - q1_init) * s_dot;

							qd[1] = q2_init + (q2_fin_pen - q2_init) * s;
							qd_dot[1] = (q2_fin_pen - q2_init) * s_dot;

							qd[2] = q3_init + (q3_fin_pen - q3_init) * s;
							qd_dot[2] = (q3_fin_pen - q3_init) * s_dot;

							error_qdot[0] = qd_dot[0] - ls_velocity;
							error_qdot[1] = qd_dot[1] - le_velocity;
							error_qdot[2] = qd_dot[2] - re_velocity;

							errorq[0] = qd[0] - ls_position;
							errorq[1] = qd[1] - le_position;
							errorq[2] = qd[2] - re_position;

							// torq[0] = - (3.0*Kp * errorq[0] + 3.0*Kd * error_qdot[0]);
							// torq[1] = 2.0*Kp * errorq[1] + 2.0*Kd * error_qdot[1];
							// torq[2] = - (4.0*Kp * errorq[2] + 4.0*Kd * error_qdot[2]);

							torq[0] = - (38.42645738* errorq[0] +  1.647501073* error_qdot[0])/186;
							torq[1] = (45.54481 * errorq[1] + 1.63743613 * error_qdot[1])/186;
							torq[2] = - (30.5740711*errorq[2] + 1.99677365 * error_qdot[2])/186;

							ROS_INFO("ls(-0.577): %lf	    le(1.89): %lf       re(0.40): %lf", ls_position, le_position, re_position);


						} else {
							if (first_time_after_docking) {
								Kp = 0.06;
								Kd = 0.0006;
								errorq[0] = 0.0;
								errorq[1] = 0.0;
								errorq[2] = 0.0;
								first_time_after_docking = false;
							}

							qd_dot[0] = 0.0001;
							qd_dot[1] = 0.0001;
							qd_dot[2] = 0.0001;

							error_qdot[0] = qd_dot[0] - ls_velocity;
							error_qdot[1] = qd_dot[1] - le_velocity;
							error_qdot[2] = qd_dot[2] - re_velocity;

							errorq[0] = error_qdot[0] * (secs - prev_secs) + errorq[0];
							errorq[1] = error_qdot[1] * (secs - prev_secs) + errorq[1];
							errorq[2] = error_qdot[2] * (secs - prev_secs) + errorq[2];

							torq[0] = - (Kp * errorq[0] + Kd * error_qdot[0]);
							torq[1] = Kp * errorq[1] + Kd * error_qdot[1];
							torq[2] = - (Kp * errorq[2] + Kd * error_qdot[2]);

							ROS_INFO("ls(-0.577): %lf	    le(1.89): %lf       re(0.40): %lf", ls_position, le_position, re_position);

						}
					}
				}
			}
		}

		prev_secs = secs;
		
		_secs.data = secs;
		secs_pub.publish(_secs);
		torque.data = filter_torque(torq[0], prev_torq[0]);
		torque.data = torq[0];
		ls_torque_pub.publish(torque);
		torque.data = filter_torque(torq[1], prev_torq[1]);
		torque.data = torq[1];
		le_torque_pub.publish(torque);
		torque.data = filter_torque(torq[2], prev_torq[2]);
		torque.data = torq[2];
		re_torque_pub.publish(torque);

		temp_msg.data = qd[0];
		ls_qd_pub.publish(temp_msg);
		temp_msg.data = qd[1];
		le_qd_pub.publish(temp_msg);
		temp_msg.data = qd[2];
		re_qd_pub.publish(temp_msg);

		temp_msg.data = ls_position;
		ls_pos_pub.publish(temp_msg);
		temp_msg.data = le_position;
		le_pos_pub.publish(temp_msg);
		temp_msg.data = re_position;
		re_pos_pub.publish(temp_msg);
	
		temp_msg.data = ls_velocity;
		ls_vel_pub.publish(temp_msg);
		temp_msg.data = le_velocity;
		le_vel_pub.publish(temp_msg);
		temp_msg.data = re_velocity;
		re_vel_pub.publish(temp_msg);
	
		temp_msg.data = errorq[0];
		ls_error_pub.publish(temp_msg);
		temp_msg.data = errorq[1];
		le_error_pub.publish(temp_msg);
		temp_msg.data = errorq[2];
		re_error_pub.publish(temp_msg);

		// temp_msg.data = ps_x[ALX_GRIPPER];
		temp_msg.data = gripper_x;
		gripper_x_pub.publish(temp_msg);
		// temp_msg.data = ps_y[ALX_GRIPPER];
		temp_msg.data = gripper_y;
		gripper_y_pub.publish(temp_msg);
		// temp_msg.data = ps_th[ALX_GRIPPER];
		temp_msg.data = gripper_th;
		gripper_th_pub.publish(temp_msg);
		temp_msg.data = ps_x[CEPHEUS];
		cepheus_x_pub.publish(temp_msg);
		temp_msg.data = ps_y[CEPHEUS];
		cepheus_y_pub.publish(temp_msg);
		temp_msg.data = ps_th[CEPHEUS];
		cepheus_th_pub.publish(temp_msg);

		temp_msg.data = gripper_x_dot;
		gripper_x_dot_pub.publish(temp_msg);
		temp_msg.data = gripper_y_dot;
		gripper_y_dot_pub.publish(temp_msg);
		temp_msg.data = gripper_th_dot;
		gripper_th_dot_pub.publish(temp_msg);

		temp_msg.data = gripper_error_x;
		gripper_error_x_pub.publish(temp_msg);
		temp_msg.data = gripper_error_y;
		gripper_error_y_pub.publish(temp_msg);
		temp_msg.data = gripper_error_th;
		gripper_error_th_pub.publish(temp_msg);

		temp_msg.data = gripper_x_desdot;
		gripper_x_desdot_pub.publish(temp_msg);

		temp_msg.data = determinant;
		det_pub.publish(temp_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}




////////////////////move 10 cm

// a = 0.143318141419581;
// b = 0.331623152993806;
// c = 0.257342875265441;
// d = 0.257361884747802;
// d00 = a00;
// d10 = a01 * cos(qd[0]);
// d20 = a02 * cos(qd[0] + qd[1]);
// d30 = a03 * cos(qd[0] + qd[1] + qd[2]);
// d01 = d10;
// d11 = a11;
// d21 = a21 * cos(qd[1]);
// d31 = a31 * cos(qd[1] + qd[2]);
// d02 = d20;
// d12 = d21;
// d22 = a22;
// d32 = a32 * cos(qd[2]);
// d03 = d30;
// d13 = d31;
// d23 = d32;
// d33 = a33;
// d0 = d00 + d10 + d20 + d30;
// d1 = d01 + d11 + d21 + d31;
// d2 = d02 + d12 + d22 + d32;
// d3 = d03 + d13 + d23 + d33;

// S = a*b*d2*sin(qd[0]) + b*c*d0*sin(qd[1]) - a*c*d1*sin(qd[0]+qd[1]);
// s_dot = (5 * 0.00006 * pow((secs - move_time), 4)) + (4 * -0.0015 * pow((secs - move_time), 3)) + (3 * 0.01 * pow((secs - move_time), 2));
// xe_desdot = - 0.1 * s_dot;

// theta0_desdot = ((b*d2*cos(theta0_des+qd[0])-c*d1*cos(theta0_des+qd[0]+qd[1]))*xe_desdot)/S;
// qd_dot[0] = ((-d2*(a*cos(theta0_des)+b*cos(theta0_des+qd[0]))+c*(d0+d1)*cos(theta0_des+qd[0]+qd[1]))*xe_desdot)/S; 
// qd_dot[1] = ((a*(d1+d2)*cos(theta0_des)-d0*(b*cos(theta0_des+qd[0])+c*cos(theta0_des+qd[0]+qd[1])))*xe_desdot)/S;
// qd_dot[2] = ((-a*d1*cos(theta0_des)+b*d0*cos(theta0_des+qd[0]))*xe_desdot)/S;

// qd[0] = qd_dot[0] * (secs - prev_secs) + qd[0];
// qd[1] = qd_dot[1] * (secs - prev_secs) + qd[1];
// qd[2] = qd_dot[2] * (secs - prev_secs) + qd[2];
// theta0_des = theta0_desdot * (secs - prev_secs) + theta0_des;
