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

using namespace Eigen;

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

double ls_position = 0.0, le_position = 0.0, re_position = 0.0, rw_position = 0.0;
double ls_velocity = 0.0, le_velocity = 0.0, re_velocity = 0.0, rw_velocity = 0.0;
bool first_time_movement = true;
bool start_docking = false;
bool first_time_before_docking = true;
bool first_time_docking = true;
bool first_time_penetrating = true;
bool first_time_after_docking = true;

double ps_x[3];
double ps_x_prev[3];
double ps_y[3];
double ps_y_prev[3];
double ps_th[3];
double ps_th_prev[3];

double q1_init = -60 * (M_PI / 180);
double q2_init = 105 * (M_PI / 180);
double q3_init = 45 * (M_PI / 180);

double Kp = 0.06;
double Kd = 0.006;

double force_x;
double force_y;
double force_z;
double torque_x;
double torque_y;
double torque_z;


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
	ps_x_prev[ALX_GRIPPER] = ps_x[ALX_GRIPPER];
	ps_y_prev[ALX_GRIPPER] = ps_y[ALX_GRIPPER];
	ps_th_prev[ALX_GRIPPER] = ps_th[ALX_GRIPPER];
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
	// ps_x_prev[ASSIST] = ps_x[ASSIST];
	// ps_y_prev[ASSIST] = ps_y[ASSIST];
	// ps_th_prev[ASSIST] = ps_th[ASSIST];
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
	ps_x_prev[CEPHEUS] = ps_x[CEPHEUS];
	ps_y_prev[CEPHEUS] = ps_y[CEPHEUS];
	ps_th_prev[CEPHEUS] = ps_th[CEPHEUS];
	ps_x[CEPHEUS] = f_x3->filter(temp.transform.translation.x);
	ps_y[CEPHEUS] = f_y3->filter(temp.transform.translation.y);
	ps_th[CEPHEUS] = f_z3->filter(yaw);
	return;
}


void botasysCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
	geometry_msgs::WrenchStamped temp;
	temp = *msg;
	force_x = temp.wrench.force.x;
	force_y = temp.wrench.force.y;
	force_z = temp.wrench.force.z;
	torque_x = temp.wrench.torque.x;
	torque_y = temp.wrench.torque.y;
	torque_z = temp.wrench.torque.z;
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


void rwPosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - rw_position) > POS_FILTER)
		return;
	else
		rw_position = cmd->data;
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


void rwVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - rw_velocity) > VEL_FILTER)
		return;
	else
		rw_velocity = cmd->data;
}


void startDockingCallback(const std_msgs::Bool::ConstPtr& msg) {
	start_docking = true;
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


ros::Publisher reset_movement_pub;
sig_atomic_t volatile g_request_shutdown = 0;
std_msgs::Bool reset_movement;
// Replacement SIGINT handler
void ctrl_c_handler(int sig) {
    reset_movement.data = true;
    reset_movement_pub.publish(reset_movement);
	g_request_shutdown = 1;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "erros_control_node", ros::init_options::NoSigintHandler);
	signal(SIGINT, ctrl_c_handler);
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
	// ros::Subscriber ps_cepheus_sub =  nh.subscribe("map_to_cepheus", 1, PSCepheusCallback);

	ros::Publisher start_moving_pub = nh.advertise<std_msgs::Bool>("start_moving", 1);
    reset_movement_pub = nh.advertise<std_msgs::Bool>("reset_movement", 1);
	// ros::Subscriber phase_space_sub =  nh.subscribe("joint_states", 1, statesCallback);
	ros::Publisher rw_torque_pub = nh.advertise<std_msgs::Float64>("set_reaction_wheel_effort", 1);
	ros::Publisher ls_torque_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_effort", 1);
	ros::Publisher le_torque_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_effort", 1);
	ros::Publisher re_torque_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_effort", 1);
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
	ros::Subscriber rw_pos_sub = nh.subscribe("read_reaction_wheel_position", 1, rwPosCallback);
	ros::Subscriber ls_vel_sub = nh.subscribe("read_left_shoulder_velocity", 1, lsVelCallback);
	ros::Subscriber le_vel_sub = nh.subscribe("read_left_elbow_velocity", 1, leVelCallback);
	ros::Subscriber re_vel_sub = nh.subscribe("read_right_elbow_velocity", 1, reVelCallback);
	ros::Subscriber rw_vel_sub = nh.subscribe("read_reaction_wheel_velocity", 1, rwVelCallback);

	ros::Subscriber start_docking_sub = nh.subscribe("start_docking", 1, startDockingCallback);
	ros::Subscriber reset_docking_sub = nh.subscribe("reset_docking", 1, resetDockingCallback);

	ros::Subscriber botasys_sub = nh.subscribe("filtered_botasys", 1, botasysCallback);

	ros::Subscriber set_gains_sub = nh.subscribe("set_gains", 1, setGainsCallback);

	double rate;
	ros::param::param<double>("~loop_rate", rate, 200);
	ros::Rate loop_rate(rate);

	double errorq[4];
	double error_qdot[4];
	double torq[4];
	double prev_torq[4];
	double qd[4];
	double qd_dot[4];

	for (int i = 0; i < 4; i++) {
		errorq[i] = 0.0;
		error_qdot[i] = 0.0;
		torq[i] = 0.0;
		prev_torq[i] = 0.0;
		qd[i] = 0.0;
		qd_dot[i] = 0.0;
	}

	double theta0_des = 90 * (M_PI / 180);
	double theta0dot_des = 0.0;

	ros::Time curr_time, t_beg = ros::Time::now();
	ros::Duration all_time;
	double prev_secs = 0.0;

    std_msgs::Bool start_moving;
    start_moving.data = true;
	std_msgs::Float64 torque;
	std_msgs::Float64 offset;
	std_msgs::Float64 temp_msg;
	std_msgs::Float64 _secs;
	std_msgs::UInt16 lar;

	double s, sdot, sdotdot;
	double secs, time_step, move_time, docking_time, penetration_time, prin;
	double xE_in, yE_in, thE_in, th0_in, xE_fin, yE_fin, thE_fin, th0_fin;
	double s_0, s_f, s0dot, sfdot, s0dotdot, sfdotdot;
	double K, L;
	double t0 = 0.0, tf = 20.0, time = 0.0;
	double a0, a1, a2, a3, a4, a5;
	double th0_des, xE, yE, thE, th0dot_des, xEdot, yEdot, thEdot, th0dotdot_des, xEdotdot, yEdotdot, thEdotdot;
	double m0 = 400;
	double m1 = 100;
	double m2 = 100;
	double m3 = 50;
	double M = m0 + m1 + m2 + m3;
	double r0x = 0.5;
	double r0y = 0.0;
	double r1 = 1;
	double l1 = 1;
	double r2 = 0.5;
	double l2 = 0.5;
	double r3 = 0.5;
	double l3 = 0.5;
	double I0z = 100;
	double I1z = 50;
	double I2z = 50;
	double I3z = 10;
	double q01 = 0;

	double p1 = M;
	double p2 = (m1+m2+m3)*r0x;
	double p3 = (m1+m2+m3)*r0y;
	double p4 = (m1+m2+m3)*l1+(m2+m3)*r1;
	double p5 = (m2+m3)*l2+m3*r2;
	double p6 = I0z+(m1+m2+m3)*(pow(r0x,2)+pow(r0y,2));
	double p7 = I1z+(m1+m2+m3)*pow(l1,2)+2*(m2+m3)*l1*r1+(m2+m3)*pow(r1,2);
	double p8 = I2z+(m2+m3)*pow(l2,2)+2*m3*l2*r2+m3*pow(r2,2);
	double p9 = I3z+m3*pow(l3,2);
	double p10 = ((m1+m2+m3)*l1+(m2+m3)*r1)*r0x;
	double p11 = ((m1+m2+m3)*l1+(m2+m3)*r1)*r0y;
	double p12 = (l1+r1)*((m2+m3)*l2+m3*r2);
	double p13 = ((m2+m3)*l2+m3*r2)*r0x;
	double p14 = ((m2+m3)*l2+m3*r2)*r0y;
	double p15 = m3*l3;
	double p16 = m3*l3*r0x;
	double p17 = m3*l3*r0y;
	double p18 = (l1+r1)*m3*l3;
	double p19 = (l2+r2)*m3*l3;

	double theta0, theta1, theta2, theta3;
	double theta0Dot, theta1Dot, theta2Dot, theta3Dot;
	double th0, q1, q2, q3;
	double th0dot, q1dot, q2dot, q3dot;
	double thetaEdot, thetaE, thetaE_des, thetaEdot_des, thetaEdotdot_des;
	MatrixXd eye_3;
	eye_3.setIdentity(3, 3);
	MatrixXd eye_2;
	eye_2.setIdentity(2, 2);
	Matrix3d KD = eye_3 * 2;
	Matrix3d KP = eye_3 * 4;
	Vector3d qE;
	Vector3d qEDot;
	Vector3d ve;
	Vector3d vedot;
	Matrix2d HPS = eye_2;
	Matrix2d DPS = eye_2 * 2;
	Matrix2d KPS = eye_2 * 4;
	Matrix3d HA = eye_3;
	Matrix3d DA = eye_3 * 2;
	Matrix3d KA = eye_3 * 4;
	Vector3d ve_des;
	Vector3d vedot_des;
	MatrixXd G(6, 6);
	VectorXd B1(6);
	VectorXd x1(6);
	MatrixXd inv(6, 6);

	while (!g_request_shutdown) {
        // order initialize_arm_node to release arm
        start_moving_pub.publish(start_moving);

		for (int i = 0; i < 3; i++) {
			prev_torq[i] = torq[i];
		}

		curr_time = ros::Time::now();
		all_time = curr_time - t_beg;

		secs = all_time.sec + all_time.nsec * pow(10, -9);
		time_step = secs - prev_secs;
		if (first_time_movement) {
			move_time = secs;
			// xE_in = ps_x[ALX_GRIPPER];
			// yE_in = ps_y[ALX_GRIPPER];
			// thE_in = ps_th[ALX_GRIPPER];
			// th0_in = ps_th[CEPHEUS];
			// xE_fin = ps_x[ASSIST];
			// yE_fin = ps_y[ASSIST];
			// thE_fin = ps_th[ASSIST];
			// th0_fin = th0_in;
			ps_x_prev[ALX_GRIPPER] = ps_x[ALX_GRIPPER];
			ps_y_prev[ALX_GRIPPER] = ps_y[ALX_GRIPPER];
			ps_th_prev[ALX_GRIPPER] = ps_th[ALX_GRIPPER];
			xE_in=-1.975;
			yE_in=3.409;
			thE_in = 170*(M_PI/180);
			th0_in = 90*(M_PI/180);
			xE_fin = -2.1;
			yE_fin = 3.5;
			thE_fin=150*(M_PI/180);
			th0_fin=90*(M_PI/180);

			K = (yE_fin-yE_in)/(xE_fin-xE_in);
			L = (yE_in*(xE_fin-xE_in)-xE_in*(yE_fin-yE_in))/(xE_fin-xE_in);

			s_0 = 0.0;
			s_f = 1.0;
			s0dot = 0.0;
			sfdot = 0.0;
			s0dotdot = 0.0;
			sfdotdot = 0.0;

			G << 1, t0, pow(t0,2), pow(t0,3), pow(t0,4), pow(t0,5),
				1, tf, pow(tf,2), pow(tf,3), pow(tf,4), pow(tf,5),
				0, 1, 2*t0, 3*pow(t0,2), 4*pow(t0,3), 5*pow(t0,4),
				0, 1, 2*tf, 3*pow(tf,2), 4*pow(tf,3), 5*pow(tf,4),
				0, 0, 2, 6*t0, 12*pow(t0,2), 20*pow(t0,3),
				0, 0, 2, 6*tf, 12*pow(tf,2), 20*pow(tf,3);

			B1 << s_0, 
				s_f, 
				s0dot, 
				sfdot, 
				s0dotdot, 
				sfdotdot;

			// std::cout << "Here is the matrix G:\n" << G << std::endl;
			inv = G.inverse();
			// std::cout << "Here is the inv matrix G:\n" << inv << std::endl;
			x1 = inv*B1;

			a0 = x1(0);
			a1 = x1(1);
			a2 = x1(2);
			a3 = x1(3);
			a4 = x1(4);
			a5 = x1(5);
			first_time_movement = false;
		}

		time = secs - move_time;
        if (time <= tf) {
			// s = a5*pow(time,5)+a4*pow(time,4)+a3*pow(time,3)+a2*pow(time,2)+a1*time+a0;
			// sdot = 5*a5*pow(time,4)+4*a4*pow(time,3)+3*a3*pow(time,2)+2*a2*time+a1;
			// sdotdot = 20*a5*pow(time,3)+12*a4*pow(time,2)+6*a3*time+2*a2;

			// th0_des = th0_in+s*(th0_fin-th0_in);
			// xE = xE_in+s*(xE_fin-xE_in);
			// yE = K*xE+L;
			// thE = thE_in+s*(thE_fin-thE_in);
			// th0dot_des = sdot*(th0_fin-th0_in);
			// xEdot = sdot*(xE_fin-xE_in);
			// yEdot = K*xEdot;
			// thEdot = sdot*(thE_fin-thE_in);
			// th0dotdot_des = sdotdot*(th0_fin-th0_in);
			// xEdotdot = sdotdot*(xE_fin-xE_in);
			// yEdotdot = K*xEdotdot;
			// thEdotdot = sdotdot*(thE_fin-thE_in);

			// Vector3d qE_des(xE, yE, thE);
			// // ROS_INFO("qE_des      : %f %f %f %f", time, xE, yE, thE);
			// Vector3d qEdot_des(xEdot, yEdot, thEdot);
			// // ROS_INFO("qEdot_des   : %f %f %f %f", time, xEdot, yEdot, thEdot);
			// Vector3d qEdotdot_des(xEdotdot, yEdotdot, thEdotdot);
			// // ROS_INFO("qEdotdot_des: %f %f %f %f", time, xEdotdot, yEdotdot, thEdotdot);

			// qE << ps_x[ALX_GRIPPER], ps_y[ALX_GRIPPER], ps_th[ALX_GRIPPER];
			// qEDot <<
			// 		(ps_x[ALX_GRIPPER] - ps_x_prev[ALX_GRIPPER]) / time_step,
			// 		(ps_y[ALX_GRIPPER] - ps_y_prev[ALX_GRIPPER]) / time_step,
			// 		(ps_th[ALX_GRIPPER] - ps_th_prev[ALX_GRIPPER]) / time_step;
			// ve = qE;
			// vedot = qEDot;

			// ve_des = qE_des;
			// vedot_des = qEdot_des;

			// theta0 = ps_th[CEPHEUS];
			// theta1 = ls_position;
			// theta2 = le_position;
			// theta3 = re_position;

			// theta0Dot = (ps_th[CEPHEUS] - ps_th_prev[CEPHEUS]) / time_step;
			// theta1Dot = ls_velocity;
			// theta2Dot = le_velocity;
			// theta3Dot = re_velocity;

			// th0 = theta0;
			// q1 = theta1 + q01;
			// q2 = theta2;
			// q3 = theta3;

			// th0dot = theta0Dot;
			// q1dot = theta1Dot;
			// q2dot = theta2Dot;
			// q3dot = theta3Dot;

			// thetaEdot = qEDot(2);
			// thetaE = qE(2);
			// thetaE_des = qE_des(2);
			// thetaEdot_des = qEdot_des(2);
			// thetaEdotdot_des = qEdotdot_des(3);

			// Vector2d xEdotdot_des;
			// xEdotdot_des << qEdotdot_des(0), qEdotdot_des(1);

			// Vector2d xEdot;
			// xEdot << qEDot(0), qEDot(1);

			// Vector2d xE;
			// xE << qE(0), qE(1);

			// Vector2d xEdot_des;
			// xEdot_des << qEdot_des(0), qEdot_des(1);

			// Vector2d xE_des;
			// xE_des << qE_des(0), qE_des(1);

			// double error_theta = theta0_des - theta0;
			// double errordot_theta = theta0dot_des - theta0Dot;
			// Vector3d error_ve = ve_des - ve;
			// Vector3d errordot_ve = vedot_des - vedot;

			// Vector2d error;
			// error << error_theta, error_ve;

			// Vector2d error_dot;
			// error_dot << errordot_theta, errordot_ve;

			// VectorXd qDot(6);
			// qDot << (ps_x[CEPHEUS] - ps_x_prev[CEPHEUS]) / time_step,
			// 		(ps_y[CEPHEUS] - ps_y_prev[CEPHEUS]) / time_step,
			// 		(ps_th[CEPHEUS] - ps_th_prev[CEPHEUS]) / time_step,
			// 		ls_velocity,
			// 		le_velocity,
			// 		re_velocity;
			VectorXd v1(6);
			// v1 = qDot;

			// for testing matrix results
			th0 = 90 * (M_PI / 180);
			theta0 = 90 * (M_PI / 180);
			q1 = 80 * (M_PI / 180);
			q2 = 70 * (M_PI / 180);
			q3 = 60 * (M_PI / 180);
			th0dot = 50 * (M_PI / 180);
			q1dot = 40 * (M_PI / 180);
			q2dot = 30 * (M_PI / 180);
			q3dot = 20 * (M_PI / 180);
			v1 << 1, 2, 3, 4, 5, 6;
			double theta0dotdot_des=2;
			double theta0Dot=2;
			double theta0dot_des = 90 * (M_PI / 180);
			thetaEdot = 30 * (M_PI / 180);
			thetaE = 30 * (M_PI / 180);
			thetaE_des = 30 * (M_PI / 180);
			thetaEdot_des = 30 * (M_PI / 180);
			thetaEdotdot_des = 30 * (M_PI / 180);
			double FextX = 0.0;
			// for production
			// FextX = force_z;
			double FextY = 0.0;
			// FextY = force_x;
			double Next = 0.0;
			// Next = torque_y;
			double xEdotdot_des = 10.0;
			double xEdot_des = 10.0;
			double xE_des = 10.0;
			double xEdot = 10.0;
			double xE = 10.0;

			// std::cout << "Forces: " << force_x << force_y << force_z << " Torques: " << torque_x << torque_y << torque_z << std::endl;

			MatrixXd H(6, 6);
			H << p1,
				0,
				(-1)*p3*cos(th0) + (-1)*p2*sin(th0) + (-1)*p4*sin(q1 + th0) + (-1)*p5*sin(q1 + q2 + th0) + (-1)*p15*sin(q1 + q2 + q3 + th0),
				(-1)*p4*sin(q1 + th0) + (-1)*p5*sin(q1 + q2 + th0) + (-1)*p15*sin(q1 + q2 + q3 + th0),
				(-1)*p5*sin(q1 + q2 + th0) + (-1)*p15*sin(q1 + q2 + q3 + th0),
				(-1)*p15*sin(q1 + q2 + q3 + th0), //end of row 1
				
				0,
				p1,
				p2*cos(th0) + p4*cos(q1 + th0) + p5*cos(q1 + q2 + th0) + p15*cos(q1 + q2 + q3 + th0) + (-1)*p3*sin(th0),
				p4*cos(q1 + th0) + p5*cos(q1 + q2 + th0) + p15*cos(q1 + q2 + q3 + th0),
				p5*cos(q1 + q2 + th0) + p15 * cos(q1 + q2 + q3 + th0),
				p15*cos(q1 + q2 + q3 + th0), // end of row 2
				
				(-1)*p3*cos(th0) + (-1)* p2*sin(th0) + (-1)*p4*sin(q1 + th0) + (-1)*p5*sin(q1 + q2 + th0) + (-1)* p15*sin(q1 + q2 + q3 + th0),
				p2*cos(th0) + p4*cos(q1 + th0) + p5*cos(q1 + q2 + th0) + p15*cos(q1 + q2 + q3 + th0) + (-1)*p3*sin(th0),
				p6 + p7 + p8 + p9 + 2* p10*cos(q1) + 2*p12*cos(q2) + 2*p13*cos(q1 + q2) + 2*p19*cos(q3) + 2*p18*cos(q2 + q3) + 2*p16*cos(q1 + q2 + q3) + 2*p11*sin(q1) + 2*p14*sin(q1 + q2) + 2*p17*sin(q1 + q2 + q3),
				p7 + p8 + p9 + p10*cos(q1) + 2*p12* cos(q2) + p13*cos(q1 + q2) + 2*p19*cos(q3) + 2*p18*cos(q2 + q3) + p16*cos(q1 + q2 + q3) + p11*sin(q1) + p14*sin(q1 + q2) + p17*sin(q1 + q2 + q3),
				p8 + p9 + p12*cos(q2) + p13*cos(q1 + q2) + 2*p19*cos(q3) + p18*cos(q2 + q3) + p16*cos(q1 + q2 + q3) + p14*sin(q1 + q2) + p17*sin(q1 + q2 + q3),
				p9 + p19*cos(q3) + p18*cos(q2 + q3) + p16*cos(q1 + q2 + q3) + p17*sin(q1 + q2 + q3), // end of row 3

				(-1)* p4*sin(q1 + th0) + (-1)*p5*sin(q1 + q2 + th0) + (-1)*p15*sin(q1 + q2 + q3 +  th0),
				p4*cos(q1 + th0) + p5*cos(q1 + q2 + th0) + p15*cos(q1 + q2 + q3 + th0),
				p7 +  p8 + p9 + p10*cos(q1) + 2*p12*cos(q2) + p13*cos(q1 + q2) + 2*p19*cos(q3)+ 2*p18*cos(q2 + q3) + p16*cos(q1 + q2 + q3) + p11*sin(q1) + p14*sin(q1 + q2) + p17*sin(q1 + q2 + q3),
				p7 + p8 + p9 + 2*p12*cos(q2) + 2*p19*cos(q3) +  2*p18*cos(q2 + q3),
				p8 + p9 + p12*cos(q2) + 2*p19*cos(q3) + p18*cos(q2 + q3),
				p9 + p19*cos(q3) + p18*cos(q2 + q3), // end of row 4

				(-1)*p5*sin(q1 + q2 + th0) + (-1)*p15*sin(q1 + q2 + q3 + th0),
				p5*cos(q1 + q2 + th0) + p15*cos(q1 + q2 + q3 + th0) ,
				p8 + p9 + p12*cos(q2) + p13*cos(q1 + q2) + 2*p19*cos(q3) + p18*cos(q2 +q3) + p16*cos(q1 + q2 + q3) + p14*sin(q1 + q2) + p17*sin(q1 + q2 + q3),
				p8 + p9 + p12*cos(q2) + 2*p19*cos(q3) + p18*cos(q2 + q3),
				p8 + p9 + 2*p19*cos(q3),
				p9 + p19*cos(q3), // end of row 5

				(-1)*p15*sin(q1 + q2 + q3 + th0),
				p15*cos(q1 + q2 + q3 + th0),
				p9 + p19*cos(q3) + p18*cos(q2 + q3) + p16*cos(q1 + q2 + q3) + p17*sin( q1 + q2 + q3),
				p9 + p19*cos(q3) + p18*cos(q2 + q3),
				p9 + p19*cos(q3),
				p9;
			
			// std::cout << "Here is the matrix H:\n" << H << std::endl;

			VectorXd c(6);
			c << (-1)*p2*pow(th0dot,2)*cos(th0) + (-1)*p4*pow((q1dot + th0dot),2)*cos( 
				q1 + th0) + (-1)*p5*pow(q1dot,2)*cos(q1 + q2 + th0) + (-2)*p5*q1dot* 
				q2dot*cos(q1 + q2 + th0) + (-1)*p5*pow(q2dot,2)*cos(q1 + q2 + th0) + (-2)* 
				p5*q1dot*th0dot*cos(q1 + q2 + th0) + (-2)*p5*q2dot*th0dot*cos(q1 +  
				q2 + th0) + (-1)*p5*pow(th0dot,2)*cos(q1 + q2 + th0) + (-1)*p15*pow(q1dot,2)* 
				cos(q1 + q2 + q3 + th0) + (-2)*p15*q1dot*q2dot*cos(q1 + q2 + q3 + th0) + (-1) 
				*p15*pow(q2dot,2)*cos(q1 + q2 + q3 + th0) + (-2)*p15*q1dot*q3dot*cos( 
				q1 + q2 + q3 + th0) + (-2)*p15*q2dot*q3dot*cos(q1 + q2 + q3 + th0) + (-1)* 
				p15*pow(q3dot,2)*cos(q1 + q2 + q3 + th0) + (-2)*p15*q1dot*th0dot*cos(q1 +  
				q2 + q3 + th0) + (-2)*p15*q2dot*th0dot*cos(q1 + q2 + q3 + th0) + (-2)*p15* 
				q3dot*th0dot*cos(q1 + q2 + q3 + th0) + (-1)*p15*pow(th0dot,2)*cos(q1 + q2 +  
				q3 + th0) + p3*pow(th0dot,2)*sin(th0),
				(-1)*p3*pow(th0dot,2)*cos(th0) + (-1) 
				*p2*pow(th0dot,2)*sin(th0) + (-1)*p4*pow(q1dot,2)*sin(q1 + th0) + (-2)* 
				p4*q1dot*th0dot*sin(q1 + th0) + (-1)*p4*pow(th0dot,2)*sin(q1 + th0) + ( 
				-1)*p5*pow(q1dot,2)*sin(q1 + q2 + th0) + (-2)*p5*q1dot*q2dot*sin(q1 +  
				q2 + th0) + (-1)*p5*pow(q2dot,2)*sin(q1 + q2 + th0) + (-2)*p5*q1dot* 
				th0dot*sin(q1 + q2 + th0) + (-2)*p5*q2dot*th0dot*sin(q1 + q2 + th0) + ( 
				-1)*p5*pow(th0dot,2)*sin(q1 + q2 + th0) + (-1)*p15*pow(q1dot,2)*sin(q1 + q2 +  
				q3 + th0) + (-2)*p15*q1dot*q2dot*sin(q1 + q2 + q3 + th0) + (-1)*p15* pow(q2dot,2)*sin(q1 + q2 + q3 + th0) + (-2)*p15*q1dot*q3dot*sin(q1 + q2 + q3 +  
				th0) + (-2)*p15*q2dot*q3dot*sin(q1 + q2 + q3 + th0) + (-1)*p15* 
				pow(q3dot,2)*sin(q1 + q2 + q3 + th0) + (-2)*p15*q1dot*th0dot*sin(q1 + q2 +  
				q3 + th0) + (-2)*p15*q2dot*th0dot*sin(q1 + q2 + q3 + th0) + (-2)*p15* 
				q3dot*th0dot*sin(q1 + q2 + q3 + th0) + (-1)*p15*pow(th0dot,2)*sin(q1 + q2 +  
				q3 + th0),
				p11*q1dot*(q1dot + 2*th0dot)*cos(q1) + p14*(q1dot + q2dot) 
				*(q1dot + q2dot + 2*th0dot)*cos(q1 + q2) + p17*pow(q1dot,2)*cos(q1 + q2 + q3) 
				+ 2*p17*q1dot*q2dot*cos(q1 + q2 + q3) + p17*pow(q2dot,2)*cos(q1 + q2 + q3) +  
				2*p17*q1dot*q3dot*cos(q1 + q2 + q3) + 2*p17*q2dot*q3dot*cos(q1 +  
				q2 + q3) + p17*pow(q3dot,2)*cos(q1 + q2 + q3) + 2*p17*q1dot*th0dot*cos(q1 +  
				q2 + q3) + 2*p17*q2dot*th0dot*cos(q1 + q2 + q3) + 2*p17*q3dot* 
				th0dot*cos(q1 + q2 + q3) + (-1)*p10*pow(q1dot,2)*sin(q1) + (-2)*p10* 
				q1dot*th0dot*sin(q1) + (-2)*p12*q1dot*q2dot*sin(q2) + (-1)* 
				p12*pow(q2dot,2)*sin(q2) + (-2)*p12*q2dot*th0dot*sin(q2) + (-1)* 
				p13*pow(q1dot,2)*sin(q1 + q2) + (-2)*p13*q1dot*q2dot*sin(q1 + q2) + (-1) 
				*p13*pow(q2dot,2)*sin(q1 + q2) + (-2)*p13*q1dot*th0dot*sin(q1 + q2) + ( 
				-2)*p13*q2dot*th0dot*sin(q1 + q2) + (-2)*p19*q1dot*q3dot*sin( 
				q3) + (-2)*p19*q2dot*q3dot*sin(q3) + (-1)*p19*pow(q3dot,2)*sin(q3) +  
				(-2)*p19*q3dot*th0dot*sin(q3) + (-2)*p18*q1dot*q2dot*sin(q2 +  
				q3) + (-1)*p18*pow(q2dot,2)*sin(q2 + q3) + (-2)*p18*q1dot*q3dot*sin( 
				q2 + q3) + (-2)*p18*q2dot*q3dot*sin(q2 + q3) + (-1)*p18*pow(q3dot,2)* 
				sin(q2 + q3) + (-2)*p18*q2dot*th0dot*sin(q2 + q3) + (-2)*p18*q3dot* 
				th0dot*sin(q2 + q3) + (-1)*p16*pow(q1dot,2)*sin(q1 + q2 + q3) + (-2)*p16* 
				q1dot*q2dot*sin(q1 + q2 + q3) + (-1)*p16*pow(q2dot,2)*sin(q1 + q2 + q3) + ( 
				-2)*p16*q1dot*q3dot*sin(q1 + q2 + q3) + (-2)*p16*q2dot*q3dot* 
				sin(q1 + q2 + q3) + (-1)*p16*pow(q3dot,2)*sin(q1 + q2 + q3) + (-2)*p16* 
				q1dot*th0dot*sin(q1 + q2 + q3) + (-2)*p16*q2dot*th0dot*sin(q1 + q2 +  
				q3) + (-2)*p16*q3dot*th0dot*sin(q1 + q2 + q3),
				(-1)*p11*pow(th0dot,2)* 
				cos(q1) + (-1)*p14*pow(th0dot,2)*cos(q1 + q2) + (-1)*p17*pow(th0dot,2)* 
				cos(q1 + q2 + q3) + p10*pow(th0dot,2)*sin(q1) + (-2)*p12*q1dot*q2dot* 
				sin(q2) + (-1)*p12*pow(q2dot,2)*sin(q2) + (-2)*p12*q2dot*th0dot* 
				sin(q2) + p13*pow(th0dot,2)*sin(q1 + q2) + (-2)*p19*q1dot*q3dot*sin( 
				q3) + (-2)*p19*q2dot*q3dot*sin(q3) + (-1)*p19*pow(q3dot,2)*sin(q3) +  
				(-2)*p19*q3dot*th0dot*sin(q3) + (-2)*p18*q1dot*q2dot*sin(q2 +  
				q3) + (-1)*p18*pow(q2dot,2)*sin(q2 + q3) + (-2)*p18*q1dot*q3dot*sin( 
				q2 + q3) + (-2)*p18*q2dot*q3dot*sin(q2 + q3) + (-1)*p18*pow(q3dot,2)* 
				sin(q2 + q3) + (-2)*p18*q2dot*th0dot*sin(q2 + q3) + (-2)*p18*q3dot* 
				th0dot*sin(q2 + q3) + p16*pow(th0dot,2)*sin(q1 + q2 + q3),
				(-1)*p14* pow(th0dot,2)*cos(q1 + q2) + (-1)*p17*pow(th0dot,2)*cos(q1 + q2 + q3) + p12* pow(q1dot,2)*sin(q2) + 2*p12*q1dot*th0dot*sin(q2) + p12*pow(th0dot,2)* 
				sin(q2) + p13*pow(th0dot,2)*sin(q1 + q2) + (-2)*p19*q1dot*q3dot*sin( 
				q3) + (-2)*p19*q2dot*q3dot*sin(q3) + (-1)*p19*pow(q3dot,2)*sin(q3) +  
				(-2)*p19*q3dot*th0dot*sin(q3) + p18*pow(q1dot,2)*sin(q2 + q3) + 2* 
				p18*q1dot*th0dot*sin(q2 + q3) + p18*pow(th0dot,2)*sin(q2 + q3) + p16*pow(th0dot,2)*sin(q1 + q2 + q3),
				(-1)*p17*pow(th0dot,2)*cos(q1 + q2 + q3) + p19* 
				pow((q1dot + q2dot + th0dot),2)*sin(q3) + p18*pow(q1dot,2)*sin(q2 + q3) + 2* 
				p18*q1dot*th0dot*sin(q2 + q3) + p18*pow(th0dot,2)*sin(q2 + q3) + p16* 
				pow(th0dot,2)*sin(q1 + q2 + q3);
			
			// std::cout << "Here is the vector c:\n" << c << std::endl;

			MatrixXd Je(3, 6);
			Je <<  1,
				0,
				(-1)*r0y*cos(th0) + (-1)*r0x*sin(th0) + (-1)*l1*sin(q1 + th0) + (-1)*r1*sin(q1 + th0) + (-1)*l2*sin(q1 + q2 + th0) + (-1)*r2*sin(q1 + q2 + th0) + (-1)*l3*sin(q1 + q2 + q3 + th0) + (-1)*r3*sin(q1 + q2 + q3 + th0),
				(-1)*(l1 + r1)*sin(q1 + th0) + (-1)*(l2 + r2)*sin(q1 + q2 + th0) + (-1)*(l3 + r3)*sin(q1 + q2 + q3 + th0),
				(-1)*(l2 + r2)*sin(q1 + q2 + th0) + (-1)*(l3 + r3) *sin(q1 + q2 + q3 + th0),
				(-1)*(l3 + r3)*sin(q1 + q2 + q3 + th0), //end of row 1

				0,
				1,
				r0x*cos(th0) + l1*cos(q1 + th0) + r1*cos(q1 + th0) + l2*cos(q1 + q2 + th0) + r2*cos(q1 + q2 + th0) + l3*cos(q1 + q2 + q3 + th0) + r3*cos(q1 + q2 + q3 + th0) + (-1)*r0y*sin(th0),
				(l1 + r1)*cos(q1 + th0) + (l2 + r2)*cos(q1 + q2 + th0) + (l3 + r3)*cos(q1 + q2 + q3 + th0),
				(l2 + r2)*cos(q1 + q2 + th0) + (l3 + r3)*cos(q1 + q2 + q3 + th0),
				(l3 + r3)*cos(q1 + q2 + q3 + th0), //end of row 2

				0, 0, 1, 1, 1, 1;
			
			// std::cout << "Here is the matrix Je:\n" << Je << std::endl;

			MatrixXd Jedot(3, 6);
			Jedot << 0,
					0,
					q3dot*((-1)*l3*cos(q1 + q2 + q3 + th0) + (-1)*r3*cos(q1 + q2 + q3 +th0)) + q2dot*((-1)*l2*cos(q1 + q2 + th0) + 
					(-1)*r2*cos(q1 + q2 + th0) + (-1)*l3*cos(q1 + q2 + q3 + th0) + (-1)*r3*cos(q1 + q2 + q3 + th0)) + 
					q1dot*(( -1)*l1*cos(q1 + th0) + (-1)*r1*cos(q1 + th0) + (-1)*l2*cos(q1 + q2 +th0) + (-1)*r2*cos(q1 + q2 + th0) + (-1)*l3*cos(q1 + q2 + q3 + 
					th0) + (-1)*r3*cos(q1 + q2 + q3 + th0)) + th0dot*((-1)*r0x*cos(th0) + (-1)*l1*cos( q1 + th0) + 
					(-1)*r1*cos(q1 + th0) + (-1)*l2*cos(q1 + q2 + th0) + (-1)*r2*cos(q1 + q2 + th0) + (-1)*l3*cos(q1 + q2 + q3 + th0) + 
					(-1)*r3*cos(q1 + q2 + q3 +th0) + r0y*sin(th0)),
					(-1)*q3dot*(l3 + r3)*cos(q1 + q2 + q3 + th0) + 
					q2dot*((-1)*(l2 + r2)*cos(q1 + q2 + th0) + (-1)*(l3 + r3)*cos(q1 + q2 + q3 + th0)) + 
					q1dot*((-1)*(l1 + r1)*cos(q1 + th0) + (-1)*(l2 + r2)*cos(q1 + q2 +  th0) + (-1)*(l3 + r3)*cos(q1 + q2 + q3 + th0)) + 
					th0dot*((-1)*(l1 + r1)* cos(q1 + th0) + (-1)*(l2 + r2)*cos(q1 + q2 + th0) + (-1)*(l3 + r3)*cos(q1 +  q2 + q3 + th0)),
					(-1)*q3dot*(l3 + r3)*cos(q1 + q2 + q3 + th0) + q1dot*((-1)*(l2 + r2)*cos(q1 + q2 + th0) + 
					(-1)*(l3 + r3)*cos(q1 + q2 + q3 + th0)) + q2dot*((-1)*(l2 + r2)*cos(q1 + q2 + th0) + (-1)*(l3 + r3)*cos(q1 + q2 + q3 + th0)) + 
					th0dot*((-1)*(l2 + r2)*cos(q1 + q2 + th0) + (-1)*(l3 + r3)*cos(q1 + q2 +q3 + th0)),
					(-1)*q1dot*(l3 + r3)*cos(q1 + q2 + q3 + th0) + (-1)*q2dot*(l3 +r3)*cos(q1 + q2 + q3 + th0) + 
					(-1)*q3dot*(l3 + r3)*cos(q1 + q2 + q3 + th0) + (-1)*(l3 + r3)*th0dot*cos(q1 + q2 + q3 + th0), //end of row 1
					0,
					0,
					q3dot*((-1)*l3*sin( 
					q1 + q2 + q3 + th0) + (-1)*r3*sin(q1 + q2 + q3 + th0)) + q2dot*((-1)*l2*sin( 
					q1 + q2 + th0) + (-1)*r2*sin(q1 + q2 + th0) + (-1)*l3*sin(q1 + q2 + q3 + th0) + ( 
					-1)*r3*sin(q1 + q2 + q3 + th0)) + q1dot*((-1)*l1*sin(q1 + th0) + (-1)* 
					r1*sin(q1 + th0) + (-1)*l2*sin(q1 + q2 + th0) + (-1)*r2*sin(q1 + q2 + th0) +  
					(-1)*l3*sin(q1 + q2 + q3 + th0) + (-1)*r3*sin(q1 + q2 + q3 + th0)) + th0dot*( 
					(-1)*r0y*cos(th0) + (-1)*r0x*sin(th0) + (-1)*l1*sin(q1 + th0) + (-1) 
					*r1*sin(q1 + th0) + (-1)*l2*sin(q1 + q2 + th0) + (-1)*r2*sin(q1 + q2 +  
					th0) + (-1)*l3*sin(q1 + q2 + q3 + th0) + (-1)*r3*sin(q1 + q2 + q3 + th0)),
					(-1) 
					*q3dot*(l3 + r3)*sin(q1 + q2 + q3 + th0) + q2dot*((-1)*(l2 + r2)*sin(q1 +  
					q2 + th0) + (-1)*(l3 + r3)*sin(q1 + q2 + q3 + th0)) + q1dot*((-1)*(l1 + r1)* 
					sin(q1 + th0) + (-1)*(l2 + r2)*sin(q1 + q2 + th0) + (-1)*(l3 + r3)*sin(q1 +  
					q2 + q3 + th0)) + th0dot*((-1)*(l1 + r1)*sin(q1 + th0) + (-1)*(l2 + r2)* 
					sin(q1 + q2 + th0) + (-1)*(l3 + r3)*sin(q1 + q2 + q3 + th0)),
					(-1)*q3dot*(l3 +  
					r3)*sin(q1 + q2 + q3 + th0) + q1dot*((-1)*(l2 + r2)*sin(q1 + q2 + th0) + (-1) 
					*(l3 + r3)*sin(q1 + q2 + q3 + th0)) + q2dot*((-1)*(l2 + r2)*sin(q1 + q2 +  
					th0) + (-1)*(l3 + r3)*sin(q1 + q2 + q3 + th0)) + th0dot*((-1)*(l2 + r2)* 
					sin(q1 + q2 + th0) + (-1)*(l3 + r3)*sin(q1 + q2 + q3 + th0)),
					(-1)*q1dot*(l3 +  
					r3)*sin(q1 + q2 + q3 + th0) + (-1)*q2dot*(l3 + r3)*sin(q1 + q2 + q3 + th0) + ( 
					-1)*q3dot*(l3 + r3)*sin(q1 + q2 + q3 + th0) + (-1)*(l3 + r3)*th0dot*sin( 
					q1 + q2 + q3 + th0), //end of row 2
					0, 0, 0, 0, 0, (-1)*(l3 + r3)*th0dot*sin(q1 + q2 + q3 + th0);

			// std::cout << "Here is the matrix Jedot:\n" << Jedot << std::endl;

			Vector2d Jvw;
			Jvw << Je(0,2), Je(1,2);
			MatrixXd Jvq(2, 3);
			Jvq <<  Je(0,3), Je(0,4), Je(0,5),
					Je(1,3), Je(1,4), Je(1,5);
			Vector3d Jwq;
			Jwq << Je(2,3), Je(2,4), Je(2,5);  
			Vector2d Jvwdot;
			Jvwdot << Jedot(0,2), Jedot(1,2);
			MatrixXd Jvqdot(2, 3);
			Jvqdot <<  Jedot(0,3), Jedot(0,4), Jedot(0,5),
						Jedot(1,3), Jedot(1,4), Jedot(1,5);
			Vector3d Jwqdot;
			Jwqdot << Jedot(2,3), Jedot(2,4), Jedot(2,5); 
			
			MatrixXd J1(6,6);
			J1 << 1, 0, 0, 0, 0, 0,
					0, 1, 0, 0, 0, 0,
					0, 0, 1, 0, 0, 0,
					1, 0, Jvw(0), Jvq(0,0), Jvq(0,1), Jvq(0,2),
					0, 1, Jvw(1), Jvq(1,0), Jvq(1,1), Jvq(1,2),
					0, 0, 1, Jwq(0), Jwq(1), Jwq(2);  

			// std::cout << "Here is the matrix J1:\n" << J1 << std::endl;

			MatrixXd J1dot(6,6);
			J1dot << 0, 0, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0,
						0, 0, Jvwdot(0), Jvqdot(0,0), Jvqdot(0,1), Jvqdot(0,2),  
						0, 0, Jvwdot(1), Jvqdot(1,0), Jvqdot(1,1), Jvqdot(1,2), 
						0, 0, 0, Jwqdot(0), Jwqdot(1), Jwqdot(2);    
		
			// std::cout << "Here is the matrix J1dot:\n" << J1dot << std::endl;


			MatrixXd J1_trans(6,6);
			J1_trans = J1.transpose();
			MatrixXd J1_trans_inv(6,6);
			J1_trans_inv = J1_trans.inverse();
			MatrixXd J1_inv(6,6);
			J1_inv = J1.inverse();
			MatrixXd Hstar(6,6);
			Hstar = J1_trans_inv*H*J1_inv;
			VectorXd cstar(6);
			cstar = J1_trans_inv*(c-(H*J1_inv*J1dot*v1));
			MatrixXd Jstar(6,6);
			Jstar = J1_trans_inv;

			double DJstar =Jstar.determinant();

			MatrixXd H11star(2,2);
			MatrixXd H12star(2,4);
			MatrixXd H21star(4,2);
			MatrixXd H22star(4,4);


			H11star << Hstar(0,0), Hstar(0,1),
						Hstar(1,0), Hstar(1,1);

			H12star << Hstar(0,2), Hstar(0,3), Hstar(0,4), Hstar(0,5),
						Hstar(1,2), Hstar(1,3), Hstar(0,4), Hstar(0,5);

			H21star << Hstar(2,0), Hstar(2,1),
						Hstar(3,0), Hstar(3,1),
						Hstar(4,0), Hstar(4,1),
						Hstar(5,0), Hstar(5,1);

			H22star << Hstar(2,2), Hstar(2,3), Hstar(2,4), Hstar(2,5), 
						Hstar(3,2), Hstar(3,3), Hstar(3,4), Hstar(3,5),
						Hstar(4,2), Hstar(4,3), Hstar(4,4), Hstar(4,5),
						Hstar(5,2), Hstar(5,3), Hstar(5,4), Hstar(5,5);

			Vector2d c1star;
			Vector4d c2star;

			c1star << cstar(0,0), cstar(1,0);
			c2star << cstar(2,0), cstar(3,0), cstar(4,0), cstar(5,0);

			MatrixXd J12star(2,4);
			MatrixXd J22star(4,4);


			J12star << Jstar(0,2), Jstar(0,3), Jstar(0,4), Jstar(0,5), 
						Jstar(1,2), Jstar(1,3), Jstar(1,4), Jstar(1,5); 

			J22star << Jstar(2,2), Jstar(2,3), Jstar(2,4), Jstar(2,5), 
						Jstar(3,2), Jstar(3,3), Jstar(3,4), Jstar(3,5),
						Jstar(4,2), Jstar(4,3), Jstar(4,4), Jstar(4,5),
						Jstar(5,2), Jstar(5,3), Jstar(5,4), Jstar(5,5);
			
			
			MatrixXd Je_trans(6,3);
			Je_trans = Je.transpose();

			MatrixXd Jestar(6,3);
			Jestar = Jstar*Je_trans;

			Matrix2d Je11star;
			MatrixXd Je21star(4,2);
			Vector2d Je12star;
			Vector4d Je22star;
			
			Je11star << Jestar(0,0), Jestar(0,1),
						Jestar(1,0), Jestar(1,1);


			Je12star << Jestar(0,2), Jestar(1,2);

			Je21star << Jestar(2,0), Jestar(2,1),
						Jestar(3,0), Jestar(3,1),
						Jestar(4,0), Jestar(4,1),
						Jestar(5,0), Jestar(5,1);
			
			Je22star << Jestar(2,2), Jestar(3,2), Jestar(4,2), Jestar(5,2); 

			Matrix2d H11star_inv;
			H11star_inv = H11star.inverse();

			Matrix4d Hbar;
			Hbar=H22star-H21star*H11star_inv*H12star;
			Vector4d cbar;
			cbar=c2star-H21star*H11star_inv*c1star;
			Matrix4d Jbar;
			Jbar=J22star-H21star*H11star_inv*J12star;
			MatrixXd Jebar_f(4,2);
			Jebar_f=Je21star-H21star*H11star_inv*Je11star; 
			Vector4d Jebar_s;
			Jebar_s=Je22star-H21star*H11star_inv*Je12star;
			MatrixXd Jebar(4,3);
			Jebar << Jebar_f(0,0), Jebar_f(0,1), Jebar_s(0),
						Jebar_f(1,0), Jebar_f(1,1), Jebar_s(1),
						Jebar_f(2,0), Jebar_f(2,1), Jebar_s(2),
						Jebar_f(3,0), Jebar_f(3,1), Jebar_s(3);

			Vector3d omegabdot_des;
			omegabdot_des << 0, 0, theta0dotdot_des;

			double eb1=0;
			double eb2=0;
			double eb3=sin(theta0/2);
			double nb=cos(theta0/2);
			double eb1_des=0;
			double eb2_des=0;
			double eb3_des=sin(theta0_des/2);
			double nb_des=cos(theta0_des/2);

			Vector3d eb;
			eb << eb1, eb2, eb3;
			Vector3d eb_des;
			eb_des << eb1_des, eb2_des, eb3_des;

			Matrix3d Rb;
			Rb << pow(eb1,2)+(-1)*pow(eb2,2)+(-1)*pow(eb3,2)+pow(nb,2),      2*eb1*eb2+(-2)*eb3*nb,      2*eb1*eb3+2*eb2*nb,
			2*eb1*eb2+2*eb3*nb,       (-1)*pow(eb1,2)+pow(eb2,2)+(-1)*pow(eb3,2)+pow(nb,2),        2*eb2*eb3+(-2)*eb1*nb, 
			2*eb1*eb3+(-2)*eb2*nb,    2*eb2*eb3+2*eb1*nb,        (-1)*pow(eb1,2)+(-1)*pow(eb2,2)+pow(eb3,2)+pow(nb,2);

			
			Vector3d omegab_inert;
			omegab_inert << 0, 0, theta0Dot;

			
			Matrix3d Rb_trans;
			Rb_trans=Rb.transpose();
			
			Vector3d omegab;
			omegab=Rb_trans*omegab_inert;

			Vector3d omegab_des;
			omegab_des << 0, 0, theta0dot_des;


			Vector3d errob_omega;
			errob_omega=omegab-Rb_trans*omegab_des;

			Matrix3d ebdcross;
			ebdcross << 0, -eb_des(2,0), eb_des(1,0),
						eb_des(2,0), 0, -eb_des(0,0),
						-eb_des(1,0), eb_des(0,0), 0;

			Matrix3d Ebd;
			Ebd = nb_des*eye_3+ebdcross;

			Matrix3d Ebd_trans;
			Ebd_trans = Ebd.transpose();

			Vector3d error_base;
			error_base = Ebd_trans*eb-eb_des*nb;

			RowVector3d eb_des_trans;
			eb_des_trans = eb_des.transpose();

			double ebn = eb_des_trans*eb + nb_des*nb;

			Matrix3d omegabcross;
			omegabcross << 0, -omegab(2,0), omegab(1, 0),
							omegab(2, 0), 0, -omegab(0,0),
							-omegab(1, 0), omegab(0, 0), 0;

			double ee1 = 0.0;
			double ee2 = 0.0;
			double ee3 = sin(thetaE / 2);
			double ne = cos(thetaE / 2);

			double ee1_des = 0.0;
			double ee2_des = 0.0;
			double ee3_des = sin(thetaE_des / 2);
			double ne_des = cos(thetaE_des / 2);


			Vector3d ee(ee1, ee2, ee3);
			Vector3d ee_des(ee1_des, ee2_des, ee3_des);

			Vector3d omegae_des(0, 0, thetaEdot_des);
			Vector3d omegaedot_des(0, 0, thetaEdotdot_des);

			Matrix3d Re;
			Re <<
				pow(ee1,2)+(-1)*pow(ee2,2)+(-1)*pow(ee3,2)+pow(ne,2),
				2*ee1*ee2+(-2)*ee3*ne,
				2*ee1*ee3+2*ee2*ne,
				
				2*ee1*ee2+2*ee3*ne,
				(-1)*pow(ee1,2)+pow(ee2,2)+(-1)*pow(ee3,2)+pow(ne,2),
				2*ee2*ee3+(-2)*ee1*ne,
				
				2*ee1*ee3+(-2)*ee2*ne,
				2*ee2*ee3+2*ee1*ne,
				(-1)*pow(ee1,2)+(-1)*pow(ee2,2)+pow(ee3,2)+pow(ne,2);

			Vector3d omegae_LAR(0, 0, thetaEdot);

			Matrix3d Re_trans;
			Re_trans = Re.transpose();
			Vector3d omegae;
			omegae = Re_trans*omegae_LAR;

			Vector3d errore_omega;
			errore_omega = omegae - Re_trans*omegae_des;

			RowVector3d ee_des_trans;
			ee_des_trans = ee_des.transpose();
			double een=ee_des_trans*ee+ne_des*ne;

			Matrix3d omegaecross;
			omegaecross << 
					0, -omegae(2,0), omegae(1,0),
					omegae(2,0), 0, -omegae(0,0),
					-omegae(1,0), omegae(0,0), 0;

			Matrix3d eedcross;
			eedcross <<
					0, -ee_des(2,0), ee_des(1,0),
					ee_des(2,0), 0, -ee_des(0,0),
					-ee_des(1,0), ee_des(0,0), 0;

			Matrix3d Eed;
			Eed = ne_des*eye_3 + eedcross;
			Matrix3d Eed_trans;
			Eed_trans = Eed.transpose();

			Vector3d error_ee;
			error_ee = Eed_trans*ee - ee_des*ne;

			Matrix3d error_eecoss;
			error_eecoss << 0, -error_ee(2,0), error_ee(1,0),
							error_ee(2,0), 0, -error_ee(0,0),
							-error_ee(1,0), error_ee(0,0), 0;

			Matrix3d Ees;
			Ees = een*eye_3 + error_eecoss;

			Vector2d Qef(FextX, FextY);
			Vector3d Qet(0.0, 0.0, Next);        
			Vector3d uRW;
			RowVector3d errob_omega_trans;
			errob_omega_trans = errob_omega.transpose();
			// std::cout <<  << std::endl;
			// double wtf = errob_omega_trans*errob_omega / 4.0;
			// uRW = Rb*(Rb_trans*omegabdot_des+ omegabcross*errob_omega - KD*errob_omega - 2.0*(KP - wtf)*error_base/ebn);
			Vector2d umr;
			Matrix2d HPS_inv;
			HPS_inv = HPS.inverse();
			// umr = HPS_inv*(HPS*xEdotdot_des+ DPS*(xEdot_des-xEdot) + KPS*(xE_des-xE) + Qef);
			// umw;
			// umw = Re*((HA*Ees)\(HA*Ees*(Re_trans*omegaedot_des + omegaecross*errore_omega) - DA*Ees*errore_omega + 2*Re_trans*Qet - 2*(KA*Ees - HA*Ees*(errore_omega'*errore_omega)/4)*error_ee/een));
			// uRWn=uRW(3);
			// umwn=umw(3);
			// u=[uRWn;umr;umwn];

			Vector3d Qe(FextX, FextY, Next);
			// Qbar = Hbar*u + cbar-Jebar*Qe;
			double DHbar = Hbar.determinant();
			double DJbar = Jbar.determinant();
			// tau=Jbar\Qbar;
        } else {
			th0_des=th0_fin;
			th0dot_des=0;
			th0dotdot_des=0;
		
			xE=xE_fin;
			yE=yE_fin;
			thE=thE_fin;
		
			xEdot=0;
			yEdot=0;
			thEdot=0;
		
			xEdotdot=0;
			yEdotdot=0;
			thEdotdot=0;

			Vector3d qE_des(xE, yE, thE);
			ROS_INFO("qE_des      : %f %f %f %f", time, xE, yE, thE);
			Vector3d qEdot_des(xEdot, yEdot, thEdot);
			ROS_INFO("qEdot_des   : %f %f %f %f", time, xEdot, yEdot, thEdot);
			Vector3d qEdotdot_des(xEdotdot, yEdotdot, thEdotdot);
			ROS_INFO("qEdotdot_des: %f %f %f %f", time, xEdotdot, yEdotdot, thEdotdot);
		}
		
		// ROS_WARN("position: %f, torq: %f",rw_position,torq[3]);

		prev_secs = secs;
		
		// _secs.data = secs;
		// secs_pub.publish(_secs);
		// // torque.data = filter_torque(torq[0], prev_torq[0]);
		// torque.data = - 0.0000001;
		// ls_torque_pub.publish(torque);
		// // torque.data = filter_torque(torq[1], prev_torq[1]);
		// torque.data = 0.0000001;
		// le_torque_pub.publish(torque);
		// // torque.data = filter_torque(torq[2], prev_torq[2]);
		// torque.data = - 0.0000001;
		// re_torque_pub.publish(torque);
		// torque.data = torq[3];
		// rw_torque_pub.publish(torque);

		// temp_msg.data = qd[0];
		// ls_qd_pub.publish(temp_msg);
		// temp_msg.data = qd[1];
		// le_qd_pub.publish(temp_msg);
		// temp_msg.data = qd[2];
		// re_qd_pub.publish(temp_msg);

		// temp_msg.data = ls_position;
		// ls_pos_pub.publish(temp_msg);
		// temp_msg.data = le_position;
		// le_pos_pub.publish(temp_msg);
		// temp_msg.data = re_position;
		// re_pos_pub.publish(temp_msg);
	
		// temp_msg.data = ls_velocity;
		// ls_vel_pub.publish(temp_msg);
		// temp_msg.data = le_velocity;
		// le_vel_pub.publish(temp_msg);
		// temp_msg.data = re_velocity;
		// re_vel_pub.publish(temp_msg);
	
		// temp_msg.data = errorq[0];
		// ls_error_pub.publish(temp_msg);
		// temp_msg.data = errorq[1];
		// le_error_pub.publish(temp_msg);
		// temp_msg.data = errorq[2];
		// re_error_pub.publish(temp_msg);

		// // temp_msg.data = ps_x[ALX_GRIPPER];
		// temp_msg.data = gripper_x;
		// gripper_x_pub.publish(temp_msg);
		// // temp_msg.data = ps_y[ALX_GRIPPER];
		// temp_msg.data = gripper_y;
		// gripper_y_pub.publish(temp_msg);
		// // temp_msg.data = ps_th[ALX_GRIPPER];
		// temp_msg.data = gripper_th;
		// gripper_th_pub.publish(temp_msg);
		// temp_msg.data = ps_x[CEPHEUS];
		// cepheus_x_pub.publish(temp_msg);
		// temp_msg.data = ps_y[CEPHEUS];
		// cepheus_y_pub.publish(temp_msg);
		// temp_msg.data = ps_th[CEPHEUS];
		// cepheus_th_pub.publish(temp_msg);

		// temp_msg.data = gripper_x_dot;
		// gripper_x_dot_pub.publish(temp_msg);
		// temp_msg.data = gripper_y_dot;
		// gripper_y_dot_pub.publish(temp_msg);
		// temp_msg.data = gripper_th_dot;
		// gripper_th_dot_pub.publish(temp_msg);

		// temp_msg.data = gripper_error_x;
		// gripper_error_x_pub.publish(temp_msg);
		// temp_msg.data = gripper_error_y;
		// gripper_error_y_pub.publish(temp_msg);
		// temp_msg.data = gripper_error_th;
		// gripper_error_th_pub.publish(temp_msg);

		// temp_msg.data = gripper_x_desdot;
		// gripper_x_desdot_pub.publish(temp_msg);

		// temp_msg.data = determinant;
		// det_pub.publish(temp_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
