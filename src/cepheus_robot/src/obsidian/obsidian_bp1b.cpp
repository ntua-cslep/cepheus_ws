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

#define POS_FILTER 0.05
#define VEL_FILTER 0.3
#define TORQUE_LIMIT 0.00000001

using namespace Eigen;
using namespace std;

double acc_x = 0.0, acc_y = 0.0;
double acc_x_offset = 0.0, acc_y_offset = 0.0;
double acc_x_prev = 0.0, acc_y_prev = 0.0;

double ls_position = 0.0, le_position = 0.0, re_position = 0.0, rw_position = 0.0;
double ls_velocity = 0.0, le_velocity = 0.0, re_velocity = 0.0, rw_velocity = 0.0;
double x_value = 0.0, y_value = 0.0, z_value = 0.0, w_value = 0.0;
bool first_time_movement = true;
bool first_time_ls = true;
bool first_time_le = true;
bool first_time_re = true;

double ps_x = 0.0, ps_y = 0.0, ps_theta = 0.0, ps_theta_prev = 0.0, rw_position_prev = 0.0;
double ps_x_prev = 0.0, ps_y_prev = 0.0;
double angular_vel_z = 0.0, angular_vel_z_offset = 0.0, angular_vel_z_prev = 0.0;

void imuAccelerationCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
	geometry_msgs::Vector3Stamped temp;
	temp = *msg;
	acc_x = temp.vector.x - acc_x_offset;
	acc_y = temp.vector.y - acc_y_offset;
}

void imuAngularVelCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
	geometry_msgs::Vector3Stamped temp;
	temp = *msg;
	angular_vel_z = temp.vector.z - angular_vel_z_offset;
}

void PSCepheusCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
	geometry_msgs::TransformStamped temp;
	temp = *msg;

	double x = temp.transform.rotation.x;
	double y = temp.transform.rotation.y;
	double z = temp.transform.rotation.z;
	double w = temp.transform.rotation.w;
	double roll, pitch, yaw;

	x_value = x;
	y_value = y;
	z_value = z;
	w_value = w;

	tf::Quaternion q(x, y, z, w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

	ps_x = temp.transform.translation.x;
	ps_y = temp.transform.translation.y;
	ps_theta = yaw;
	// if (ps_theta < -1) {
	// 	ps_theta = ps_theta + 2*M_PI;
	// }
	return;
}

void lsPosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (first_time_ls) {
		ls_position = cmd->data;
		first_time_ls = false;
		return; 
	}
	if (abs(cmd->data - ls_position) > POS_FILTER)
		return;
	else
		ls_position = cmd->data;
}


void lePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (first_time_le) {
		le_position = cmd->data;
		first_time_le = false;
		return;
	}
	if (abs(cmd->data - le_position) > POS_FILTER)
		return;
	else
		le_position = cmd->data;
}


void rwPosCallback(const std_msgs::Float64::ConstPtr& cmd) {
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


void rwVelCallback(const std_msgs::Float64::ConstPtr& cmd) {

	// if (abs(cmd->data - rw_velocity) > 500)
	// 	return;
	// else
		rw_velocity = cmd->data;

}


double filter_torque(double torq, double prev) {
	if (torq == 0.0){
		torq = 0.00001;
		if (prev < 0.0)
			torq = torq * -1;
		printf("CHANGED ZERO TORQUE\n");
	}
	// if (torq >= 0.01)
	// 	torq = 0.01;
	// else if (torq <= -0.01)
	// 	torq = -0.01;

	// if (abs(torq - prev) >= 0.001)
	// 	torq = prev;

	return torq;
}

ros::Publisher reset_movement_pub;
std_msgs::Bool reset_movement;

sig_atomic_t volatile g_request_shutdown = 0;
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
	ros::CallbackQueue main_queue;
	nh.setCallbackQueue(&main_queue);


	ros::Subscriber ps_cepheus_sub =  nh.subscribe("map_to_cepheus", 1, PSCepheusCallback);
	ros::Subscriber imu_acc_sub = nh.subscribe("/imu/acceleration", 1, imuAccelerationCallback);
	ros::Subscriber imu_ang_vel_sub = nh.subscribe("/imu/angular_velocity", 1, imuAngularVelCallback);

	ros::Publisher start_moving_pub = nh.advertise<std_msgs::Bool>("start_moving", 1);
	reset_movement_pub = nh.advertise<std_msgs::Bool>("reset_movement", 1);
	// ros::Subscriber phase_space_sub =  nh.subscribe("joint_states", 1, statesCallback);
	ros::Publisher rw_torque_pub = nh.advertise<std_msgs::Float64>("set_reaction_wheel_effort", 1);
	ros::Publisher ls_torque_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_effort", 1);
	ros::Publisher le_torque_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_effort", 1);
	ros::Publisher re_torque_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_effort", 1);
	ros::Publisher rw_trq_pub = nh.advertise<std_msgs::Float64>("rw_torque", 1);
	ros::Publisher ls_trq_pub = nh.advertise<std_msgs::Float64>("ls_torque", 1);
	ros::Publisher le_trq_pub = nh.advertise<std_msgs::Float64>("le_torque", 1);
	ros::Publisher re_trq_pub = nh.advertise<std_msgs::Float64>("re_torque", 1);
	ros::Publisher rw_pos_pub = nh.advertise<std_msgs::Float64>("reaction_wheel_pos", 1);
	ros::Publisher rw_vel_pub = nh.advertise<std_msgs::Float64>("reaction_wheel_vel", 1);
	ros::Publisher rw_vell_pub = nh.advertise<std_msgs::Float64>("reaction_wheel_vell", 1);
	ros::Publisher thrust_pub = nh.advertise<geometry_msgs::Vector3Stamped>("cmd_thrust", 1);
	ros::Publisher Fx_base_pub = nh.advertise<std_msgs::Float64>("Fx_base", 1);
	ros::Publisher Fy_base_pub = nh.advertise<std_msgs::Float64>("Fy_base", 1);
	ros::Publisher Tz_base_pub = nh.advertise<std_msgs::Float64>("Tz_base", 1);
	ros::Publisher thrust_x_pub = nh.advertise<std_msgs::Float64>("thrust_x", 1);
	ros::Publisher thrust_y_pub = nh.advertise<std_msgs::Float64>("thrust_y", 1);
	ros::Publisher thrust_z_pub = nh.advertise<std_msgs::Float64>("thrust_z", 1);

	ros::Publisher cepheus_x_pub = nh.advertise<std_msgs::Float64>("c_x", 1);
	ros::Publisher cepheus_y_pub = nh.advertise<std_msgs::Float64>("c_y", 1);
	ros::Publisher cepheus_xdot_pub = nh.advertise<std_msgs::Float64>("c_xdot", 1);
	ros::Publisher cepheus_ydot_pub = nh.advertise<std_msgs::Float64>("c_ydot", 1);

	ros::Publisher x_value_pub = nh.advertise<std_msgs::Float64>("x_value", 1);
	ros::Publisher y_value_pub = nh.advertise<std_msgs::Float64>("y_value", 1);
	ros::Publisher z_value_pub = nh.advertise<std_msgs::Float64>("z_value", 1);
	ros::Publisher w_value_pub = nh.advertise<std_msgs::Float64>("w_value", 1);

	ros::Publisher ls_pos_pub = nh.advertise<std_msgs::Float64>("left_shoulder_pos", 1);
	ros::Publisher le_pos_pub = nh.advertise<std_msgs::Float64>("left_elbow_pos", 1);
	ros::Publisher ls_vel_pub = nh.advertise<std_msgs::Float64>("left_shoulder_vel", 1);
	ros::Publisher le_vel_pub = nh.advertise<std_msgs::Float64>("left_elbow_vel", 1);
	
	ros::Publisher qe_des_x_pub = nh.advertise<std_msgs::Float64>("qe_des_x", 1);
	ros::Publisher qe_des_y_pub = nh.advertise<std_msgs::Float64>("qe_des_y", 1);
	ros::Publisher qe_dot_des_x_pub = nh.advertise<std_msgs::Float64>("qe_dot_des_x", 1);
	ros::Publisher qe_dot_des_y_pub = nh.advertise<std_msgs::Float64>("qe_dot_des_y", 1);
	ros::Publisher qe_dotdot_des_x_pub = nh.advertise<std_msgs::Float64>("qe_dotdot_des_x", 1);
	ros::Publisher qe_dotdot_des_y_pub = nh.advertise<std_msgs::Float64>("qe_dotdot_des_y", 1);

	ros::Publisher imu_angular_vel_pub = nh.advertise<std_msgs::Float64>("imu_angular_vel", 1);
	ros::Publisher ps_theta_pub = nh.advertise<std_msgs::Float64>("ps_theta", 1);
	ros::Publisher ps_theta_dot_pub = nh.advertise<std_msgs::Float64>("ps_theta_dot", 1);
    ros::Publisher imu_vel_y_pub = nh.advertise<std_msgs::Float64>("imu_vel_y", 1);
    ros::Publisher imu_vel_x_pub = nh.advertise<std_msgs::Float64>("imu_vel_x", 1);
    ros::Publisher imu_acc_y_pub = nh.advertise<std_msgs::Float64>("imu_acc_y", 1);
    ros::Publisher imu_acc_x_pub = nh.advertise<std_msgs::Float64>("imu_acc_x", 1);


	ros::Publisher baseposx_des_pub = nh.advertise<std_msgs::Float64>("baseposx_des", 1);
	ros::Publisher baseposy_des_pub = nh.advertise<std_msgs::Float64>("baseposy_des", 1);
	ros::Publisher theta0_des_pub = nh.advertise<std_msgs::Float64>("theta0_des", 1);
	ros::Publisher baseposxDot_des_pub = nh.advertise<std_msgs::Float64>("baseposxDot_des", 1);
	ros::Publisher baseposyDot_des_pub = nh.advertise<std_msgs::Float64>("baseposyDot_des", 1);
	ros::Publisher theta0Dot_des_pub = nh.advertise<std_msgs::Float64>("theta0Dot_des", 1);

	ros::Subscriber ls_pos_sub = nh.subscribe("read_left_shoulder_position", 1, lsPosCallback);
	ros::Subscriber le_pos_sub = nh.subscribe("read_left_elbow_position", 1, lePosCallback);
	ros::Subscriber rw_pos_sub = nh.subscribe("read_reaction_wheel_position", 1, rwPosCallback);
	ros::Subscriber ls_vel_sub = nh.subscribe("read_left_shoulder_velocity", 1, lsVelCallback);
	ros::Subscriber le_vel_sub = nh.subscribe("read_left_elbow_velocity", 1, leVelCallback);
	ros::Subscriber rw_vel_sub = nh.subscribe("read_reaction_wheel_velocity", 1, rwVelCallback);

	ros::Publisher secs_pub = nh.advertise<std_msgs::Float64>("secs", 1);

	double rate;
	ros::param::param<double>("~loop_rate", rate, 200);
	ros::Rate loop_rate(rate);

	string yaml_path = ros::package::getPath("cepheus_robot");
    yaml_path.append("/config/imu_offsets.yaml");
	YAML::Node imu_offsets = YAML::LoadFile(yaml_path);
	acc_x_offset = imu_offsets["acc_x_offset"].as<double>();
	acc_y_offset = imu_offsets["acc_y_offset"].as<double>();
	angular_vel_z_offset = imu_offsets["angular_vel_z_offset"].as<double>();

	ros::Time curr_time, t_beg = ros::Time::now();
	ros::Duration all_time;
	double prev_secs = 0.0;
	double t = 0.0, time = 0.0, move_time = 0.0, tminus = 0.0, t0=0;
	// double tf = 23.365;//identification
	double tf = 17.00;//validation
	double wf = (2 * M_PI)/tf;
	double a0, a1, a2, a3, a4, a5;
	double s_0, s_f, s0dot, sfdot, s0dotdot, sfdotdot;
	double a11 = 0.0;
	double b11 = 0.0;
	double a21 = 0.0;
	double b21 = 0.0;
	double a31 = 0.0;
	double b31 = 0.0;
	double a12 = 0.0;
	double b12 = 0.0;
	double a22 = 0.0;
	double b22 = 0.0;
	double a32 = 0.0;
	double b32 = 0.0;

	// double A1 = 0.38/wf; //0.64 max
	// double A2 = 0.14/wf; //0.36 max

	double theta1init = 0.0;
	double theta2init = 0.0;
	double theta1fin = 0.0;
	double theta2fin = 0.0;
	double theta0_in = 0.0;
	double theta0_fin = 0.0;

	double c01 = 0.0;
	double c11 = 0.0;
	double c21 = 0.0;
	double c31 = 0.0;
	double c41 = 0.0;
	double c51 = 0.0;
	double c02 = 0.0;
	double c12 = 0.0;
	double c22 = 0.0;
	double c32 = 0.0;
	double c42 = 0.0;
	double c52 = 0.0;

	double Fmax, Fmax_thrust = 0.3, rpx_in, vpx_in, rpy_in, vpy_in, rpx_tar, vpx_tar, rpy_tar, vpy_tar;
	double a, amax, b, c, delta, t11, t12, t21, t22, solt11, solt12, solt21, solt22;
	double baseposx_des, baseposy_des, baseposxDot_des, baseposyDot_des, baseposxDotDot_des, baseposyDotDot_des; 
	double theta1_des = 0.0;
	double theta1Dot_des = 0.0;
	double theta1DotDot_des = 0.0;
	double theta2_des = 0.0;
	double theta2Dot_des = 0.0;
	double theta2DotDot_des = 0.0;
	double theta0_des, theta0Dot_des, theta0DotDot_des;
	double ps_xdot, ps_ydot;
	double yamp = 0.3;

	double m0 = 13.0, m1 = 0.4409, m2 = 0.3874;
	double M = m0 + m1 + m2;
	double l1 = 0.2690;
	double l2 = 0.2406;
	double r0x = 0.1645;
	double r0y = 0.0;
	double r1 = 0.101;
	double I0z = 0.1775;
	double I1z = 0.0068;
	double I2z = 0.0030;

	VectorXd xx(12);
	VectorXd x(12);

	MatrixXd G(6, 6);
	VectorXd B1(6);
	VectorXd x1(6);
	MatrixXd inv(6, 6);

	Vector2d theta_des;
	Vector2d thetaDot_des;
	Vector2d thetaDotDot_des;

	double Kp_ls = 0.09644245;
	double Kp_le = 0.01127526;
	double Kp_rw = 3.0;
	double Kd_ls = 0.009644245;
	double Kd_le = 0.001127526;
	double Kd_rw = 1.4;

	MatrixXd eye_5;
	eye_5.setIdentity(5, 5);


	MatrixXd KD(5,5);
	// KD = 12*eye_5;
	KD << 	12, 0, 0, 0, 0,
			0, 12, 0, 0, 0,
			0, 0, Kd_rw*10, 0, 0,
			0, 0, 0, Kd_ls, 0,
			0, 0, 0, 0, Kd_le;
	
	MatrixXd KP(5,5);
	// KP = 36*eye_5;
	KP << 	36, 0, 0, 0, 0,
			0, 36, 0, 0, 0,
			0, 0, Kp_rw*10, 0, 0,
			0, 0, 0, Kp_ls, 0,
			0, 0, 0, 0, Kp_le;

	double errorq[4];
	double error_qdot[4];
	double torq[4];
	double prev_torq[4];
	double qd[4];
	double qd_dot[4];
	double ps_theta_dot = 0.0;
	double ps_theta_dot_prev = 0.0;
	double rw_vel = 0.0;
	double imu_vel_x = 0.0, imu_vel_y = 0.0;
	double imu_vel_x_prev = 0.0, imu_vel_y_prev = 0.0;
	double wff;

	for (int i = 0; i < 4; i++) {
		errorq[i] = 0.0;
		error_qdot[i] = 0.0;
		torq[i] = 0.0;
		prev_torq[i] = 0.0;
		qd[i] = 0.0;
		qd_dot[i] = 0.0;
	}

	std_msgs::Bool start_moving;
	start_moving.data = true;

	std_msgs::Float64 torque;
	std_msgs::Float64 offset;
	std_msgs::Float64 temp_msg;
	std_msgs::Float64 _secs;

	geometry_msgs::Vector3Stamped thrust_vector;

	double time_step = 0.0, time_step_prev = 0.0;
	double secs;

	while (ls_position == 0.0 || le_position == 0.0 || ps_theta == 0.0 || angular_vel_z == 0.0 || acc_x == 0 || acc_y == 0) {
		cout << "ps_theta " << ps_theta << " ls_position " << ls_position << " angular_vel_z " << angular_vel_z << endl;
		cout << "acc_x " << acc_x << " acc_y " << acc_y << "ps_x " << endl;
		main_queue.callAvailable(ros::WallDuration(1.0));
	}


	while (!g_request_shutdown) {
		// order initialize_arm_node to release arm
		start_moving_pub.publish(start_moving);

		for (int i = 0; i < 3; i++) {
			prev_torq[i] = torq[i];
		}

		curr_time = ros::Time::now();
		all_time = curr_time - t_beg;

		secs = all_time.sec + all_time.nsec * pow(10, -9);
		time_step_prev = time_step;
		time_step = secs - prev_secs;

		// time_step = 0.05;

		ps_theta_dot = (ps_theta - ps_theta_prev) / time_step;
		// rw_vel = (rw_position - rw_position_prev) / time_step;
		// if (abs(ps_theta_dot - ps_theta_dot_prev) > 0.1)
		// 	ps_theta_dot = ps_theta_dot / 10;

		// if (abs(acc_x) < 0.0005)
		// 	acc_x = 0.0;

		// if (abs(acc_y) < 0.002)
		// 	acc_y = 0.0;

		// if (abs(angular_vel_z) < 0.007)
		// 	angular_vel_z = 0.0;

		

/////////////////////////////////////////////////// INITIAL CONDITIONS //////////////////////**********************************///////////////////////////////////////////////

		if (first_time_movement) {
			move_time = secs;

			/////////////////////////////////////////////////// MANIPULATOR //////////////////////**********************************///////////////////////////////////////////////

			x <<	-0.0502531230821664, -0.00963668816229735, -0.0313752998135336, 0.0226123153397242,
					0.11837350042829, -0.122707402889061, -0.0643085499244572, -0.0153700294670522,
					0.0664011498205846, 0.0476593441567345, 0.0887493654713886, 0.00800266207171;
			
			xx = 0.9 * x;

			a11 = xx(0);
			b11 = xx(1);
			a21 = xx(2);
			b21 = xx(3);
			a31 = xx(4);
			b31 = xx(5);
			a12 = xx(6);
			b12 = xx(7);
			a22 = xx(8);
			b22 = xx(9);
			a32 = xx(10);
			b32 = xx(11);

			c01 = (6*b11 + 3*b21 + 2*b31 + 6*theta1init*wf)/(6*wf);
			c11 = - a11 - a21 - a31;
			c21 = - (b11*wf)/2 - b21*wf - (3*b31*wf)/2;
			c31 = (10*theta1fin - 10*theta1init + 10*a11*tf + 10*a21*tf + 10*a31*tf + b11*pow(tf, 2)*wf + 2*b21*pow(tf, 2)*wf + 3*b31*pow(tf, 2)*wf)/pow(tf, 3);
			c41 = -(30*theta1fin - 30*theta1init + 30*a11*tf + 30*a21*tf + 30*a31*tf + b11*pow(tf, 2)*wf + 2*b21*pow(tf, 2)*wf + 3*b31*pow(tf, 2)*wf)/(2*pow(tf, 4));
			c51 = (6*(theta1fin - theta1init + a11*tf + a21*tf + a31*tf))/pow(tf, 5);
			c02 = (6*b12 + 3*b22 + 2*b32 + 6*theta2init*wf)/(6*wf);
			c12 = - a12 - a22 - a32;
			c22 = - (b12*wf)/2 - b22*wf - (3*b32*wf)/2;
			c32 = (10*theta2fin - 10*theta2init + 10*a12*tf + 10*a22*tf + 10*a32*tf + b12*pow(tf, 2)*wf + 2*b22*pow(tf, 2)*wf + 3*b32*pow(tf, 2)*wf)/pow(tf, 3);
			c42 = -(30*theta2fin - 30*theta2init + 30*a12*tf + 30*a22*tf + 30*a32*tf + b12*pow(tf, 2)*wf + 2*b22*pow(tf, 2)*wf + 3*b32*pow(tf, 2)*wf)/(2*pow(tf, 4));
			c52 = (6*(theta2fin - theta2init + a12*tf + a22*tf + a32*tf))/pow(tf, 5);

			/////////////////////////////////////////////////// BASE THETA //////////////////////**********************************///////////////////////////////////////////////

			theta0_in = ps_theta;
			theta0_fin = theta0_in + (180*M_PI)/180;//validation
			// theta0_fin = theta0_in + (120*M_PI)/180;//identification

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

			// cout << "Here is the matrix G:\n" << G << endl;
			inv = G.inverse();
			// cout << "Here is the inv matrix G:\n" << inv << endl;
			x1 = inv*B1;

			a0 = x1(0);
			a1 = x1(1);
			a2 = x1(2);
			a3 = x1(3);
			a4 = x1(4);
			a5 = x1(5);

			/////////////////////////////////////////////////// BASE X //////////////////////**********************************///////////////////////////////////////////////


			Fmax = 2*cos(M_PI/3)*Fmax_thrust;
			amax = Fmax/M;

			rpx_in = ps_x;
			vpx_in = 0.0; 

			rpx_tar = ps_x + 1.5;//validation
			// rpx_tar = ps_x + 1.6;//identification
			vpx_tar = 0.0; 


			//validation
			a = amax;
			b = 2*(vpx_in - vpx_tar);
			c = rpx_in-rpx_tar + 0.5*pow((vpx_tar - vpx_in),2)/amax;
			delta = pow(b,2) - (4*a*c);

			t11=(-b + sqrt(delta))/(2*a);
			t12=(-b - sqrt(delta))/(2*a);

			if ((t11>=0) && (t12>=0)){
				solt11 = min(t11,t12);
				solt12 = (2*solt11) - ((vpx_tar - vpx_in)/amax);
			}
			else if ((t11>=0) && (t12<=0)){ 
				solt11 = t11;
				solt12 = (2*solt11) - ((vpx_tar - vpx_in)/amax);
			}
			else if ((t11<=0) && (t12>=0)){
				solt11 = t12;
				solt12 = (2*solt11) - ((vpx_tar - vpx_in)/amax);
			}
			else {
				solt11 = 0.0;
				solt12 = 0.0;
			}

			/////////////////////////////////////////////////// BASE Y //////////////////////**********************************///////////////////////////////////////////////

			Fmax = 2*cos(M_PI/3)*Fmax_thrust;
			amax = Fmax/M;

			rpy_in = ps_y;
			vpy_in = 0.0; 

			rpy_tar = ps_y + 0.5;//validation
			// rpy_tar = ps_y;//identification
			vpy_tar = 0.0; 

			//validation
			a = amax;
			b = 2*(vpy_in - vpy_tar);
			c = rpy_in-rpy_tar + 0.5*pow((vpy_tar - vpy_in),2)/amax;
			delta = pow(b,2) - (4*a*c);

			t21 =(-b + sqrt(delta))/(2*a);
			t22 =(-b - sqrt(delta))/(2*a);

			if ((t21>=0) && (t22>=0)){
				solt21 = min(t21,t22);
				solt22 = (2*solt21) - ((vpy_tar - vpy_in)/amax);
			}
			else if ((t21>=0) && (t22<=0)){ 
				solt21 = t21;
				solt22 = (2*solt21) - ((vpy_tar - vpy_in)/amax);
			}
			else if ((t21<=0) && (t22>=0)){
				solt21 = t22;
				solt22 = (2*solt21) - ((vpy_tar - vpy_in)/amax);
			}
			else {
				solt21 = 0.0;
				solt22 = 0.0;
			}
			
			wff = 2*(M_PI)/(rpx_tar - rpx_in);


			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			ps_x_prev = ps_x;
			ps_y_prev = ps_y;
			first_time_movement = false;
		}

		t = secs - move_time;


		/////////////////////////////////////////////////// DESIRED TRAJECTORIES//////////////////////**********************************///////////////////////////////////////////////

		if (t <= tf ) {
			tminus = t;

			/////////////////////////////////////////////////// MANIPULATOR //////////////////////**********************************///////////////////////////////////////////////
			//validation
			theta1_des = 69.3/180*(M_PI)*sin(wf*t);
			theta1Dot_des = 69.3/180*(M_PI)*wf*cos(wf*t);
			theta1DotDot_des = -69.3/180*(M_PI)*pow(wf,2)*sin(wf*t);
			theta2_des = 25.5/180*(M_PI)*sin(wf*t);
			theta2Dot_des = 25.5/180*(M_PI)*wf*cos(wf*t);
			theta2DotDot_des = -25.5/180*(M_PI)*pow(wf,2)*sin(wf*t);
	
			//identification		
			// theta1_des = a11/wf*sin(wf*tminus)-b11/wf*cos(wf*tminus)+a21/wf/2*sin(2*wf*tminus)-b21/wf/2*cos(2*wf*tminus) +a31/wf/3*sin(3*wf*tminus)-b31/wf/3*cos(3*wf*tminus)+c01+c11*tminus+c21*pow(tminus, 2)+c31*pow(tminus, 3)+c41*pow(tminus, 4)+c51*pow(tminus, 5);
			// theta1Dot_des = a11*cos(wf*tminus)+b11*sin(wf*tminus)+a21*cos(2*wf*tminus)+b21*sin(2*wf*tminus) +a31*cos(3*wf*tminus)+b31*sin(3*wf*tminus)+c11+2*c21*tminus+3*c31*pow(tminus, 2)+4*c41*pow(tminus, 3)+5*c51*pow(tminus, 4);
			// theta1DotDot_des = -a11*wf*sin(wf*tminus)+b11*wf*cos(wf*tminus)+a21*(-2*wf)*sin(2*wf*tminus)+b21*2*wf*cos(2*wf*tminus) +a31*(-3*wf)*sin(3*wf*tminus)+b31*3*wf*cos(3*wf*tminus)+2*c21+3*2*c31*tminus+4*3*c41*pow(tminus, 2)+5*4*c51*pow(tminus, 3);
			// theta2_des = a12/wf*sin(wf*tminus)-b12/wf*cos(wf*tminus)+a22/wf/2*sin(2*wf*tminus)-b22/wf/2*cos(2*wf*tminus) +a32/wf/3*sin(3*wf*tminus)-b32/wf/3*cos(3*wf*tminus)+c02+c12*tminus+c22*pow(tminus, 2)+c32*pow(tminus, 3)+c42*pow(tminus, 4)+c52*pow(tminus, 5);
			// theta2Dot_des = a12*cos(wf*tminus)+b12*sin(wf*tminus)+a22*cos(2*wf*tminus)+b22*sin(2*wf*tminus) +a32*cos(3*wf*tminus)+b32*sin(3*wf*tminus)+c12+2*c22*tminus+3*c32*pow(tminus, 2)+4*c42*pow(tminus, 3)+5*c52*pow(tminus, 4);
			// theta2DotDot_des = -a12*wf*sin(wf*tminus)+b12*wf*cos(wf*tminus)+a22*(-2*wf)*sin(2*wf*tminus)+b22*2*wf*cos(2*wf*tminus) +a32*(-3*wf)*sin(3*wf*tminus)+b32*3*wf*cos(3*wf*tminus)+2*c22+3*2*c32*tminus+4*3*c42*pow(tminus, 2)+5*4*c52*pow(tminus, 3);
		
			theta_des << theta1_des, theta2_des;
			thetaDot_des << theta1Dot_des, theta2Dot_des;
			thetaDotDot_des << theta1DotDot_des, theta2DotDot_des;

			/////////////////////////////////////////////////// BASE THETA //////////////////////**********************************///////////////////////////////////////////////

			double s = a5*pow(tminus,5)+a4*pow(tminus,4)+a3*pow(tminus,3)+a2*pow(tminus,2)+a1*tminus+a0;
			double sDot = 5*a5*pow(tminus,4)+4*a4*pow(tminus,3)+3*a3*pow(tminus,2)+2*a2*tminus+a1;
			double sDotDot = 20*a5*pow(tminus,3)+12*a4*pow(tminus,2)+6*a3*tminus+2*a2;

			theta0_des= theta0_in+s*(theta0_fin-theta0_in);
			theta0Dot_des = sDot*(theta0_fin-theta0_in);
			theta0DotDot_des = sDotDot*(theta0_fin-theta0_in);

			/////////////////////////////////////////////////// BASE X & Y //////////////////////**********************************///////////////////////////////////////////////
			//validation
			if (t<=solt11){
				baseposx_des = rpx_in + vpx_in*t + 0.5*amax*pow(t,2);
				baseposxDot_des = vpx_in + amax*t;
				baseposxDotDot_des = amax;
			}
			else if ((t>solt11) && (t<=solt12)){
				baseposx_des = rpx_in + vpx_in*solt11 + 0.5*amax*pow(solt11,2) + (vpx_in + amax*solt11)*(t - solt11) - 0.5*amax*pow((t - solt11),2);
				baseposxDot_des = vpx_in + amax*(2*solt11 - t);
				baseposxDotDot_des = -amax;
			}
			else {
				baseposx_des = rpx_tar;
				baseposxDot_des = 0;
				baseposxDotDot_des = 0;
			}

			if (t<=solt21){
				baseposy_des = rpy_in + vpy_in*t + 0.5*amax*pow(t,2);
				baseposyDot_des = vpy_in + amax*t;
				baseposyDotDot_des = amax;
			}
			else if ((t>solt21) && (t<=solt22)){
				baseposy_des = rpy_in + vpy_in*solt21 + 0.5*amax*pow(solt21,2) + (vpy_in + amax*solt21)*(t - solt21) - 0.5*amax*pow((t - solt21),2);
				baseposyDot_des = vpy_in + amax*(2*solt21 - t);
				baseposyDotDot_des = -amax;
			}
			else {
				baseposy_des = rpy_tar;
				baseposyDot_des = 0;
				baseposyDotDot_des = 0;
			}
			
			//identification
			// baseposxDot_des = (rpx_tar - rpx_in)/tf;
			// baseposx_des = rpx_in + baseposxDot_des*t;
			// baseposxDotDot_des = 0.0;

			// baseposy_des = rpy_in - yamp*sin(wff*baseposxDot_des*t);
			// baseposyDot_des = -yamp*wff*baseposxDot_des*cos(wff*baseposxDot_des*t);
			// baseposyDotDot_des = yamp*pow((wff*baseposxDot_des),2)*sin(wff*baseposxDot_des*t);
	
		}
		
		/////////////////////////////////////////////////////////////////controller/////////////////////////////////////////////////////////
		if (ps_x != ps_x_prev)
			ps_xdot = (ps_x - ps_x_prev)/time_step;
		if (ps_y != ps_y_prev)
			ps_ydot = (ps_y - ps_y_prev)/time_step;
		ps_x_prev = ps_x;
		ps_y_prev = ps_y;

		Vector2d rp00;
		rp00 << 0, 0;

		double theta01 = 0.0;

		VectorXd qstar(5); 
		qstar << ps_x, ps_y, ps_theta, ls_position, le_position; 

		VectorXd qstar_des(5); 
		qstar_des << baseposx_des, baseposy_des, theta0_des, theta1_des, theta2_des;

		VectorXd qstarDot(5);
		qstarDot << ps_xdot, ps_ydot, angular_vel_z, ls_velocity, le_velocity;

		VectorXd qstarDot_des(5); 
		qstarDot_des << baseposxDot_des, baseposyDot_des, theta0Dot_des, theta1Dot_des, theta2Dot_des;

		// VectorXd qstarDotDot_des(5); 
		// qstarDotDot_des << baseposxDotDot_des, baseposyDotDot_des, theta0DotDot_des, theta1DotDot_des, theta2DotDot_des;

		double theta0 = qstar(2);
		double theta1 = qstar(3);
		double theta2 = qstar(4);
		// double theta0 = 0;
		// double theta1 = 1;
		// double theta2 = 2;

		double theta0Dot = qstarDot(2);
		double theta1Dot = qstarDot(3);
		double theta2Dot = qstarDot(4);
		// double theta0Dot = 0.2;
		// double theta1Dot = 0.1;
		// double theta2Dot = 0.5;

		MatrixXd H(5, 5);

		H << 	M,
				0,
				m1*((-1)*r0y*cos(theta0) + (-1)*r0x*sin(theta0) + (-1)*l1*sin(theta01 + theta1 + theta0)) + m2*((-1)*r0y*cos(theta0) + (-1)*r0x*sin(theta0) + (-1)*l1*sin(theta01 + theta1 + theta0) + (-1)*r1*sin(theta01 + theta1 + theta0) + (-1)*l2*sin(theta01 + theta1 + theta2 + theta0)),
				(-1)*l1*m1*sin(theta01 + theta1 + theta0) + m2*((-1)*l1*sin(theta01 + theta1 + theta0) + (-1)*r1*sin(theta01 + theta1 + theta0) + (-1)*l2*sin(theta01 + theta1 + theta2 + theta0)),
				(-1)*l2*m2*sin(theta01 + theta1 + theta2 + theta0),
				
				0,
				M,
				m1*(r0x*cos(theta0) + l1*cos(theta01 + theta1 + theta0) + (-1)*r0y*sin(theta0)) + m2*(r0x*cos(theta0) + l1*cos(theta01 + theta1 + theta0) + r1*cos(theta01 + theta1 + theta0) + l2*cos(theta01 + theta1 + theta2 + theta0) + (-1)*r0y*sin(theta0)),
				l1*m1*cos(theta01 + theta1 + theta0) + m2*(l1*cos(theta01 + theta1 + theta0) + r1*cos(theta01 + theta1 + theta0) + l2*cos(theta01 + theta1 + theta2 + theta0)),
				l2*m2*cos(theta01 + theta1 + theta2 + theta0),
				
				m1*((-1)*r0y*cos(theta0) + (-1)*r0x*sin(theta0) + (-1)*l1*sin(theta01 + theta1 + theta0)) + m2*((-1)*r0y*cos(theta0) + (-1)*r0x*sin(theta0) + (-1)*l1*sin(theta01 + theta1 + theta0) + (-1)*r1*sin(theta01 + theta1 + theta0) + (-1)*l2*sin(theta01 + theta1 + theta2 + theta0)),
				m1*(r0x*cos(theta0) + l1*cos(theta01 + theta1 + theta0) + (-1)*r0y*sin(theta0)) + m2*(r0x*cos(theta0) + l1*cos(theta01 + theta1 + theta0) + r1*cos(theta01 + theta1 + theta0) + l2*cos(theta01 + theta1 + theta2 + theta0) + (-1)*r0y*sin(theta0)),
				(0.5)*(2*I0z + 2*I1z + 2*I2z + m1*(2*pow((r0x*cos(theta0) + l1*cos(theta01 + theta1 + theta0) + (-1)*r0y*sin(theta0)),2) + 2*pow(((-1)*r0y*cos(theta0) + (-1)*r0x*sin(theta0) + (-1)*l1*sin(theta01 + theta1 + theta0)),2)) + m2*(2*pow((r0x*cos(theta0) + l1*cos(theta01 + theta1 + theta0) + r1*cos(theta01 + theta1 + theta0) + l2*cos(theta01 + theta1 + theta2 + theta0) + (-1)*r0y*sin(theta0)),2) + 2*pow(((-1)*r0y*cos(theta0) + (-1)*r0x*sin(theta0) + (-1)*l1*sin(theta01 + theta1 + theta0) + (-1)*r1*sin(theta01 + theta1 + theta0) + (-1)*l2*sin(theta01 + theta1 + theta2 + theta0)),2))),
				(0.5)*(2*I1z + 2*I2z + m1*(2*l1*cos(theta01 + theta1 + theta0)*(r0x*cos(theta0) + l1*cos(theta01 + theta1 + theta0) + (-1)*r0y*sin(theta0)) + (-2)*l1*sin(theta01 + theta1 + theta0)*((-1)*r0y*cos(theta0) + (-1)*r0x*sin(theta0) + (-1)*l1*sin(theta01 + theta1 + theta0)))+ m2*(2*(l1*cos(theta01 + theta1 + theta0) + r1*cos(theta01 + theta1 + theta0) + l2*cos(theta01 + theta1 + theta2 + theta0))*(r0x*cos(theta0) + l1*cos(theta01 + theta1 + theta0) + r1*cos(theta01 + theta1 + theta0) + l2*cos(theta01 + theta1 + theta2 + theta0) + (-1)*r0y*sin(theta0)) + 2*((-1)*l1*sin(theta01 + theta1 + theta0) + (-1)*r1*sin(theta01 + theta1 + theta0) + (-1)*l2*sin(theta01 + theta1 + theta2 + theta0))*((-1)*r0y*cos(theta0) + (-1)*r0x*sin(theta0) + (-1)*l1*sin(theta01 + theta1 + theta0) + (-1)*r1*sin(theta01 + theta1 + theta0) + (-1)*l2*sin(theta01 + theta1 + theta2 + theta0)))),
				(0.5)*(2*I2z + m2*(2*l2*cos(theta01 + theta1 + theta2 + theta0)*(r0x*cos(theta0) + l1*cos(theta01 + theta1 + theta0) + r1*cos(theta01 + theta1 + theta0) + l2*cos(theta01 + theta1 + theta2 + theta0) + (-1)*r0y*sin(theta0)) + (-2)*l2*sin(theta01 + theta1 + theta2 + theta0)*((-1)*r0y*cos(theta0) + (-1)*r0x*sin(theta0) + (-1)*l1*sin(theta01 + theta1 + theta0) + (-1)*r1*sin(theta01 + theta1 + theta0) + (-1)*l2*sin(theta01 + theta1 + theta2 + theta0)))),
				
				(-1)*l1*m1*sin(theta01 + theta1 + theta0) + m2*((-1)*l1*sin(theta01 + theta1 + theta0) + (-1)*r1*sin(theta01 + theta1 + theta0) + (-1)*l2*sin(theta01 + theta1 + theta2 + theta0)),
				l1*m1*cos(theta01 + theta1 + theta0) + m2*(l1*cos(theta01 + theta1 + theta0) + r1*cos(theta01 + theta1 + theta0) + l2*cos(theta01 + theta1 + theta2 + theta0)),
				(0.5)*(2*I1z + 2*I2z + m1*(2*l1*cos(theta01 + theta1 + theta0)*(r0x*cos(theta0) + l1*cos(theta01 + theta1 + theta0) + (-1)*r0y*sin(theta0)) + (-2)*l1*sin(theta01 + theta1 + theta0)*((-1)*r0y*cos(theta0) + (-1)*r0x*sin(theta0) + (-1)*l1*sin(theta01 + theta1 + theta0))) + m2*(2*(l1*cos(theta01 + theta1 + theta0) + r1*cos(theta01 + theta1 + theta0) + l2*cos(theta01 + theta1 + theta2 + theta0))*(r0x*cos(theta0) + l1*cos(theta01 + theta1 + theta0) + r1*cos(theta01 + theta1 + theta0) + l2*cos(theta01 + theta1 + theta2 + theta0) + (-1)*r0y*sin(theta0)) + 2*((-1)*l1*sin(theta01 + theta1 + theta0) + (-1)*r1*sin(theta01 + theta1 + theta0) + (-1)*l2*sin(theta01 + theta1 + theta2 + theta0))*((-1)*r0y*cos(theta0) + (-1)*r0x*sin(theta0) + (-1)*l1*sin(theta01 + theta1 + theta0) + (-1)*r1*sin(theta01 + theta1 + theta0) + (-1)*l2*sin(theta01 + theta1 + theta2 + theta0)))),
				(0.5)*(2*I1z + 2*I2z + m1*(2*pow(l1,2)*pow(cos(theta01 + theta1 + theta0),2) + 2*pow(l1,2)*pow(sin(theta01 + theta1 + theta0),2)) + m2*(2*pow((l1*cos(theta01 + theta1 + theta0) + r1*cos(theta01 + theta1 + theta0) + l2*cos(theta01 + theta1 + theta2 + theta0)),2) + 2*pow(((-1)*l1*sin(theta01 + theta1 + theta0) + (-1)*r1*sin(theta01 + theta1 + theta0) + (-1)*l2*sin(theta01 + theta1 + theta2 + theta0)),2))),
				(0.5)*(2*I2z + m2*(2*l2*cos(theta01 + theta1 + theta2 + theta0)*(l1*cos(theta01 + theta1 + theta0) + r1*cos(theta01 + theta1 + theta0) + l2*cos(theta01 + theta1 + theta2 + theta0)) + (-2)*l2*sin(theta01 + theta1 + theta2 + theta0)*((-1)*l1*sin(theta01 + theta1 + theta0) + (-1)*r1*sin(theta01 + theta1 + theta0) + (-1)*l2*sin(theta01 + theta1 + theta2 + theta0)))),
				
				(-1)*l2*m2*sin(theta01 + theta1 + theta2 + theta0),
				l2*m2*cos(theta01 + theta1 + theta2 + theta0),
				(0.5)*(2*I2z + m2*(2*l2*cos(theta01 + theta1 + theta2 + theta0)*(r0x*cos(theta0)+ l1*cos(theta01 + theta1 + theta0) + r1*cos(theta01 + theta1 + theta0) + l2*cos(theta01 + theta1 + theta2 + theta0) + (-1)*r0y*sin(theta0)) + (-2)*l2*sin(theta01 + theta1 + theta2 + theta0)*((-1)*r0y*cos(theta0) + (-1)*r0x*sin(theta0) + (-1)*l1*sin(theta01 + theta1 + theta0) + (-1)*r1*sin(theta01 + theta1 + theta0) + (-1)*l2*sin(theta01 + theta1 + theta2 + theta0)))),
				(0.5)*(2*I2z + m2*(2*l2*cos(theta01 + theta1 + theta2 + theta0)*(l1*cos(theta01 + theta1 + theta0) + r1*cos(theta01 + theta1 + theta0) + l2*cos(theta01 + theta1 + theta2 + theta0)) + (-2)*l2*sin(theta01 + theta1 + theta2 + theta0)*((-1)*l1*sin(theta01 + theta1 + theta0) + (-1)*r1*sin(theta01 + theta1 + theta0) + (-1)*l2*sin(theta01 + theta1 + theta2 + theta0)))),
				(0.5)*(2*I2z + m2*(2*pow(l2,2)*pow(cos(theta01 + theta1 + theta2 + theta0),2) + 2*pow(l2,2)*pow(sin(theta01 + theta1 + theta2 + theta0),2)));


		// // cout << "H Matrix:" << endl << H << endl << endl;
		// VectorXd c(5);

		// c <<	(-1)*(m1 + m2)*r0x*pow(theta0Dot,2)*cos(theta0) + (-1)*(l1*(m1 + m2) + m2*r1)*pow((theta1Dot + theta0Dot),2)*cos(theta01 + theta1 + theta0) + (-1)*l2*m2*pow(theta1Dot,2)*cos(theta01 + theta1 + theta2 + theta0) + (-2)*l2*m2*theta1Dot*theta2Dot*cos(theta01 + theta1 + theta2 + theta0) + (-1)*l2*m2*pow(theta2Dot,2)*cos(theta01 + theta1 + theta2 + theta0) + (-2)*l2*m2*theta1Dot*theta0Dot*cos(theta01 + theta1 + theta2 + theta0) + (-2)*l2*m2*theta2Dot*theta0Dot*cos(theta01 + theta1 + theta2 + theta0) + (-1)*l2*m2*pow(theta0Dot,2)*cos(theta01 + theta1 + theta2 + theta0) + m1*r0y*pow(theta0Dot,2)*sin(theta0) + m2*r0y*pow(theta0Dot,2)*sin(theta0),

		// 		(-1)*l2*m2*theta2Dot*(theta1Dot + theta2Dot + theta0Dot)*sin(theta01 + theta1 + theta2 + theta0) + theta1Dot*((-1)*l1*m1*(theta1Dot + theta0Dot)*sin(theta01 + theta1 + theta0) + m2*((-1)*(l1 + r1)*(theta1Dot + theta0Dot)*sin(theta01 + theta1 + theta0) + (-1)*l2*(theta1Dot + theta2Dot + theta0Dot)*sin(theta01 + theta1 + theta2 + theta0))) + theta0Dot*(m1*((-1)*theta0Dot*(r0y*cos(theta0) + r0x*sin(theta0)) + (-1)*l1*(theta1Dot + theta0Dot)*sin(theta01 + theta1 + theta0)) + m2*((-1)*theta0Dot*(r0y*cos(theta0) + r0x*sin(theta0)) + (-1)*(l1 + r1)*(theta1Dot + theta0Dot)*sin(theta01 + theta1 + theta0) + (-1)*l2*(theta1Dot + theta2Dot + theta0Dot)*sin(theta01 + theta1 + theta2 + theta0))),

		// 		theta1Dot*r0y*(l1*(m1 + m2) + m2*r1)*(theta1Dot + 2*theta0Dot)*cos(theta01 + theta1) + l2*m2*(theta1Dot + theta2Dot)*r0y*(theta1Dot + theta2Dot + 2*theta0Dot)*cos(theta01 + theta1 + theta2) + (-1)*l1*m1*pow(theta1Dot,2)*r0x*sin(theta01 + theta1) + (-1)*l1*m2*pow(theta1Dot,2)*r0x*sin(theta01 + theta1) + (-1)*m2*pow(theta1Dot,2)*r0x*r1*sin(theta01 + theta1) + (-2)*l1*m1*theta1Dot*r0x*theta0Dot*sin(theta01 + theta1) + (-2)*l1*m2*theta1Dot*r0x*theta0Dot*sin(theta01 + theta1) + (-2)*m2*theta1Dot*r0x*r1*theta0Dot*sin(theta01 + theta1) + (-2)*l1*l2*m2*theta1Dot*theta2Dot*sin(theta2) + (-1)*l1*l2*m2*pow(theta2Dot,2)*sin(theta2) + (-2)*l2*m2*theta1Dot*theta2Dot*r1*sin(theta2) + (-1)*l2*m2*pow(theta2Dot,2)*r1*sin(theta2) + (-2)*l1*l2*m2*theta2Dot*theta0Dot*sin(theta2) + (-2)*l2*m2*theta2Dot*r1*theta0Dot*sin(theta2) + (-1)*l2*m2*pow(theta1Dot,2)*r0x*sin(theta01 + theta1 + theta2) + (-2)*l2*m2*theta1Dot*theta2Dot*r0x*sin(theta01 + theta1 + theta2) + (-1)*l2*m2*pow(theta2Dot,2)*r0x*sin(theta01 + theta1 + theta2) + (-2)*l2*m2*theta1Dot*r0x*theta0Dot*sin(theta01 + theta1 + theta2) + (-2)*l2*m2*theta2Dot*r0x*theta0Dot*sin(theta01 + theta1 + theta2),

		// 		(-1)*r0y*(l1*(m1 + m2) + m2*r1)*pow(theta0Dot,2)*cos(theta01 + theta1) + (-1)*l2*m2*r0y*pow(theta0Dot,2)*cos(theta01 + theta1 + theta2) + l1*m1*r0x*pow(theta0Dot,2)*sin(theta01 + theta1) + l1*m2*r0x*pow(theta0Dot,2)*sin(theta01 + theta1) + m2*r0x*r1*pow(theta0Dot,2)*sin(theta01 + theta1) + (-2)*l1*l2*m2*theta1Dot*theta2Dot*sin(theta2) + (-1)*l1*l2*m2*pow(theta2Dot,2)*sin(theta2) + (-2)*l2*m2*theta1Dot*theta2Dot*r1*sin(theta2) + (-1)*l2*m2*pow(theta2Dot,2)*r1*sin(theta2) + (-2)*l1*l2*m2*theta2Dot*theta0Dot*sin(theta2) + (-2)*l2*m2*theta2Dot*r1*theta0Dot*sin(theta2) + l2*m2*r0x*pow(theta0Dot,2)*sin(theta01 + theta1 + theta2),

		// 		(-1)*l2*m2*(r0y*pow(theta0Dot,2)*cos(theta01 + theta1 + theta2) + (-1)*(l1 + r1)*pow((theta1Dot + theta0Dot),2)*sin(theta2) + (-1)*r0x*pow(theta0Dot,2)*sin(theta01 + theta1 + theta2));


		// // cout << "c Vector:" << endl << c << endl << endl;
		MatrixXd H1(5,2);
		H1 << 	H(0,0), H(0,1),
				H(1,0), H(1,1),
				H(2,0), H(2,1),
				H(3,0), H(3,1),
				H(4,0), H(4,1);

		VectorXd H2(5);
		H2 << 	H(0,2),
				H(1,2),
				H(2,2),
				H(3,2),
				H(4,2);

		MatrixXd H3(5,2);
		H3 << 	H(0,3), H(0,4),
				H(1,3), H(1,4),
				H(2,3), H(2,4),
				H(3,3), H(3,4),
				H(4,3), H(4,4);

		Matrix2d R0;
		R0 << 	cos(theta0), -sin(theta0),
				sin(theta0), cos(theta0);

		Matrix2d R0_trans;
		R0_trans = R0.transpose();

		Vector2d acc_inert;
		acc_inert << acc_x, acc_y;
		
		Vector2d acc_base;
		acc_base = R0*acc_inert;

		imu_vel_x = imu_vel_x_prev + (acc_base(0) * time_step);
		imu_vel_y = imu_vel_y_prev + (acc_base(1) * time_step);

		angular_vel_z_prev = angular_vel_z;

		imu_vel_x_prev = imu_vel_x;
		imu_vel_y_prev = imu_vel_y;


		Vector2d rp0;
		rp0 = R0*rp00;

		double rp0x = rp0(0);
		double rp0y = rp0(1);

		Matrix2d omega0cross;
		omega0cross << 	0, -theta0Dot,
						theta0Dot, 0;

		Vector2d rp0Dot;
		rp0Dot = omega0cross*rp0;

		double rp0Dotx = rp0Dot(0);
		double rp0Doty = rp0Dot(1);

		Vector2d rp0cross;
		rp0cross << rp0x, -rp0y;

		Vector2d rp0Dotcross;
		rp0Dotcross << rp0Dotx, -rp0Doty;

		MatrixXd H1star(5,2);
		VectorXd H2star(5);
		MatrixXd H3star(5,2);

		H1star = H1;
		H2star = H2 + H1*rp0cross;
		H3star = H3;


		MatrixXd Hstar(5,5);
		Hstar << H1star, H2star, H3star;

		// VectorXd auxVector(5);
		// auxVector = H1*rp0Dotcross;

		// VectorXd cstar(5);
		// cstar = c + auxVector*theta0Dot;

		VectorXd e(5);
		e = qstar_des - qstar;

		VectorXd eDot(5);
		eDot = qstarDot_des - qstarDot;

		VectorXd u(5);
		// u = qstarDotDot_des + KD*eDot + KP*e;
		u = KD*eDot + KP*e;
		// cout << "e Vector:" << endl << e << endl << endl;
		// cout << "eDot Vector:" << endl << eDot << endl << endl;
		// cout << "u Vector:" << endl << u << endl << endl;
		// cout << "Hstar Matrix:" << endl << Hstar << endl << endl;
		// cout << "cstar Vector:" << endl << cstar << endl << endl;

		VectorXd Q(5);
		// Q = Hstar*u;// + cstar;
		Q = u;

		// VectorXd hstaru(5);
		// hstaru = Hstar*u;

		// cout << "Q Vector:" << endl << Q << endl << endl;
		// cout << "---------------------" << endl << endl;
		// torq[0] = - Q(3);
		// torq[1] = Q(4);
		error_qdot[0] = theta1Dot_des - ls_velocity;
		error_qdot[1] = theta2Dot_des - le_velocity;
		errorq[0] = theta1_des - ls_position;
		errorq[1] = theta2_des - le_position;

		torq[0] = - (Kp_ls * errorq[0] + Kd_ls * error_qdot[0]);
		torq[1] = Kp_le * errorq[1] + Kd_le * error_qdot[1];

		// cout << torq[0] << " " << torq[1] << endl;
		// cout<<hstaru(3) << " " << hstaru(4)<<endl;
		// cout<<cstar(3) << " " << cstar(4)<<endl;
		// cout<<"------------"<<endl;

		MatrixXd Dpinv(4,3);
		Dpinv << 0.5774, 0.3333, -0.1405,
				0.0000, -0.6667, -0.1405,
				0.5774, -0.3333, 0.1405,
				-0.0000, 0.0000, 0.9368;

		Vector2d Q2;
		Q2 << Q(0), Q(1);

		Vector2d q2inv;
		q2inv = R0_trans * Q2;

		Vector3d qvect;
		qvect << q2inv(0), q2inv(1), Q(2);

		Vector4d Factuators;
		Factuators = Dpinv * qvect;

		torq[3] = Factuators(3);
		// cout << "0: " << Factuators(0) << " 1: " << Factuators(1) << " 2: " << Factuators(2) << endl;
		thrust_vector.vector.x = Factuators(0);
		thrust_vector.vector.y = Factuators(1);
		thrust_vector.vector.z = Factuators(2);

		if (t > tf ) {
			thrust_vector.vector.x = 0.0;
			thrust_vector.vector.y = 0.0;
			thrust_vector.vector.z = 0.0;
			torq[3] = 0.0;
			cout<<"finished"<<endl;
		}


		prev_secs = secs;
		_secs.data = secs;
		secs_pub.publish(_secs);
		torque.data = filter_torque(torq[0], prev_torq[0]);
		// torque.data = torq[0];
		ls_torque_pub.publish(torque);
		ls_trq_pub.publish(torque);
		torque.data = filter_torque(torq[1], prev_torq[1]);
		// torque.data = torq[1];
		le_torque_pub.publish(torque);
		le_trq_pub.publish(torque);

		torque.data = torq[3];
		rw_torque_pub.publish(torque);
		rw_trq_pub.publish(torque);

		temp_msg.data = qvect(0);
		Fx_base_pub.publish(temp_msg);
		temp_msg.data = qvect(1);
		Fy_base_pub.publish(temp_msg);
		temp_msg.data = qvect(2);
		Tz_base_pub.publish(temp_msg);
		temp_msg.data = Factuators(0);
		thrust_x_pub.publish(temp_msg);
		temp_msg.data = Factuators(1);
		thrust_y_pub.publish(temp_msg);
		temp_msg.data = Factuators(2);
		thrust_z_pub.publish(temp_msg);
		
		thrust_pub.publish(thrust_vector);

		temp_msg.data = rw_position;
		rw_pos_pub.publish(temp_msg);
		temp_msg.data = rw_velocity;
		rw_vel_pub.publish(temp_msg);

		temp_msg.data = x_value;
		x_value_pub.publish(temp_msg);
		temp_msg.data = y_value;
		y_value_pub.publish(temp_msg);
		temp_msg.data = z_value;
		z_value_pub.publish(temp_msg);
		temp_msg.data = w_value;
		w_value_pub.publish(temp_msg);

		// temp_msg.data = rw_vel;
		// rw_vell_pub.publish(temp_msg);

		temp_msg.data = ls_position;
		ls_pos_pub.publish(temp_msg);
		temp_msg.data = le_position;
		le_pos_pub.publish(temp_msg);
	
		temp_msg.data = ls_velocity;
		ls_vel_pub.publish(temp_msg);
		temp_msg.data = le_velocity;
		le_vel_pub.publish(temp_msg);

		temp_msg.data = theta1_des;
		qe_des_x_pub.publish(temp_msg);
		temp_msg.data = theta2_des;
		qe_des_y_pub.publish(temp_msg);
		temp_msg.data = theta1Dot_des;
		qe_dot_des_x_pub.publish(temp_msg);
		temp_msg.data = theta2Dot_des;
		qe_dot_des_y_pub.publish(temp_msg);
		// temp_msg.data = theta1DotDot_des;
		// qe_dotdot_des_x_pub.publish(temp_msg);
		// temp_msg.data = theta2DotDot_des;
		// qe_dotdot_des_y_pub.publish(temp_msg);

		temp_msg.data = baseposx_des;
		baseposx_des_pub.publish(temp_msg);
		temp_msg.data = baseposy_des;
		baseposy_des_pub.publish(temp_msg);
		temp_msg.data = theta0_des;
		theta0_des_pub.publish(temp_msg);
		temp_msg.data = baseposxDot_des;
		baseposxDot_des_pub.publish(temp_msg);
		temp_msg.data = baseposyDot_des;
		baseposyDot_des_pub.publish(temp_msg);
		temp_msg.data = theta0Dot_des;
		theta0Dot_des_pub.publish(temp_msg);

		temp_msg.data = angular_vel_z;
		imu_angular_vel_pub.publish(temp_msg);
		temp_msg.data = ps_theta;
		ps_theta_pub.publish(temp_msg);
		temp_msg.data = ps_theta_dot;
		ps_theta_dot_pub.publish(temp_msg);

		temp_msg.data = ps_x;
		cepheus_x_pub.publish(temp_msg);
		temp_msg.data = ps_y;
		cepheus_y_pub.publish(temp_msg);
		temp_msg.data = ps_xdot;
		cepheus_xdot_pub.publish(temp_msg);
		temp_msg.data = ps_ydot;
		cepheus_ydot_pub.publish(temp_msg);

		temp_msg.data = imu_vel_x;
		imu_vel_x_pub.publish(temp_msg);
		temp_msg.data = imu_vel_y;
		imu_vel_y_pub.publish(temp_msg);
		temp_msg.data = acc_x;
		imu_acc_x_pub.publish(temp_msg);
		temp_msg.data = acc_y;
		imu_acc_y_pub.publish(temp_msg);

		ps_theta_prev = ps_theta;
		rw_position_prev = rw_position;
		ps_theta_dot_prev = ps_theta_dot;

		main_queue.callAvailable(ros::WallDuration(0.0));
		loop_rate.sleep();
	}

	return 0;
}
