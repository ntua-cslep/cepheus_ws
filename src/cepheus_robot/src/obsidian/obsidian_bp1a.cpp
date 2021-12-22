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

double ls_position = 0.0, le_position = 0.0, re_position = 0.0, rw_position = 0.0;
double ls_velocity = 0.0, le_velocity = 0.0, re_velocity = 0.0, rw_velocity = 0.0;
double x_value = 0.0, y_value = 0.0, z_value = 0.0, w_value = 0.0;
bool first_time_movement = true;
bool first_time_ls = true;
bool first_time_le = true;
bool first_time_re = true;

double ps_x = 0.0, ps_y = 0.0, ps_theta = 0.0, ps_theta_prev = 0.0, rw_position_prev = 0.0;
double angular_vel_z = 0.0, angular_vel_z_offset = 0.0;

// void imuAccelerationCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
// 	geometry_msgs::Vector3Stamped temp;
// 	temp = *msg;
// 	acc_x = temp.vector.x - acc_x_offset;
// 	acc_y = temp.vector.y - acc_y_offset;
// }


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
	// cout<<"ps_theta "<<ps_theta<<endl;
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
	// ros::Subscriber imu_acc_sub = nh.subscribe("/imu/acceleration", 1, imuAccelerationCallback);
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
	angular_vel_z_offset = imu_offsets["angular_vel_z_offset"].as<double>();

	ros::Time curr_time, t_beg = ros::Time::now();
	ros::Duration all_time;
	double prev_secs = 0.0;
	double t = 0.0, time = 0.0, move_time = 0.0, tminus = 0.0;
	double tf = 20.0;
	double wf = (2 * M_PI)/tf;
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
	double A1 = 0.38/wf; //0.64 max
	double A2 = 0.14/wf; //0.36 max

	double theta1init = 0.0;
	double theta2init = 0.0;
	double theta1fin = 0.0;
	double theta2fin = 0.0;
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

	double theta1_des = 0.0;
	double theta1Dot_des = 0.0;
	double theta1DotDot_des = 0.0;
	double theta2_des = 0.0;
	double theta2Dot_des = 0.0;
	double theta2DotDot_des = 0.0;

	VectorXd x(12);
	VectorXd xx(12);

	double Kp_ls = 0.09644245;
	double Kp_le = 0.01127526;
	double Kd_ls = 0.009644245;
	double Kd_le = 0.001127526;

	double errorq[4];
	double error_qdot[4];
	double torq[4];
	double prev_torq[4];
	double qd[4];
	double qd_dot[4];
	double ps_theta_dot = 0.0;
	double ps_theta_dot_prev = 0.0;
	double rw_vel = 0.0;

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

	while (ls_position == 0.0 || le_position == 0.0 || ps_theta == 0.0 || angular_vel_z == 0.0) {
	 
	
		cout << "ps_theta " << ps_theta << " ls_position " << ls_position << " angular_vel_z " << angular_vel_z << endl;
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
		rw_vel = (rw_position - rw_position_prev) / time_step;
		// if (abs(ps_theta_dot - ps_theta_dot_prev) > 0.1)
		// 	ps_theta_dot = ps_theta_dot / 10;

		if (first_time_movement) {
			move_time = secs;

			x <<	-0.0502531230821664, -0.00963668816229735, -0.0313752998135336, 0.0226123153397242,
					0.11837350042829, -0.122707402889061, -0.0643085499244572, -0.0153700294670522,
					0.0664011498205846, 0.0476593441567345, 0.0887493654713886, 0.00800266207171;
			xx = 1.0 * x;

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

			theta1init = 0;
			theta2init = 0;
			theta1fin = 0;
			theta2fin = 0;

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
			first_time_movement = false;
		}

		t = secs - move_time;

		if ((t > 30) && (t <= tf + 30)) {
			tminus = t - 30;
		// 	// cout << tminus << endl;
			theta1_des = a11/wf*sin(wf*tminus)-b11/wf*cos(wf*tminus)+a21/wf/2*sin(2*wf*tminus)-b21/wf/2*cos(2*wf*tminus) +a31/wf/3*sin(3*wf*tminus)-b31/wf/3*cos(3*wf*tminus)+c01+c11*tminus+c21*pow(tminus, 2)+c31*pow(tminus, 3)+c41*pow(tminus, 4)+c51*pow(tminus, 5);
			theta1Dot_des = a11*cos(wf*tminus)+b11*sin(wf*tminus)+a21*cos(2*wf*tminus)+b21*sin(2*wf*tminus) +a31*cos(3*wf*tminus)+b31*sin(3*wf*tminus)+c11+2*c21*tminus+3*c31*pow(tminus, 2)+4*c41*pow(tminus, 3)+5*c51*pow(tminus, 4);
			theta1DotDot_des = -a11*wf*sin(wf*tminus)+b11*wf*cos(wf*tminus)+a21*(-2*wf)*sin(2*wf*tminus)+b21*2*wf*cos(2*wf*tminus) +a31*(-3*wf)*sin(3*wf*tminus)+b31*3*wf*cos(3*wf*tminus)+2*c21+3*2*c31*tminus+4*3*c41*pow(tminus, 2)+5*4*c51*pow(tminus, 3);
			theta2_des = a12/wf*sin(wf*tminus)-b12/wf*cos(wf*tminus)+a22/wf/2*sin(2*wf*tminus)-b22/wf/2*cos(2*wf*tminus) +a32/wf/3*sin(3*wf*tminus)-b32/wf/3*cos(3*wf*tminus)+c02+c12*tminus+c22*pow(tminus, 2)+c32*pow(tminus, 3)+c42*pow(tminus, 4)+c52*pow(tminus, 5);
			theta2Dot_des = a12*cos(wf*tminus)+b12*sin(wf*tminus)+a22*cos(2*wf*tminus)+b22*sin(2*wf*tminus) +a32*cos(3*wf*tminus)+b32*sin(3*wf*tminus)+c12+2*c22*tminus+3*c32*pow(tminus, 2)+4*c42*pow(tminus, 3)+5*c52*pow(tminus, 4);
			theta2DotDot_des = -a12*wf*sin(wf*tminus)+b12*wf*cos(wf*tminus)+a22*(-2*wf)*sin(2*wf*tminus)+b22*2*wf*cos(2*wf*tminus) +a32*(-3*wf)*sin(3*wf*tminus)+b32*3*wf*cos(3*wf*tminus)+2*c22+3*2*c32*tminus+4*3*c42*pow(tminus, 2)+5*4*c52*pow(tminus, 3);
		}
	    else if ((t >= 50) && (t <= 70)){
			tminus = t - 50;
			theta1_des = A1 * sin(wf*tminus); 
			theta1Dot_des = A1 * wf * cos(wf*tminus);
			theta2_des = A2 * sin(wf*tminus);
			theta2Dot_des = A2 * wf * cos(wf*tminus);
		}

		if (t <= 30) {
			error_qdot[0] = 0 - ls_velocity;
			error_qdot[1] = 0 - le_velocity;
			errorq[0] = 0.0 - ls_position;
			errorq[1] = 0.0 - le_position;

			torq[0] = - (Kp_ls * errorq[0] + Kd_ls * error_qdot[0]);
			torq[1] = Kp_le * errorq[1] + Kd_le * error_qdot[1];

		} else {
			error_qdot[0] = theta1Dot_des - ls_velocity;
			error_qdot[1] = theta2Dot_des - le_velocity;
			errorq[0] = theta1_des - ls_position;
			errorq[1] = theta2_des - le_position;

			torq[0] = - (Kp_ls * errorq[0] + Kd_ls * error_qdot[0]);
			torq[1] = Kp_le * errorq[1] + Kd_le * error_qdot[1];
		}
		// if (t < 50) { 
		// 	error_qdot[3] = 150 - rw_velocity;
		// }
		// else{
		// 	error_qdot[3] = 80 - rw_velocity;
		// }
		error_qdot[3] = 150 - rw_vel;

		errorq[3] = error_qdot[3] * (time_step) + errorq[3];
		torq[3] = - (0.00778 * error_qdot[3] + 0.00078 * errorq[3]);

		// cout << torq[0] << " " << torq[1] << " " << torq[3] << endl;

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

		// thrust_vector.vector.x = 0.0;
		// thrust_vector.vector.y = 0.0;
		// thrust_vector.vector.z = 0.1;
		// thrust_pub.publish(thrust_vector);

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

		temp_msg.data = rw_vel;
		rw_vell_pub.publish(temp_msg);

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
		temp_msg.data = theta1DotDot_des;
		qe_dotdot_des_x_pub.publish(temp_msg);
		temp_msg.data = theta2DotDot_des;
		qe_dotdot_des_y_pub.publish(temp_msg);

		temp_msg.data = angular_vel_z;
		imu_angular_vel_pub.publish(temp_msg);
		temp_msg.data = ps_theta;
		ps_theta_pub.publish(temp_msg);
		temp_msg.data = ps_theta_dot;
		ps_theta_dot_pub.publish(temp_msg);

		ps_theta_prev = ps_theta;
		rw_position_prev = rw_position;
		ps_theta_dot_prev = ps_theta_dot;

		main_queue.callAvailable(ros::WallDuration(0.0));
		loop_rate.sleep();
	}

	return 0;
}
