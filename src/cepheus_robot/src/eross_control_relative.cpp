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

#include "digital_filter.h"

#define POS_FILTER 0.005
#define VEL_FILTER 0.05
#define TORQUE_LIMIT 0.00000001

using namespace Eigen;
using namespace std;



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

enum BotaSysEnumeration{
	X = 0,
	Y,
	Z
};

double ls_position = 0.0, le_position = 0.0, re_position = 0.0, rw_position = 0.0;
double ls_velocity = 0.0, le_velocity = 0.0, re_velocity = 0.0, rw_velocity = 0.0;
bool first_time_movement = true;
bool start_docking = false;
bool first_time_before_docking = true; 
bool first_time_docking = true;
bool first_time_penetrating = true;
bool first_time_after_docking = true;
bool first_time_ls = true;
bool first_time_le = true;
bool first_time_re = true;

double botarm_force_x;
double botarm_force_z;
double botarm_torque_y;

double ps_x[3];
double ps_x_prev[3];
double ps_x_prev_prev[3];
double ps_y[3];
double ps_y_prev[3];
double ps_y_prev_prev[3];
double ps_th[3];
double ps_th_prev[3];
double ps_th_prev_prev[3];

double force_x = 0.0;
double force_y = 0.0;
double force_z = 0.0;
double torque_x = 0.0;
double torque_y = 0.0;
double torque_z = 0.0;
double prev_force_x = 0.0;
double prev_force_y = 0.0;
double prev_force_z = 0.0;
double prev_torque_x = 0.0;
double prev_torque_y = 0.0;
double prev_torque_z = 0.0;


double acc_x = 0.0, acc_y = 0.0;
double acc_x_offset = 0.0, acc_y_offset = 0.0;
double acc_x_prev = 0.0, acc_y_prev = 0.0;
double angular_vel_z = 0.0;
double angular_vel_z_offset = 0.0;


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


// in bota queue that runs in higher rate
double num_of_avail_rokubimini_readings;

double sum_of_forces[3];
double sum_of_torques[3];

void rokubiminiCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
	num_of_avail_rokubimini_readings++;
	geometry_msgs::WrenchStamped temp;
	temp = *msg;
	sum_of_forces[X] += temp.wrench.force.x;
	sum_of_forces[Y] -= temp.wrench.force.y;
	sum_of_forces[Z] += temp.wrench.force.z;
	sum_of_torques[X] -= temp.wrench.torque.x;
	sum_of_torques[Y] += temp.wrench.torque.y;
	sum_of_torques[Z] += temp.wrench.torque.z;
	// force_x = sum_of_forces[X] / num_of_avail_rokubimini_readings;
    // cout << num_of_avail_rokubimini_readings << endl;
}

void botarmCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
	geometry_msgs::WrenchStamped temp;
	temp = *msg;
	botarm_force_x = temp.wrench.force.x;
	botarm_force_z = temp.wrench.force.z;
	botarm_torque_y = temp.wrench.torque.y;
}


// hard pass 10 first values to digital filters
int ps_counter = 0; 

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
	// ps_x[ALX_GRIPPER] = f_x1->filter(temp.transform.translation.x);
	// ps_y[ALX_GRIPPER] = f_y1->filter(temp.transform.translation.y);
	// ps_th[ALX_GRIPPER] = f_z1->filter(yaw);
	// if (ps_counter < 25) {
		ps_x[ALX_GRIPPER] = temp.transform.translation.x;
		ps_y[ALX_GRIPPER] = temp.transform.translation.y;
		ps_th[ALX_GRIPPER] = yaw;
	// }
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
	// ps_x[ASSIST] = f_x2->filter(temp.transform.translation.x);
	// ps_y[ASSIST] = f_y2->filter(temp.transform.translation.y);
	// ps_th[ASSIST] = f_z2->filter(yaw);
	// if (ps_counter < 25) {
		ps_x[ASSIST] = temp.transform.translation.x;
		ps_y[ASSIST] = temp.transform.translation.y;
		ps_th[ASSIST] = yaw;
	// }
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
	// ps_x[CEPHEUS] = f_x3->filter(temp.transform.translation.x);
	// ps_y[CEPHEUS] = f_y3->filter(temp.transform.translation.y);
	// ps_th[CEPHEUS] = f_z3->filter(yaw);
	// if (ps_counter < 25) {
		ps_x[CEPHEUS] = temp.transform.translation.x;
		ps_y[CEPHEUS] = temp.transform.translation.y;
		ps_th[CEPHEUS] = yaw;
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


void rePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (first_time_re) {
		re_position = cmd->data;
		first_time_re = false;
		return;
	}
	if (abs(cmd->data - re_position) > POS_FILTER)
		return;
	else
		re_position = cmd->data;
}


void rwPosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	rw_position = cmd->data;
}


void lsVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	// if (abs(cmd->data - ls_velocity) > VEL_FILTER)
	// 	return;
	// else
		ls_velocity = cmd->data;
}


void leVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	// if (abs(cmd->data - le_velocity) > VEL_FILTER)
	// 	return;
	// else
		le_velocity = cmd->data;
}


void reVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	// if (abs(cmd->data - re_velocity) > VEL_FILTER)
	// 	return;
	// else
		re_velocity = cmd->data;
}


void rwVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	// if (abs(cmd->data - rw_velocity) > VEL_FILTER)
	// 	return;
	// else
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


// ros::Publisher lar_pub;
// std_msgs::String lar_command;

ros::Publisher reset_movement_pub;
std_msgs::Bool reset_movement;

sig_atomic_t volatile g_request_shutdown = 0;
// Replacement SIGINT handler
void ctrl_c_handler(int sig) {
	// lar_command.data = "disable";
	// lar_pub.publish(lar_command);
	// sleep(0.1);
	// lar_pub.publish(lar_command);
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
	// node handler for rokubimini callback
	ros::NodeHandle nh_rokubimini;
	ros::CallbackQueue rokubimini_queue;
	nh_rokubimini.setCallbackQueue(&rokubimini_queue);

	f_x1 = new DigitalFilter(10, 0.0);
	f_x2 = new DigitalFilter(10, 0.0);
	f_x3 = new DigitalFilter(10, 0.0);
	f_y1 = new DigitalFilter(10, 0.0);
	f_y2 = new DigitalFilter(10, 0.0);
	f_y3 = new DigitalFilter(10, 0.0);
	f_z1 = new DigitalFilter(10, 0.0);
	f_z2 = new DigitalFilter(10, 0.0);
	f_z3 = new DigitalFilter(10, 0.0);
	
	ros::Subscriber ps_alx_sub =  nh.subscribe("map_to_alxgripper", 1, PSAlxCallback);
	ros::Subscriber ps_assist_sub =  nh.subscribe("map_to_assist_robot", 1, PSAssistCallback);
	ros::Subscriber ps_cepheus_sub =  nh.subscribe("map_to_cepheus", 1, PSCepheusCallback);

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
	// ros::Publisher gripper_x_desdot_pub = nh.advertise<std_msgs::Float64>("g_x_desdot", 1);
	// ros::Publisher cepheus_x_pub = nh.advertise<std_msgs::Float64>("c_x", 1);
	// ros::Publisher cepheus_y_pub = nh.advertise<std_msgs::Float64>("c_y", 1);
	ros::Publisher cepheus_th_pub = nh.advertise<std_msgs::Float64>("c_th", 1);
	ros::Publisher cepheus_th_des_pub = nh.advertise<std_msgs::Float64>("c_th_des", 1);
	ros::Publisher cepheus_th_dot_pub = nh.advertise<std_msgs::Float64>("c_th_dot", 1);
	// ros::Publisher gripper_error_x_pub = nh.advertiYAML::Node config = YAML::LoadFile("config.yaml");se<std_msgs::Float64>("g_error_x", 1);
	// ros::Publisher gripper_error_y_pub = nh.advertise<std_msgs::Float64>("g_error_y", 1);
	// ros::Publisher gripper_error_th_pub = nh.advertise<std_msgs::Float64>("g_error_th", 1);
	ros::Publisher qe_des_x_pub = nh.advertise<std_msgs::Float64>("qe_des_x", 1);
	ros::Publisher qe_des_y_pub = nh.advertise<std_msgs::Float64>("qe_des_y", 1);
	ros::Publisher qe_des_th_pub = nh.advertise<std_msgs::Float64>("qe_des_th", 1);
	ros::Publisher qe_dot_des_x_pub = nh.advertise<std_msgs::Float64>("qe_dot_des_x", 1);
	ros::Publisher qe_dot_des_y_pub = nh.advertise<std_msgs::Float64>("qe_dot_des_y", 1);
	ros::Publisher qe_dot_des_th_pub = nh.advertise<std_msgs::Float64>("qe_dot_des_th", 1);
	ros::Publisher qe_dotdot_des_x_pub = nh.advertise<std_msgs::Float64>("qe_dotdot_des_x", 1);
	ros::Publisher qe_dotdot_des_y_pub = nh.advertise<std_msgs::Float64>("qe_dotdot_des_y", 1);
	ros::Publisher qe_dotdot_des_th_pub = nh.advertise<std_msgs::Float64>("qe_dotdot_des_th", 1);
	// ros::Publisher errordot_th_pub = nh.advertise<std_msgs::Float64>("errordot_th", 1);
	// ros::Publisher error_th_pub = nh.advertise<std_msgs::Float64>("error_th", 1);
	ros::Publisher det_pub = nh.advertise<std_msgs::Float64>("det", 1);
	ros::Publisher det_h_pub = nh.advertise<std_msgs::Float64>("det_h", 1);
	ros::Publisher fextx_pub = nh.advertise<std_msgs::Float64>("fextx", 1);
	ros::Publisher fexty_pub = nh.advertise<std_msgs::Float64>("fexty", 1);
	ros::Publisher fextz_pub = nh.advertise<std_msgs::Float64>("fextz", 1);
	ros::Publisher next_pub = nh.advertise<std_msgs::Float64>("next", 1);
	ros::Publisher nexty_pub = nh.advertise<std_msgs::Float64>("nexty", 1);
	ros::Publisher nextx_pub = nh.advertise<std_msgs::Float64>("nextx", 1);
	ros::Publisher uRWn_pub = nh.advertise<std_msgs::Float64>("uRWn", 1);  
	ros::Publisher umr_x_pub = nh.advertise<std_msgs::Float64>("umrX", 1); 
	ros::Publisher umr_y_pub = nh.advertise<std_msgs::Float64>("umrY", 1); 
	ros::Publisher umwn_pub = nh.advertise<std_msgs::Float64>("umwn", 1); 
	// lar_pub = nh.advertise<std_msgs::String>("gripperCommand", 1);
	ros::Publisher secs_pub = nh.advertise<std_msgs::Float64>("secs", 1);

	ros::Publisher botarm_force_x_pub = nh.advertise<std_msgs::Float64>("botarm_force_x", 1);
	ros::Publisher botarm_force_z_pub = nh.advertise<std_msgs::Float64>("botarm_force_z", 1);
	ros::Publisher botarm_torque_y_pub = nh.advertise<std_msgs::Float64>("botarm_torque_y", 1);

	ros::Publisher error_0_pub = nh.advertise<std_msgs::Float64>("error_0", 1);
	ros::Publisher error_1_pub = nh.advertise<std_msgs::Float64>("error_1", 1);
	ros::Publisher error_2_pub = nh.advertise<std_msgs::Float64>("error_2", 1);
	ros::Publisher error_3_pub = nh.advertise<std_msgs::Float64>("error_3", 1);
	ros::Publisher error_dot_0_pub = nh.advertise<std_msgs::Float64>("error_dot_0", 1);
	ros::Publisher error_dot_1_pub = nh.advertise<std_msgs::Float64>("error_dot_1", 1);
	ros::Publisher error_dot_2_pub = nh.advertise<std_msgs::Float64>("error_dot_2", 1);
	ros::Publisher error_dot_3_pub = nh.advertise<std_msgs::Float64>("error_dot_3", 1);

	ros::Subscriber ls_pos_sub = nh.subscribe("read_left_shoulder_position", 1, lsPosCallback);
	ros::Subscriber le_pos_sub = nh.subscribe("read_left_elbow_position", 1, lePosCallback);
	ros::Subscriber re_pos_sub = nh.subscribe("read_right_elbow_position", 1, rePosCallback);
	ros::Subscriber rw_pos_sub = nh.subscribe("read_reaction_wheel_position", 1, rwPosCallback);
	ros::Subscriber ls_vel_sub = nh.subscribe("read_left_shoulder_velocity", 1, lsVelCallback);
	ros::Subscriber le_vel_sub = nh.subscribe("read_left_elbow_velocity", 1, leVelCallback);
	ros::Subscriber re_vel_sub = nh.subscribe("read_right_elbow_velocity", 1, reVelCallback);
	ros::Subscriber rw_vel_sub = nh.subscribe("read_reaction_wheel_velocity", 1, rwVelCallback);

	ros::Subscriber start_docking_sub = nh.subscribe("start_docking", 1, startDockingCallback);
	// ros::Subscriber reset_docking_sub = nh.subscribe("reset_docking", 1, resetDockingCallback);

	ros::Subscriber rokubimini_sub = nh_rokubimini.subscribe("/rokubimini/ft_sensor0/ft_sensor_readings/wrench", 10, rokubiminiCallback);
	ros::Subscriber botarm_sub = nh.subscribe("filtered_botasys", 1, botarmCallback);

	ros::Subscriber imu_acc_sub = nh.subscribe("/imu/acceleration", 1, imuAccelerationCallback);
	ros::Subscriber imu_ang_vel_sub = nh.subscribe("/imu/angular_velocity", 1, imuAngularVelCallback);

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

	ros::Time curr_time, t_beg = ros::Time::now();
	ros::Duration all_time;
	double prev_secs = 0.0;


    std_msgs::Bool start_moving;
	start_moving.data = true;

	std_msgs::Float64 torque;
	std_msgs::Float64 offset;
	std_msgs::Float64 temp_msg;
	std_msgs::Float64 _secs;

/////////////////////////////////////////////////////////////////  DEFINING PARAMETERS  ///////////////////////////////////////////////////////////

	double s, sdot, sdotdot;
	double s1, s1dot, s1dotdot;
	double time_step = 0.0, time_step_prev = 0.0;
	double secs, move_time, docking_time, penetration_time, prin;
	double xE_in, yE_in, thE_in, xE_fin, yE_fin, thE_fin, th0_fin;
	double th0_in;
	double s_0, s_f, s0dot, sfdot, s0dotdot, sfdotdot;
	double a0, a1, a2, a3, a4, a5;
	double a10, a11, a12, a13, a14, a15;
	double xE, yE, thE, xEdot, yEdot, thEdot, theta0dotdot_des, xEdotdot, yEdotdot, thEdotdot, theta0dot_des, theta0_des;
	double m0 = 53.53;
	double m1 = 0.4409;
	double m2 = 0.1304;
	double m3 = 30.0;
	double M = m0 + m1 + m2 + m3;
	double r0x = 0.1628;
	double r0y = 0.0;
	double r1 = 0.1010;
	double l1 = 0.2690;
	double r2 = 0.1430;
	double l2 = 0.1430;
	double r3 = 0.467;  //0.467 run with 0.1865
	double l3 = 0.313;  //0.313 run with 0.0885
	double I0z = 2.2491;
	double I1z = 0.0068;
	double I2z = 0.0010;
	double I3z = 1.100;
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
	double FextX, FextY, Next;
	double calibration_x, calibration_y, calibration_z, calibration_tx, calibration_ty, calibration_tz;
	double xE_in_abs, yE_in_abs, xE_fin_abs, yE_fin_abs;  
	double h;
	double g_vel_x, g_vel_y, g_vel_th, c_vel_x, c_vel_y, c_vel_th, g_vel_x_prev, g_vel_y_prev, g_vel_th_prev, c_vel_x_prev, c_vel_y_prev, c_vel_th_prev; 
	MatrixXd eye_3;
	eye_3.setIdentity(3, 3);
	MatrixXd eye_2;
	eye_2.setIdentity(2, 2);
	Matrix3d KD = eye_3 * 1.6;
	Matrix3d KP = eye_3 * 0.8;
	Vector3d qE;
	Vector3d qEDot;
	Vector3d ve;
	Vector3d vedot;
	double t0 = 0.0, tf = 50.0, tf1 = 40.0, time = 0.0;
	Matrix2d HPS = eye_2;
	Matrix2d DPS = eye_2 * 0.01;
	Matrix2d KPS = eye_2 * 1;
	Matrix3d HA = eye_3;
	Matrix3d DA = eye_3 * 0.01;
	Matrix3d KA = eye_3 * 1;
	Vector3d ve_des;
	Vector3d vedot_des;
	MatrixXd G(6, 6);
	MatrixXd G1(6, 6);
	VectorXd B1(6);
	VectorXd x1(6);
	VectorXd x11(6);
	MatrixXd Ginv(6, 6);
	MatrixXd G1inv(6, 6);

	// lar_command.data = "enable";
	// lar_pub.publish(lar_command);
	// sleep(1);
	// lar_pub.publish(lar_command);
    string yaml_path = ros::package::getPath("cepheus_robot");
    yaml_path.append("/config/imu_offsets.yaml");
	YAML::Node imu_offsets = YAML::LoadFile(yaml_path);
	acc_x_offset = imu_offsets["acc_x_offset"].as<double>();
	acc_y_offset = imu_offsets["acc_y_offset"].as<double>();
	angular_vel_z_offset = imu_offsets["angular_vel_z_offset"].as<double>();

	while (
		ps_th[CEPHEUS] == 0 || 
		ps_th[ASSIST] == 0 || 
		ls_position == 0 || 
		le_position == 0 || 
		re_position == 0 ||
		force_z == 0 ||
		force_y == 0 ||
		torque_x == 0 ||
		acc_x == 0 ||
		acc_y == 0 ||
		angular_vel_z == 0
	) {
		cout << "ps_th[ASSIST] " << ps_th[ASSIST] << " " << "ps_th " << ps_th[CEPHEUS] << " " << "ls_position " << ls_position << " " << "le_position " << le_position 
		<< " " << "re_position " << re_position << "force_z " << force_z << "force_x " << force_x << "torque_y " << torque_y << endl;
		main_queue.callAvailable(ros::WallDuration(1.0));
        rokubimini_queue.callAvailable(ros::WallDuration(1.0));
		if (num_of_avail_rokubimini_readings){
			// ROS_WARN("WOWWW readiiingsss %f", num_of_avail_rokubimini_readings);
			force_x = sum_of_forces[X] / num_of_avail_rokubimini_readings;
			force_y = sum_of_forces[Y] / num_of_avail_rokubimini_readings;
			force_z = sum_of_forces[Z] / num_of_avail_rokubimini_readings;
			torque_x = sum_of_torques[X] / num_of_avail_rokubimini_readings;
			torque_y = sum_of_torques[Y] / num_of_avail_rokubimini_readings;
			torque_z = sum_of_torques[Z] / num_of_avail_rokubimini_readings;
		}
		num_of_avail_rokubimini_readings = 0;
		for (int k = 0; k < 3; k++){
			sum_of_forces[k] = 0;
			sum_of_torques[k] = 0;
		}
	}

	///////////////////////////////////////////////////////////////// LOOP STARTING ///////////////////////////////////////////////////////////
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

		// cout << "force_x: " << force_x << endl << endl;
		// cout << "x " << ps_x[ASSIST] - ps_x[CEPHEUS] << endl;
		// cout << "y " << ps_y[ASSIST] - ps_y[CEPHEUS] << endl;
		// cout << "thE " << ps_th[ASSIST] - ps_th[CEPHEUS] << endl;
		// cout << "th0 " << ps_th[CEPHEUS] << endl;


		///////////////////////////////////////////////////////////////// CALCULATING POLYNOMIAL PARAMETERS FOR PATH PLANNING ///////////////////////////////////////////////////////////
		if (first_time_movement) {

			move_time = secs;

			//Relative end-eff position, expressed in base frame
			xE_in_abs = ps_x[ASSIST] - ps_x[CEPHEUS];
			yE_in_abs = ps_y[ASSIST] - ps_y[CEPHEUS];
			thE_in = ps_th[ASSIST] - ps_th[CEPHEUS];	
			th0_in = ps_th[CEPHEUS];

			Matrix2d R00;
			R00 << cos(th0_in), -sin(th0_in),
					sin(th0_in), cos(th0_in);

			Matrix2d R00_trans;
			R00_trans = R00.transpose();
			
			Vector2d xys;
			xys << xE_in_abs, yE_in_abs;

			Vector2d qE_base;
			qE_base = R00_trans * xys;

			xE_in = qE_base(0);
			yE_in = qE_base(1);

			//Final conditions
			xE_fin_abs = 0.368654;
			yE_fin_abs = 0.444836;
			thE_fin = -0.426546;
			th0_fin = th0_in;
			double th0_metroumeno = -0.564535;

			Matrix2d R00_fin;
			R00_fin << 	cos(th0_metroumeno), -sin(th0_metroumeno),
						sin(th0_metroumeno), cos(th0_metroumeno);

			Matrix2d R00_fin_trans;
			R00_fin_trans = R00_fin.transpose();
			
			Vector2d xys_fin;
			xys_fin << xE_fin_abs, yE_fin_abs;

			Vector2d qE_base_fin;
			qE_base_fin = R00_fin_trans * xys_fin;

			xE_fin = qE_base_fin(0);
			yE_fin = qE_base_fin(1);

			q1 = ls_position;
			q2 = le_position;
			q3 = re_position;

			ps_x_prev[ASSIST] = ps_x[ASSIST];
			ps_y_prev[ASSIST] = ps_y[ASSIST];
			ps_th_prev[ASSIST] = ps_th[ASSIST];
			
			// cout << "EndEf in: " << xE_in << " " << yE_in << " " << thE_in << endl;
			// cout << "EndEf fin: " << xE_fin << " " << yE_fin << " " << thE_fin << endl;
			
			s_0 = 0.0;
			s_f = 1.0;
			s0dot = 0.0;
			sfdot = 0.0;
			s0dotdot = 0.0;
			sfdotdot = 0.0;

			G << 	1, t0, pow(t0,2), pow(t0,3), pow(t0,4), pow(t0,5),
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

			Ginv = G.inverse();
			x1 = Ginv*B1;

			a0 = x1(0);
			a1 = x1(1);
			a2 = x1(2);
			a3 = x1(3);
			a4 = x1(4);
			a5 = x1(5);


			G1 << 	1, t0, pow(t0,2), pow(t0,3), pow(t0,4), pow(t0,5),
					1, tf1, pow(tf1,2), pow(tf1,3), pow(tf1,4), pow(tf1,5),
					0, 1, 2*t0, 3*pow(t0,2), 4*pow(t0,3), 5*pow(t0,4),
					0, 1, 2*tf1, 3*pow(tf1,2), 4*pow(tf1,3), 5*pow(tf1,4),
					0, 0, 2, 6*t0, 12*pow(t0,2), 20*pow(t0,3),
					0, 0, 2, 6*tf1, 12*pow(tf1,2), 20*pow(tf1,3);

			G1inv = G.inverse();
			x11 = G1inv*B1;

			a10 = x11(0);
			a11 = x11(1);
			a12 = x11(2);
			a13 = x11(3);
			a14 = x11(4);
			a15 = x11(5);

			first_time_movement = false;

			calibration_x = force_x;
			calibration_y = force_y;
			calibration_z = force_z;
			calibration_tx = torque_x;
			calibration_ty = torque_y;
			calibration_tz = torque_z;

			// cout << "calibration_y = " << calibration_y << endl;

			ps_x_prev_prev[ASSIST] = ps_x_prev[ASSIST] = ps_x[ASSIST];
			ps_y_prev_prev[ASSIST] = ps_y_prev[ASSIST] = ps_y[ASSIST];
			ps_th_prev_prev[ASSIST] = ps_th_prev[ASSIST] = ps_th[ASSIST];
			ps_x_prev_prev[CEPHEUS] = ps_x_prev[CEPHEUS] = ps_x[CEPHEUS];
			ps_y_prev_prev[CEPHEUS] = ps_y_prev[CEPHEUS] = ps_y[CEPHEUS];
			ps_th_prev_prev[CEPHEUS] = ps_th_prev[CEPHEUS] = ps_th[CEPHEUS];
		}
		time = secs - move_time;

		if (time <= tf) {
		///////////////////////////////////////////////////////////////// PATH PLANNING INERTIAL FOR LAR GRASPING ///////////////////////////////////////////////////////////

			s = a5*pow(time,5) + a4*pow(time,4) + a3*pow(time,3) + a2*pow(time,2) + a1*time + a0;
			sdot = 5*a5*pow(time,4) + 4*a4*pow(time,3) + 3*a3*pow(time,2) + 2*a2*time + a1;
			sdotdot = 20*a5*pow(time,3) + 12*a4*pow(time,2) + 6*a3*time + 2*a2;

			// cout <<s<<sdot<<s0dotdot<<endl;//---------------------------------------------

			theta0_des = th0_in+s*(th0_fin-th0_in);
			xE = xE_in+s*(xE_fin-xE_in);
			yE = yE_in+s*(yE_fin-yE_in);
			theta0dot_des = sdot*(th0_fin-th0_in);
			xEdot = sdot*(xE_fin-xE_in);
			yEdot = sdot*(yE_fin-yE_in);
			theta0dotdot_des = sdotdot*(th0_fin-th0_in);
			xEdotdot = sdotdot*(xE_fin-xE_in);
			yEdotdot = sdotdot*(yE_fin-yE_in);
			
		} else {
			////////////////////////////////////////////////////// STAYING IN PLACE //////////////////////////////////////////////////////
			theta0_des = th0_fin;
			theta0dot_des = 0;
			theta0dotdot_des = 0;

			xE = xE_fin;
			yE = yE_fin;

			xEdot = 0;
			yEdot = 0;

			xEdotdot = 0;
			yEdotdot = 0;
		}

		if (time <= tf1){
			s1 = a15*pow(time,5) + a14*pow(time,4) + a13*pow(time,3) + a12*pow(time,2) + a11*time + a10;
			s1dot = 5*a15*pow(time,4) + 4*a14*pow(time,3) + 3*a13*pow(time,2) + 2*a12*time + a11;
			s1dotdot = 20*a15*pow(time,3) + 12*a14*pow(time,2) + 6*a13*time + 2*a12;

			thE = thE_in+s1*(thE_fin-thE_in);
			thEdot = s1dot*(thE_fin-thE_in);
			thEdotdot = s1dotdot*(thE_fin-thE_in);
		} else {
			thE = thE_fin;
			thEdot = 0;
			thEdotdot = 0;
			// cout << "xE\n" << xE << endl;//---------------------------------------------
		}

		if (ps_x[ASSIST] == ps_x_prev[ASSIST])
			ps_x[ASSIST] = ps_x_prev[ASSIST] + xEdot*time_step;

		if (ps_y[ASSIST] == ps_y_prev[ASSIST])
			ps_y[ASSIST] = ps_y_prev[ASSIST] + yEdot*time_step;

		if (ps_th[ASSIST] == ps_th_prev[ASSIST])
			ps_th[ASSIST] = ps_th_prev[ASSIST] + thEdot*time_step;


		//////////////////////////////////////////////////////////////// CONTROLLER INERTIAL FOR LAR GRASPING ///////////////////////////////////////////////////////////
		Matrix3d R0;
		R0 << cos(theta0_des), -sin(theta0_des), 0,
			  sin(theta0_des), cos(theta0_des), 0,
			  0, 0, 1;

		Vector3d qE_des(xE, yE, thE);
		Vector3d qE_rel_des;
		qE_rel_des = R0 * qE_des;

		Vector3d qEdot_des(xEdot, yEdot, thEdot);
		Vector3d qEdot_rel_des;
		qEdot_rel_des = R0 * qEdot_des;

		Vector3d qEdotdot_des(xEdotdot, yEdotdot, thEdotdot);
		Vector3d qEdotdot_rel_des;
		qEdotdot_rel_des = R0 * qEdotdot_des;

		errorq[0] = error_qdot[0] * (secs - prev_secs) + errorq[0];

		g_vel_x = (ps_x[ASSIST] - ps_x_prev[ASSIST]) / time_step;
		g_vel_y	= (ps_y[ASSIST] - ps_y_prev[ASSIST]) / time_step;
		g_vel_th = (ps_th[ASSIST] - ps_th_prev[ASSIST]) / time_step;

		// c_vel_x = (ps_x[CEPHEUS] - ps_x_prev[CEPHEUS]) / time_step;
		// c_vel_y = (ps_y[CEPHEUS] - ps_y_prev[CEPHEUS]) / time_step;
		c_vel_th = (ps_th[CEPHEUS] - ps_th_prev[CEPHEUS]) / time_step;

		if (abs(acc_x) < 0.0005)
			acc_x = 0.0;

		if (abs(acc_y) < 0.002)
			acc_y = 0.0;

		if (abs(angular_vel_z) < 0.007)
			angular_vel_z = 0.0;
		// cout << acc_x << " " << acc_y << " " << angular_vel_z << endl;

		// c_vel_x = c_vel_x + acc_x*time_step;
		// c_vel_y = c_vel_y + acc_y*time_step;
		c_vel_x = time_step * ((acc_x + acc_x_prev)/2);
		c_vel_y = time_step * ((acc_y + acc_y_prev)/2);

		// c_vel_th = angular_vel_z;

		acc_x_prev = acc_x;
		acc_y_prev = acc_y;

		if (abs(g_vel_x - g_vel_x_prev) > 0.06)
			g_vel_x = g_vel_x/10;

		if (abs(g_vel_y - g_vel_y_prev) > 0.06)
			g_vel_y = g_vel_y/10;

		if (abs(g_vel_th - g_vel_th_prev) > 0.06)
			g_vel_th = g_vel_th/10;

		if (abs(c_vel_x - c_vel_x_prev) > 0.06)
			c_vel_x = c_vel_x/10;

		if (abs(c_vel_y - c_vel_y_prev) > 0.06)
			c_vel_y = c_vel_y/10;

		if (abs(c_vel_th - c_vel_th_prev) > 0.06)
			c_vel_th = c_vel_th/10;

		g_vel_x_prev = g_vel_x;
		g_vel_y_prev = g_vel_y;
		g_vel_th_prev = g_vel_th;
		c_vel_x_prev = c_vel_x;
		c_vel_y_prev = c_vel_y;
		c_vel_th_prev = c_vel_th;

		theta1 = ls_position;
		theta2 = le_position;
		theta3 = re_position;
		theta0 = ps_th[CEPHEUS];

		Matrix2d R00;
		R00 << 	cos(theta0), -sin(theta0),
			  	sin(theta0), cos(theta0);
		
		Vector2d cvel;
		cvel << c_vel_x, c_vel_y;

		Vector2d newcvel;
		newcvel = R00 * cvel;
		c_vel_x = newcvel(0);
		c_vel_y = newcvel(1);

		// theta1 = 1;
		// theta2 = 1.5;
		// theta3 = 2;
		// theta0 = 0;Vector3Stamped

		theta0Dot = c_vel_th;
		theta1Dot = ls_velocity;
		theta2Dot = le_velocity;
		theta3Dot = re_velocity;

		// theta0Dot = 0;
		// theta1Dot = 1;
		// theta2Dot = 1.5;
		// theta3Dot = 2;

		th0 = theta0;
		q1 = theta1 + q01;
		q2 = theta2;
		q3 = theta3;

		th0dot = theta0Dot;
		q1dot = theta1Dot;
		q2dot = theta2Dot;
		q3dot = theta3Dot;

		// theta0_des = 0.1;
		// theta0dot_des = 0;
		// theta0dotdot_des = 0;

		MatrixXd Jeee(3,6);
		Jeee <<	1 , 0 , (-1)*r0y*cos(th0) + (-1)*r0x*sin(th0) + (-1)*l1*sin(q1 + th0) + (-1)*r1*sin(q1 + th0) + (-1)*l2*sin(q1 + q2 + th0) + (-1)*r2*sin(q1 + q2 + th0) + (-1)*l3*sin(q1 + q2 + q3 + th0) + (-1)*r3*sin(q1 + q2 + q3 + th0),
				(-1)*(l1 + r1)*sin(q1 + th0) + (-1)*(l2 + r2)*sin(q1 + q2 + th0) + (-1)*(l3 +r3)*sin(q1 + q2 + q3 + th0), (-1)*(l2 + r2)*sin(q1 + q2 + th0) + (-1)*(l3 + r3) *sin(q1 + q2 + q3 + th0), (-1)*(l3 + r3)*sin(q1 + q2 + q3 + th0),
				0, 1, r0x*cos(th0) + l1*cos(q1 + th0) + r1*cos(q1 + th0) + l2*cos(q1 + q2 + th0) + r2*cos(q1 + q2 + th0) + l3*cos(q1 + q2 + q3 + th0) + r3*cos(q1 + q2 + q3 + th0) + (-1)*r0y*sin(th0),
				(l1 + r1)*cos(q1 + th0) + (l2 + r2)*cos(q1 + q2 + th0) + (l3 + r3)*cos(q1 + q2 + q3 + th0),(l2 + r2)*cos(q1 + q2 + th0) + (l3 + r3)*cos(q1 + q2 + q3 + th0),(l3 + r3)*cos(q1 + q2 + q3 + th0), 
				0, 0, 1, 
				1, 1, 1;

		VectorXd qDot(6);

		qDot << c_vel_x,
				c_vel_y,
				c_vel_th,
				ls_velocity,
				le_velocity,
				re_velocity;

		qEDot = Jeee*qDot;

		qE << ps_x[ASSIST], ps_y[ASSIST], ps_th[ASSIST];

		// qE << 1.5,2.5,1.5;

		
		// qEDot << g_vel_x,
		// g_vel_y,
		// g_vel_th;

		// qEDot <<0.1,0.2,0.3;

		Vector3d qBDot;
		qBDot << c_vel_x , c_vel_y, c_vel_th;

		Vector3d qEDot_rel;
		qEDot_rel = qEDot - qBDot;

		Vector3d qB;
		qB << ps_x[CEPHEUS], ps_y[CEPHEUS], theta0;

		Vector3d qE_rel;
		qE_rel = qE - qB;
	
		ve = qE_rel;
		
		vedot = qEDot_rel;
	
		ve_des = qE_rel_des;

		vedot_des = qEdot_rel_des;

		thetaEdot = qEDot_rel(2);
		thetaE = qE_rel(2);
		thetaE_des = qE_rel_des(2);
		thetaEdot_des = qEdot_rel_des(2);
		thetaEdotdot_des = qEdotdot_rel_des(2);


		// cout << "ASSIST: " /*<< ps_x[ASSIST] << "  " << ps_y[ASSIST] << endl; << "  " */<< ps_th[ASSIST] << endl;//----------------------
		// cout << "CEPHEUS: " /*<< ps_x[CEPHEUS] << "  " << ps_y[CEPHEUS] << "  " */<< ps_th[CEPHEUS] << endl;//--------------------------------------------
		// cout << "positions: " << ls_position << "  " << le_position << "  " << re_position << endl;//--------------------------------------------
		// cout << "qE_des: " << qE_des(0) << "  " << qE_des(1) << "  " << qE_des(2) << endl;//--------------------------------------------
		// cout << "qEdot_des: " << qEdot_des(0) << "  " << qEdot_des(1) << "  " << qEdot_des(2) << endl;//--------------------------------------------
		// cout << "velocities: " << ls_velocity << "  " << le_velocity << "  " << re_velocity << endl;//--------------------------------------------

		// h = (time_step + time_step_prev)/2;

		// qEDot << (1/(2*h))*(ps_x_prev_prev[ASSIST] - 4*ps_x_prev[ASSIST] + 3*ps_x[ASSIST]),
		// 		 (1/(2*h))*(ps_y_prev_prev[ASSIST] - 4*ps_y_prev[ASSIST] + 3*ps_y[ASSIST]),
		// 		 (1/(2*h))*(ps_th_prev_prev[ASSIST] - 4*ps_th_prev[ASSIST] + 3*ps_th[ASSIST]);

		// qE << xE_in, yE_in, thE_in;

		// qEDot << 0, 0, 0;

		// cout /*<< "qE\n" << qE */<< "qEDot\n\n" << qEDot <<endl << endl;//-------------------------------------------------------

		Vector2d xEdotdot_des;
		xEdotdot_des << qEdotdot_rel_des(0), qEdotdot_rel_des(1);

		Vector2d xEdot;
		xEdot << qEDot_rel(0), qEDot_rel(1);

		Vector2d xE;
		xE << qE_rel(0), qE_rel(1);

		Vector2d xEdot_des;
		xEdot_des << qEdot_rel_des(0), qEdot_rel_des(1);

		Vector2d xE_des;
		xE_des << qE_rel_des(0), qE_rel_des(1);

		double error_theta = theta0_des - theta0;
		double errordot_theta = theta0dot_des - theta0Dot;

		Vector3d error_ve = ve_des - ve;

		// cout << "error_ve " << error_ve(0) << " " << error_ve(1) << " " << error_ve(2)  << endl;

		Vector3d errordot_ve = vedot_des - vedot;

		Vector4d error;
		error << error_theta, error_ve(0), error_ve(1), error_ve(2);
		// cout << "error: " <<  error(0) << " " << error(1) << " " << error(2) << " " << error(3) << endl;

		Vector4d error_dot;
		error_dot << errordot_theta, errordot_ve(0), errordot_ve(1), errordot_ve(2);
		// cout << "error_dot: " << error_dot(0) << " " << error_dot(1) << " " << error_dot(2) << " " << error_dot(3) << endl;

		double error_thetaE = thetaE_des - thetaE;
		double errordot_thetaE = thetaEdot_des - thetaEdot;

		// VectorXd qDot(6);

		// qDot << (1/(2*h))*(ps_x_prev_prev[CEPHEUS] - 4*ps_x_prev[CEPHEUS] + 3*ps_x[CEPHEUS]),
		// 		 (1/(2*h))*(ps_y_prev_prev[CEPHEUS] - 4*ps_y_prev[CEPHEUS] + 3*ps_y[CEPHEUS]),
		// 		 (1/(2*h))*(ps_th_prev_prev[CEPHEUS] - 4*ps_th_prev[CEPHEUS] + 3*ps_th[CEPHEUS]),
		// 		ls_velocity,
		// 		le_velocity,
		// 		re_velocity;

		// qDot << c_vel_x,
		// 		c_vel_y,
		// 		c_vel_th,
		// 		ls_velocity,
		// 		le_velocity,
		// 		re_velocity;

		// qDot << 1,2,3,1,2,3;

		// cout << "qDot:\n" << qDot << endl;//----------//----------//----------//----------//----------//----------//----------

		// VectorXd v1(6);
		// v1 = qDot;

		///////////relative///////////////////////////////

		// for testing matrix results/////////////////////////

		// th0 = 90 * (M_PI / 180);
		// theta0 = 90 * (M_PI / 180);
		// q1 = 80 * (M_PI / 180);
		// q2 = 70 * (M_PI / 180);
		// q3 = 60 * (M_PI / 180);
		// th0dot = 50 * (M_PI / 180);
		// q1dot = 40 * (M_PI / 180);
		// q2dot = 30 * (M_PI / 180);
		// q3dot = 20 * (M_PI / 180);
		// v1 << 1, 2, 3, 4, 5, 6;
		// double theta0dotdot_des = 0.0=2;
		// double theta0Dot=2;
		// double theta0dot_des = 90 * (M_PI / 180);
		// thetaEdot = 30 * (M_PI / 180);
		// thetaE = 30 * (M_PI / 180);
		// thetaE_des = 30 * (M_PI / 180);
		// thetaEdot_des = 30 * (M_PI / 180);
		// thetaEdotdot_des = 30 * (M_PI / 180);

		FextX = 0.0;
		Next = 0.0;
		FextY = 0.0;

		// cout << "forceeeee_z " << force_z << "force_y " << force_y << "torque_x " << torque_x << endl;

		if ((prev_force_z != force_z) && (force_z != 0))
			force_z = force_z - calibration_z;

		if ((prev_force_y != force_y) && (force_y != 0))
			force_y = force_y - calibration_y;

		if ((prev_torque_x != torque_x) && (torque_x != 0))
			torque_x = torque_x - calibration_tx;


		prev_force_z = force_z;
		prev_force_y = force_y;
		prev_torque_x = torque_x;

		if (abs(force_z) < 0.8)
			force_z = 0;

		if (abs(force_y) < 0.6)
			force_y = 0;

		if (abs(torque_x) < 0.2)
			torque_x = 0;


		// cout << "force_z " << force_z << " force_y " << force_y << " torque_x " << torque_x << endl;
	
		// for production
		// if (time >= 25) {

			// FextX = force_z;
			// FextY = - force_y;
			// Next = - torque_x;
		// }

		// cout << "FextX" << FextX << "FextY" << FextY << "Next" << Next << endl;
		// printf("FextX: %-10f   FextY: %-10f   Next: %-10f\n", FextX, FextY, Next);

		// cout << "Forces: " << force_x << force_y << force_z << " Torques: " << torque_x << torque_y << torque_z << endl;

		MatrixXd H(6, 6);
		H << 	p1,
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
		
		// cout << "Here is the matrix H:\n" << H << endl;

		// VectorXd c(6);
		// c << (-1)*p2*pow(th0dot,2)*cos(th0) + (-1)*p4*pow((q1dot + th0dot),2)*cos( 
		// 	q1 + th0) + (-1)*p5*pow(q1dot,2)*cos(q1 + q2 + th0) + (-2)*p5*q1dot* 
		// 	q2dot*cos(q1 + q2 + th0) + (-1)*p5*pow(q2dot,2)*cos(q1 + q2 + th0) + (-2)* 
		// 	p5*q1dot*th0dot*cos(q1 + q2 + th0) + (-2)*p5*q2dot*th0dot*cos(q1 +  
		// 	q2 + th0) + (-1)*p5*pow(th0dot,2)*cos(q1 + q2 + th0) + (-1)*p15*pow(q1dot,2)* 
		// 	cos(q1 + q2 + q3 + th0) + (-2)*p15*q1dot*q2dot*cos(q1 + q2 + q3 + th0) + (-1) 
		// 	*p15*pow(q2dot,2)*cos(q1 + q2 + q3 + th0) + (-2)*p15*q1dot*q3dot*cos( 
		// 	q1 + q2 + q3 + th0) + (-2)*p15*q2dot*q3dot*cos(q1 + q2 + q3 + th0) + (-1)* 
		// 	p15*pow(q3dot,2)*cos(q1 + q2 + q3 + th0) + (-2)*p15*q1dot*th0dot*cos(q1 +  
		// 	q2 + q3 + th0) + (-2)*p15*q2dot*th0dot*cos(q1 + q2 + q3 + th0) + (-2)*p15* 
		// 	q3dot*th0dot*cos(q1 + q2 + q3 + th0) + (-1)*p15*pow(th0dot,2)*cos(q1 + q2 +  
		// 	q3 + th0) + p3*pow(th0dot,2)*sin(th0),
		// 	(-1)*p3*pow(th0dot,2)*cos(th0) + (-1) 
		// 	*p2*pow(th0dot,2)*sin(th0) + (-1)*p4*pow(q1dot,2)*sin(q1 + th0) + (-2)* 
		// 	p4*q1dot*th0dot*sin(q1 + th0) + (-1)*p4*pow(th0dot,2)*sin(q1 + th0) + ( 
		// 	-1)*p5*pow(q1dot,2)*sin(q1 + q2 + th0) + (-2)*p5*q1dot*q2dot*sin(q1 +  
		// 	q2 + th0) + (-1)*p5*pow(q2dot,2)*sin(q1 + q2 + th0) + (-2)*p5*q1dot* 
		// 	th0dot*sin(q1 + q2 + th0) + (-2)*p5*q2dot*th0dot*sin(q1 + q2 + th0) + ( 
		// 	-1)*p5*pow(th0dot,2)*sin(q1 + q2 + th0) + (-1)*p15*pow(q1dot,2)*sin(q1 + q2 +  
		// 	q3 + th0) + (-2)*p15*q1dot*q2dot*sin(q1 + q2 + q3 + th0) + (-1)*p15* pow(q2dot,2)*sin(q1 + q2 + q3 + th0) + (-2)*p15*q1dot*q3dot*sin(q1 + q2 + q3 +  
		// 	th0) + (-2)*p15*q2dot*q3dot*sin(q1 + q2 + q3 + th0) + (-1)*p15* 
		// 	pow(q3dot,2)*sin(q1 + q2 + q3 + th0) + (-2)*p15*q1dot*th0dot*sin(q1 + q2 +  
		// 	q3 + th0) + (-2)*p15*q2dot*th0dot*sin(q1 + q2 + q3 + th0) + (-2)*p15* 
		// 	q3dot*th0dot*sin(q1 + q2 + q3 + th0) + (-1)*p15*pow(th0dot,2)*sin(q1 + q2 +  
		// 	q3 + th0),
		// 	p11*q1dot*(q1dot + 2*th0dot)*cos(q1) + p14*(q1dot + q2dot) 
		// 	*(q1dot + q2dot + 2*th0dot)*cos(q1 + q2) + p17*pow(q1dot,2)*cos(q1 + q2 + q3) 
		// 	+ 2*p17*q1dot*q2dot*cos(q1 + q2 + q3) + p17*pow(q2dot,2)*cos(q1 + q2 + q3) +  
		// 	2*p17*q1dot*q3dot*cos(q1 + q2 + q3) + 2*p17*q2dot*q3dot*cos(q1 +  
		// 	q2 + q3) + p17*pow(q3dot,2)*cos(q1 + q2 + q3) + 2*p17*q1dot*th0dot*cos(q1 +  
		// 	q2 + q3) + 2*p17*q2dot*th0dot*cos(q1 + q2 + q3) + 2*p17*q3dot* 
		// 	th0dot*cos(q1 + q2 + q3) + (-1)*p10*pow(q1dot,2)*sin(q1) + (-2)*p10* 
		// 	q1dot*th0dot*sin(q1) + (-2)*p12*q1dot*q2dot*sin(q2) + (-1)* 
		// 	p12*pow(q2dot,2)*sin(q2) + (-2)*p12*q2dot*th0dot*sin(q2) + (-1)* 
		// 	p13*pow(q1dot,2)*sin(q1 + q2) + (-2)*p13*q1dot*q2dot*sin(q1 + q2) + (-1) 
		// 	*p13*pow(q2dot,2)*sin(q1 + q2) + (-2)*p13*q1dot*th0dot*sin(q1 + q2) + ( 
		// 	-2)*p13*q2dot*th0dot*sin(q1 + q2) + (-2)*p19*q1dot*q3dot*sin( 
		// 	q3) + (-2)*p19*q2dot*q3dot*sin(q3) + (-1)*p19*pow(q3dot,2)*sin(q3) +  
		// 	(-2)*p19*q3dot*th0dot*sin(q3) + (-2)*p18*q1dot*q2dot*sin(q2 +  
		// 	q3) + (-1)*p18*pow(q2dot,2)*sin(q2 + q3) + (-2)*p18*q1dot*q3dot*sin( 
		// 	q2 + q3) + (-2)*p18*q2dot*q3dot*sin(q2 + q3) + (-1)*p18*pow(q3dot,2)* 
		// 	sin(q2 + q3) + (-2)*p18*q2dot*th0dot*sin(q2 + q3) + (-2)*p18*q3dot* 
		// 	th0dot*sin(q2 + q3) + (-1)*p16*pow(q1dot,2)*sin(q1 + q2 + q3) + (-2)*p16* 
		// 	q1dot*q2dot*sin(q1 + q2 + q3) + (-1)*p16*pow(q2dot,2)*sin(q1 + q2 + q3) + ( 
		// 	-2)*p16*q1dot*q3dot*sin(q1 + q2 + q3) + (-2)*p16*q2dot*q3dot* 
		// 	sin(q1 + q2 + q3) + (-1)*p16*pow(q3dot,2)*sin(q1 + q2 + q3) + (-2)*p16* 
		// 	q1dot*th0dot*sin(q1 + q2 + q3) + (-2)*p16*q2dot*th0dot*sin(q1 + q2 +  
		// 	q3) + (-2)*p16*q3dot*th0dot*sin(q1 + q2 + q3),
		// 	(-1)*p11*pow(th0dot,2)* 
		// 	cos(q1) + (-1)*p14*pow(th0dot,2)*cos(q1 + q2) + (-1)*p17*pow(th0dot,2)* 
		// 	cos(q1 + q2 + q3) + p10*pow(th0dot,2)*sin(q1) + (-2)*p12*q1dot*q2dot* 
		// 	sin(q2) + (-1)*p12*pow(q2dot,2)*sin(q2) + (-2)*p12*q2dot*th0dot* 
		// 	sin(q2) + p13*pow(th0dot,2)*sin(q1 + q2) + (-2)*p19*q1dot*q3dot*sin( 
		// 	q3) + (-2)*p19*q2dot*q3dot*sin(q3) + (-1)*p19*pow(q3dot,2)*sin(q3) +  
		// 	(-2)*p19*q3dot*th0dot*sin(q3) + (-2)*p18*q1dot*q2dot*sin(q2 +  
		// 	q3) + (-1)*p18*pow(q2dot,2)*sin(q2 + q3) + (-2)*p18*q1dot*q3dot*sin( 
		// 	q2 + q3) + (-2)*p18*q2dot*q3dot*sin(q2 + q3) + (-1)*p18*pow(q3dot,2)* 
		// 	sin(q2 + q3) + (-2)*p18*q2dot*th0dot*sin(q2 + q3) + (-2)*p18*q3dot* 
		// 	th0dot*sin(q2 + q3) + p16*pow(th0dot,2)*sin(q1 + q2 + q3),
		// 	(-1)*p14* pow(th0dot,2)*cos(q1 + q2) + (-1)*p17*pow(th0dot,2)*cos(q1 + q2 + q3) + p12* pow(q1dot,2)*sin(q2) + 2*p12*q1dot*th0dot*sin(q2) + p12*pow(th0dot,2)* 
		// 	sin(q2) + p13*pow(th0dot,2)*sin(q1 + q2) + (-2)*p19*q1dot*q3dot*sin( 
		// 	q3) + (-2)*p19*q2dot*q3dot*sin(q3) + (-1)*p19*pow(q3dot,2)*sin(q3) +  
		// 	(-2)*p19*q3dot*th0dot*sin(q3) + p18*pow(q1dot,2)*sin(q2 + q3) + 2* 
		// 	p18*q1dot*th0dot*sin(q2 + q3) + p18*pow(th0dot,2)*sin(q2 + q3) + p16*pow(th0dot,2)*sin(q1 + q2 + q3),
		// 	(-1)*p17*pow(th0dot,2)*cos(q1 + q2 + q3) + p19* 
		// 	pow((q1dot + q2dot + th0dot),2)*sin(q3) + p18*pow(q1dot,2)*sin(q2 + q3) + 2* 
		// 	p18*q1dot*th0dot*sin(q2 + q3) + p18*pow(th0dot,2)*sin(q2 + q3) + p16* 
		// 	pow(th0dot,2)*sin(q1 + q2 + q3);

		
		// cout << "Here is the vector c:\n" << c << endl;

		MatrixXd Je(3, 6);
		Je <<  	1,
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
		
		// cout << "Here is the matrix Je:\n" << Je << endl;

		MatrixXd Jedot(3, 6);
		Jedot << 	0,
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

		// cout << "Here is the matrix Jedot:\n" << Jedot << endl;

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
		Jvqdot <<  	Jedot(0,3), Jedot(0,4), Jedot(0,5),
					Jedot(1,3), Jedot(1,4), Jedot(1,5);
		Vector3d Jwqdot;
		Jwqdot << Jedot(2,3), Jedot(2,4), Jedot(2,5); 
		
		MatrixXd J1(6,6);
	    J1 <<   1, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0,
				0, 0, 1, 0, 0, 0,
				0, 0, Jvw(0), Jvq(0,0), Jvq(0,1), Jvq(0,2),
				0, 0, Jvw(1), Jvq(1,0), Jvq(1,1), Jvq(1,2),
				0, 0, 0, Jwq(0), Jwq(1), Jwq(2);  

		// cout << "Here is the matrix J1:\n" << J1 << endl;

		MatrixXd J1dot(6,6);
		J1dot << 	0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0,
					0, 0, Jvwdot(0), Jvqdot(0,0), Jvqdot(0,1), Jvqdot(0,2),  
					0, 0, Jvwdot(1), Jvqdot(1,0), Jvqdot(1,1), Jvqdot(1,2), 
					0, 0, 0, Jwqdot(0), Jwqdot(1), Jwqdot(2);    
	
		// cout << "Here is the matrix J1dot:\n" << J1dot << endl;


		MatrixXd J1_trans(6,6);
		J1_trans = J1.transpose();
		// cout << "J1_trans\n"<<J1_trans << endl;
		MatrixXd J1_trans_inv(6,6);
		J1_trans_inv = J1_trans.inverse();
		MatrixXd J1_inv(6,6);
		J1_inv = J1.inverse();
		MatrixXd Hstar(6,6);
		// cout << "J1_trans_inv\n"<<J1_trans_inv << endl;
		Hstar = J1_trans_inv*H*J1_inv;
		// VectorXd cstar(6);
		// cstar = J1_trans_inv*(c-(H*J1_inv*J1dot*v1));
		MatrixXd Jstar(6,6);
		Jstar = J1_trans_inv;

		double DJstar =Jstar.determinant();

		MatrixXd H11star(2,2);
		MatrixXd H12star(2,4);
		MatrixXd H21star(4,2);
		MatrixXd H22star(4,4);


		H11star << 	Hstar(0,0), Hstar(0,1),
					Hstar(1,0), Hstar(1,1);

		H12star << 	Hstar(0,2), Hstar(0,3), Hstar(0,4), Hstar(0,5),
					Hstar(1,2), Hstar(1,3), Hstar(1,4), Hstar(1,5);

		H21star << 	Hstar(2,0), Hstar(2,1),
					Hstar(3,0), Hstar(3,1),
					Hstar(4,0), Hstar(4,1),
					Hstar(5,0), Hstar(5,1);

		H22star << 	Hstar(2,2), Hstar(2,3), Hstar(2,4), Hstar(2,5), 
					Hstar(3,2), Hstar(3,3), Hstar(3,4), Hstar(3,5),
					Hstar(4,2), Hstar(4,3), Hstar(4,4), Hstar(4,5),
					Hstar(5,2), Hstar(5,3), Hstar(5,4), Hstar(5,5);

		// Vector2d c1star;
		// Vector4d c2star;

		// c1star << cstar(0,0), cstar(1,0);
		// c2star << cstar(2,0), cstar(3,0), cstar(4,0), cstar(5,0);

		MatrixXd J12star(2,4);
		MatrixXd J22star(4,4);


		J12star << 	Jstar(0,2), Jstar(0,3), Jstar(0,4), Jstar(0,5), 
					Jstar(1,2), Jstar(1,3), Jstar(1,4), Jstar(1,5); 

		J22star << 	Jstar(2,2), Jstar(2,3), Jstar(2,4), Jstar(2,5), 
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
		// Vector4d cbar;
		// cbar=c2star-H21star*H11star_inv*c1star;
		Matrix4d Jbar;
		Jbar=J22star-H21star*H11star_inv*J12star;
		MatrixXd Jebar_f(4,2);
		Jebar_f=Je21star-H21star*H11star_inv*Je11star; 
		Vector4d Jebar_s;
		Jebar_s=Je22star-H21star*H11star_inv*Je12star;
		MatrixXd Jebar(4,3);
		Jebar << 	Jebar_f(0,0), Jebar_f(0,1), Jebar_s(0),
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
		Rb << 	pow(eb1,2)+(-1)*pow(eb2,2)+(-1)*pow(eb3,2)+pow(nb,2),      2*eb1*eb2+(-2)*eb3*nb,      2*eb1*eb3+2*eb2*nb,
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
		// cout << "error_base: \n" << error_base << endl << endl; 
		RowVector3d eb_des_trans;
		eb_des_trans = eb_des.transpose();

		double ebn = eb_des_trans*eb + nb_des*nb;

		Matrix3d omegabcross;
		omegabcross << 	0, -omegab(2,0), omegab(1, 0),
						omegab(2, 0), 0, -omegab(0,0),
						-omegab(1, 0), omegab(0, 0), 0;

		double ee1 = 0.0;
		double ee2 = 0.0;
		double ee3 = sin(thetaE / 2);
		double ne = cos(thetaE / 2);
		double ee3_absolute = sin(ps_th[ASSIST]/ 2);
		double ne_absolute = cos(ps_th[ASSIST]/ 2);

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
			pow(ee1,2)+(-1)*pow(ee2,2)+(-1)*pow(ee3_absolute,2)+pow(ne_absolute,2),
			2*ee1*ee2+(-2)*ee3_absolute*ne_absolute,
			2*ee1*ee3_absolute+2*ee2*ne_absolute,
			
			2*ee1*ee2+2*ee3_absolute*ne_absolute,
			(-1)*pow(ee1,2)+pow(ee2,2)+(-1)*pow(ee3_absolute,2)+pow(ne_absolute,2),
			2*ee2*ee3_absolute+(-2)*ee1*ne_absolute,
			
			2*ee1*ee3_absolute+(-2)*ee2*ne_absolute,
			2*ee2*ee3_absolute+2*ee1*ne_absolute,
			(-1)*pow(ee1,2)+(-1)*pow(ee2,2)+pow(ee3_absolute,2)+pow(ne_absolute,2);

		Vector3d omegae_LAR(0, 0, thetaEdot);
		Matrix3d Re_trans;
		Re_trans = Re.transpose();
		Vector3d omegae;
		omegae = Re_trans*omegae_LAR;
		// cout << "omegae_LAR\n" << omegae_LAR << endl << endl;
		// cout << "omegae\n" << omegae << endl << endl;
		// cout << "omegae_des\n" << omegae_des << endl << endl;
		// cout << "Re_trans\n" << Re_trans << endl << endl;

		Vector3d errore_omega;
		errore_omega = omegae - Re_trans*omegae_des;

		RowVector3d ee_des_trans;
		ee_des_trans = ee_des.transpose();
		double een=ee_des_trans*ee+ne_des*ne;

		Matrix3d omegaecross;
		omegaecross << 	0, -omegae(2,0), omegae(1,0),
						omegae(2,0), 0, -omegae(0,0),
						-omegae(1,0), omegae(0,0), 0;

		Matrix3d eedcross;
		eedcross << 0, -ee_des(2,0), ee_des(1,0),
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

		Matrix2d ReAng;
		ReAng << cos(thetaE), -sin(thetaE), sin(thetaE), cos(thetaE);

		Vector2d Fextee;
		Fextee << FextX, FextY;
		
		Vector2d Fext;
		Fext = ReAng*Fextee;
		// cout << "Fext:\n" << Fext << endl << endl;
		Vector2d Qef(Fext(0), Fext(1));
		Vector3d Qet(0.0, 0.0, Next);      
			
		Vector3d uRW;

		RowVector3d errob_omega_trans;
		errob_omega_trans = errob_omega.transpose();

		// cout <<  << endl;

		double wtf = errob_omega_trans*errob_omega;
		wtf=wtf/4;
		
		uRW = Rb*(Rb_trans*omegabdot_des + omegabcross*errob_omega - KD*errob_omega - 2.0*(KP*error_base - wtf*error_base)/ebn);

		// cout << "errob_omega" << errob_omega << endl;
		// cout << "error_base" << error_base << endl;
		// cout << "uRW" << uRW << endl;

		Vector2d umr;
		Matrix2d HPS_inv;
		HPS_inv = HPS.inverse();


		umr = HPS_inv*(HPS*xEdotdot_des + DPS*(xEdot_des-xEdot) + KPS*(xE_des-xE) + Qef);

		// cout << "umr" << umr << endl;

		Vector3d umw;
		Matrix3d HAEes;
		HAEes = HA*Ees;
		Matrix3d HAEes_inv;
		HAEes_inv=HAEes.inverse();

		RowVector3d errore_omega_trans;
		errore_omega_trans = errore_omega.transpose();

		double test_num=(errore_omega_trans*errore_omega);

		test_num=test_num/4;
		Matrix3d test;
		test = KA*Ees - HA*Ees*test_num;

		umw = Re*((HAEes_inv)*(HA*Ees*(Re_trans*omegaedot_des + omegaecross*errore_omega) - DA*Ees*errore_omega + 2*Re_trans*Qet - 2*test*error_ee/een));

		// cout << "Re_trans:\n" << Re_trans << endl << endl;
		// cout << "Ees:\n" << Ees << endl << endl;
		
		// cout << "omegaedot_des:\n" << omegaedot_des << endl << endl;
		// cout << "omegaecross:\n" << omegaecross << endl << endl;
		// cout << "error_ee:\n" << error_ee << endl << endl;
		// cout << "een:\n" << een << endl << endl;

		// cout << "errore_omega:\n" << errore_omega << endl << endl;
		
		// cout << "test_num:\n" << test_num << endl << endl;
		// cout << "test:\n" << test << endl << endl;
		
		// cout << "Re:\n" << Re << endl << endl;
		// cout << "HAEes_inv:\n" << HAEes_inv << endl << endl;

		// cout << "umw:\n" << umw << endl << endl;

		double uRWn=uRW(2);
		double umwn=umw(2);
		Vector4d u;
		u << uRWn, umr(0), umr(1), umwn;

		Vector3d Qe(Fext(0), Fext(1), Next);
		Vector4d Qbar;
		
		Qbar = Hbar*u; //+ cbar;

		// cout << "Hbar\n" << Hbar << endl << endl;
		// cout << "u\n" << u << endl << endl;
		// cout << "cbar\n" << cbar << endl << endl;
		// cout << "Jebar\n" << Jebar << endl << endl;
		// cout << "Qe\n" << Qe << endl << endl;
		// cout << "Qbar:\n" << Qbar << endl << endl;
		double DHbar = Hbar.determinant();
		double DJbar = Jbar.determinant();
		Vector4d tau;
		Matrix4d Jbar_inv;
		Jbar_inv = Jbar.inverse();
		tau = Jbar_inv * Qbar;

		torq[0] = - tau(1)/186;
		torq[1] = tau(2)/186;
		torq[2] = - tau(3)/186;

		// test w/o joints
		// torq[0] = 0.000001;
		// torq[1] = 0.000001;
		// torq[2] = 0.000001;

		torq[3] = tau(0);

		// cout << "H22star\n" << H22star << endl << endl;
		// cout << "H21star\n" << H21star << endl << endl;
		// cout << "H11star_inv\n" << H11star_inv << endl << endl;
		// cout << "H12star\n" << H12star << endl << endl;
		// cout << "tau\n" << tau(1) << " " << tau(2)  << " " << tau(3) << endl << endl;
		// cout << "q1" << q1 << "q2" << q2 << "q3" << q3 << endl;
		// ROS_WARN("position: %f, torq: %f",rw_position,torq[3]);

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
		torque.data = filter_torque(torq[2], prev_torq[2]);
		// torque.data = torq[2];
		re_torque_pub.publish(torque);
		re_trq_pub.publish(torque);

		torque.data = torq[3];
		rw_torque_pub.publish(torque);
		rw_trq_pub.publish(torque);

		// temp_msg.data = qd[0];
		// ls_qd_pub.publish(temp_msg);
		// temp_msg.data = qd[1];
		// le_qd_pub.publish(temp_msg);
		// temp_msg.data = qd[2];
		// re_qd_pub.publish(temp_msg);

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
	
		// temp_msg.data = errorq[0];
		// ls_error_pub.publish(temp_msg);
		// temp_msg.data = errorq[1];
		// le_error_pub.publish(temp_msg);
		// temp_msg.data = errorq[2];
		// re_error_pub.publish(temp_msg);

		temp_msg.data = qE(0);
		gripper_x_pub.publish(temp_msg);
		temp_msg.data = qE(1);
		gripper_y_pub.publish(temp_msg);
		temp_msg.data = qE(2);
		gripper_th_pub.publish(temp_msg);

		// temp_msg.data = ps_x[CEPHEUS];
		// cepheus_x_pub.publish(temp_msg);
		// temp_msg.data = ps_y[CEPHEUS];
		// cepheus_y_pub.publish(temp_msg);
		temp_msg.data = theta0;
		cepheus_th_pub.publish(temp_msg);
		temp_msg.data = theta0Dot;
		cepheus_th_dot_pub.publish(temp_msg);
		temp_msg.data = theta0_des;
		cepheus_th_des_pub.publish(temp_msg);

		// temp_msg.data = qEDot(0);
		// gripper_x_dot_pub.publish(temp_msg);
		// temp_msg.data = qEDot(1);
		// gripper_y_dot_pub.publish(temp_msg);
		// temp_msg.data = qEDot(2);
		// gripper_th_dot_pub.publish(temp_msg);

		// temp_msg.data = gripper_error_x;
		// gripper_error_x_pub.publish(temp_msg);
		// temp_msg.data = gripper_error_y;
		// gripper_error_y_pub.publish(temp_msg);
		// temp_msg.data = gripper_error_th;
		// gripper_error_th_pub.publish(temp_msg);
		temp_msg.data = qE_rel_des(0);
		qe_des_x_pub.publish(temp_msg);
		temp_msg.data = qE_rel_des(1);
		qe_des_y_pub.publish(temp_msg);
		temp_msg.data = qE_rel_des(2);
		qe_des_th_pub.publish(temp_msg);
		// temp_msg.data = qEdot_des(0);
		// qe_dot_des_x_pub.publish(temp_msg);
		// temp_msg.data = qEdot_des(1);
		// qe_dot_des_y_pub.publish(temp_msg);
		// temp_msg.data = qEdot_des(2);
		// qe_dot_des_th_pub.publish(temp_msg);
		// temp_msg.data = qEdotdot_des(0);
		// qe_dotdot_des_x_pub.publish(temp_msg);
		// temp_msg.data = qEdotdot_des(1);
		// qe_dotdot_des_y_pub.publish(temp_msg);
		// temp_msg.data = qEdotdot_des(2);
		// qe_dotdot_des_th_pub.publish(temp_msg);

		// temp_msg.data = errordot_theta;
		// errordot_th_pub.publish(temp_msg);
		// temp_msg.data = error_theta;
		// error_th_pub.publish(temp_msg);
		// temp_msg.data = gripper_x_desdot;
		// gripper_x_desdot_pub.publish(temp_msg);
		temp_msg.data = force_z;
		fextx_pub.publish(temp_msg);
		temp_msg.data = force_y;
		fexty_pub.publish(temp_msg);
		temp_msg.data = force_x;
		fextz_pub.publish(temp_msg);
		temp_msg.data = torque_x;
		next_pub.publish(temp_msg);
		temp_msg.data = torque_y;
		nexty_pub.publish(temp_msg);
		temp_msg.data = torque_z;
		nextx_pub.publish(temp_msg);
		temp_msg.data = DJbar;
		det_pub.publish(temp_msg);
		temp_msg.data = DHbar;
		det_h_pub.publish(temp_msg);
		
		temp_msg.data = u(0);
		uRWn_pub.publish(temp_msg);  
		temp_msg.data = u(1);
		umr_x_pub.publish(temp_msg); 
		temp_msg.data = u(2);
		umr_y_pub.publish(temp_msg); 
		temp_msg.data = u(3);
		umwn_pub.publish(temp_msg);

		temp_msg.data = error(0);
		error_0_pub.publish(temp_msg);
		temp_msg.data = error(1);
		error_1_pub.publish(temp_msg);
		temp_msg.data = error(2);
		error_2_pub.publish(temp_msg);
		temp_msg.data = error(3);
		error_3_pub.publish(temp_msg);
		temp_msg.data = error_dot(0);
		error_dot_0_pub.publish(temp_msg);
		temp_msg.data = error_dot(1);
		error_dot_1_pub.publish(temp_msg);
		temp_msg.data = error_dot(2);
		error_dot_2_pub.publish(temp_msg);
		temp_msg.data = error_dot(3);
		error_dot_3_pub.publish(temp_msg);

		// temp_msg.data = botarm_force_x;
		// botarm_force_x_pub.publish(temp_msg);
		// temp_msg.data = botarm_force_z;
		// botarm_force_z_pub.publish(temp_msg);
		// temp_msg.data = botarm_torque_y;
		// botarm_torque_y_pub.publish(temp_msg);

		ps_x_prev_prev[ALX_GRIPPER] = ps_x_prev[ALX_GRIPPER];
		ps_y_prev_prev[ALX_GRIPPER] = ps_y_prev[ALX_GRIPPER];
		ps_th_prev_prev[ALX_GRIPPER] = ps_th_prev[ALX_GRIPPER];
		ps_x_prev[ALX_GRIPPER] = ps_x[ALX_GRIPPER];
		ps_y_prev[ALX_GRIPPER] = ps_y[ALX_GRIPPER];
		ps_th_prev[ALX_GRIPPER] = ps_th[ALX_GRIPPER];
		ps_x_prev_prev[CEPHEUS] = ps_x_prev[CEPHEUS];
		ps_y_prev_prev[CEPHEUS] = ps_y_prev[CEPHEUS];
		ps_th_prev_prev[CEPHEUS] = ps_th_prev[CEPHEUS];
		ps_x_prev[CEPHEUS] = ps_x[CEPHEUS];
		ps_y_prev[CEPHEUS] = ps_y[CEPHEUS];
		ps_th_prev[CEPHEUS] = ps_th[CEPHEUS];
		ps_x_prev_prev[ASSIST] = ps_x_prev[ASSIST];
		ps_y_prev_prev[ASSIST] = ps_y_prev[ASSIST];
		ps_th_prev_prev[ASSIST] = ps_th_prev[ASSIST];
		ps_x_prev[ASSIST] = ps_x[ASSIST];
		ps_y_prev[ASSIST] = ps_y[ASSIST];
		ps_th_prev[ASSIST] = ps_th[ASSIST];

		// ros::spinOnce();        
		num_of_avail_rokubimini_readings = 0;

		for (int k = 0; k < 3; k++){
			sum_of_forces[k] = 0;
			sum_of_torques[k] = 0;
		}
		
		rokubimini_queue.callAvailable(ros::WallDuration(0.0));
		if (num_of_avail_rokubimini_readings){
			// ROS_WARN("WOWWW readiiingsss %f", num_of_avail_rokubimini_readings);
			force_x = sum_of_forces[X] / num_of_avail_rokubimini_readings;
			force_y = sum_of_forces[Y] / num_of_avail_rokubimini_readings;
			force_z = sum_of_forces[Z] / num_of_avail_rokubimini_readings;
			torque_x = sum_of_torques[X] / num_of_avail_rokubimini_readings;
			torque_y = sum_of_torques[Y] / num_of_avail_rokubimini_readings;
			torque_z = sum_of_torques[Z] / num_of_avail_rokubimini_readings;
		}
		main_queue.callAvailable(ros::WallDuration(0.0));
		ps_counter++;
		loop_rate.sleep();
	}

	return 0;
}
