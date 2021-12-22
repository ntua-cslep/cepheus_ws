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

double rw_position = 0.0;
double rw_velocity = 0.0;
bool first_time_movement = true;
bool first_time_ls = true;
bool first_time_le = true;
bool first_time_re = true;

double ps_x = 0.0, ps_y = 0.0, ps_theta = 0.0, ps_theta_prev = 0.0, rw_position_prev = 0.0;
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

	tf::Quaternion q(x, y, z, w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

	ps_x = temp.transform.translation.x;
	ps_y = temp.transform.translation.y;
	ps_theta = yaw;
	return;
}


void rwPosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	rw_position = cmd->data;
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
	ros::Publisher rw_pos_pub = nh.advertise<std_msgs::Float64>("reaction_wheel_pos", 1);
	ros::Publisher rw_vel_pub = nh.advertise<std_msgs::Float64>("reaction_wheel_vel", 1);
	ros::Publisher rw_vell_pub = nh.advertise<std_msgs::Float64>("reaction_wheel_vell", 1);
	ros::Publisher thrust_pub = nh.advertise<geometry_msgs::Vector3Stamped>("cmd_thrust", 1);

	ros::Publisher cepheus_x_pub = nh.advertise<std_msgs::Float64>("c_x", 1);
	ros::Publisher cepheus_y_pub = nh.advertise<std_msgs::Float64>("c_y", 1);

	ros::Publisher qe_des_x_pub = nh.advertise<std_msgs::Float64>("qe_des_x", 1);
	ros::Publisher qe_des_y_pub = nh.advertise<std_msgs::Float64>("qe_des_y", 1);
	ros::Publisher qe_dot_des_x_pub = nh.advertise<std_msgs::Float64>("qe_dot_des_x", 1);
	ros::Publisher qe_dot_des_y_pub = nh.advertise<std_msgs::Float64>("qe_dot_des_y", 1);
	ros::Publisher qe_dotdot_des_x_pub = nh.advertise<std_msgs::Float64>("qe_dotdot_des_x", 1);
	ros::Publisher qe_dotdot_des_y_pub = nh.advertise<std_msgs::Float64>("qe_dotdot_des_y", 1);

	ros::Publisher imu_angular_vel_pub = nh.advertise<std_msgs::Float64>("imu_angular_vel", 1);
	ros::Publisher ps_theta_pub = nh.advertise<std_msgs::Float64>("ps_theta", 1);
	ros::Publisher ps_theta_dot_pub = nh.advertise<std_msgs::Float64>("ps_theta_dot", 1);

    ros::Publisher imu_orientation_pub = nh.advertise<std_msgs::Float64>("imu_orient", 1);
    ros::Publisher imu_vel_y_pub = nh.advertise<std_msgs::Float64>("imu_vel_y", 1);
    ros::Publisher imu_vel_x_pub = nh.advertise<std_msgs::Float64>("imu_vel_x", 1);
    ros::Publisher imu_acc_y_pub = nh.advertise<std_msgs::Float64>("imu_acc_y", 1);
    ros::Publisher imu_acc_x_pub = nh.advertise<std_msgs::Float64>("imu_acc_x", 1);

	ros::Subscriber rw_pos_sub = nh.subscribe("read_reaction_wheel_position", 1, rwPosCallback);
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
	double t = 0.0, time = 0.0, move_time = 0.0, tminus = 0.0;
	
	double errorq[4];
	double error_qdot[4];
	double torq[4];
	double prev_torq[4];
	double qd[4];
	double qd_dot[4];
	double ps_theta_dot = 0.0;
	double ps_theta_dot_prev = 0.0;
	double rw_vel = 0.0;
	double orientation = 0.0;
	double imu_orient = 0.0, imu_orient_prev = 0.0;
	double imu_vel_x = 0.0, imu_vel_y = 0.0;
	double imu_vel_x_prev = 0.0, imu_vel_y_prev = 0.0;

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

	while (ps_theta == 0.0 || angular_vel_z == 0.0 || ps_x == 0.0 || ps_y == 0.0 || acc_x == 0 || acc_y == 0) {
		cout << "ps_theta " << ps_theta << " angular_vel_z " << angular_vel_z << "ps_x " << ps_x << " ps_y " << ps_y << endl;
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

		ps_theta_dot = (ps_theta - ps_theta_prev) / time_step;
		rw_vel = (rw_position - rw_position_prev) / time_step;
		// if (abs(ps_theta_dot - ps_theta_dot_prev) > 0.1)ee
		// 	ps_theta_dot = ps_theta_dot / 10;
		
		if (abs(acc_x) < 0.0005)
			acc_x = 0.0;

		if (abs(acc_y) < 0.002)
			acc_y = 0.0;

		if (abs(angular_vel_z) < 0.007)
			angular_vel_z = 0.0;

		// imu_vel_x = time_step * ((acc_x + acc_x_prev)/2);
		// imu_vel_y = time_step * ((acc_y + acc_y_prev)/2);
		imu_vel_x = imu_vel_x_prev + (acc_x * time_step);
		imu_vel_y = imu_vel_y_prev + (acc_y * time_step);
		// imu_orient = time_step * ((angular_vel_z + angular_vel_z_prev)/2);
		imu_orient = imu_orient_prev + (angular_vel_z * time_step);

		angular_vel_z_prev = angular_vel_z;

		acc_x_prev = acc_x;
		acc_y_prev = acc_y;

		imu_vel_x_prev = imu_vel_x;
		imu_vel_y_prev = imu_vel_y;

		imu_orient_prev = imu_orient;

		if (first_time_movement) {
			move_time = secs;
			orientation = ps_theta;
			first_time_movement = false;
		}

		t = secs - move_time;

		error_qdot[3] = 0 - angular_vel_z;
		errorq[3] = orientation - ps_theta;
		torq[3] = (1.4 * error_qdot[3] + 3.0 * errorq[3]);
		// torq[3] = 0.0;

		thrust_vector.vector.y = 0.0;

		if (t <= 4) {
			thrust_vector.vector.x = 0.6;
			thrust_vector.vector.z = 0.6;
		}
		else if (t <= 12) {
			thrust_vector.vector.x = - 0.6;
			thrust_vector.vector.z = - 0.6;
		} else if (t <= 20) {
			thrust_vector.vector.x =  0.6;
			thrust_vector.vector.z =  0.6;
		} else if (t <= 28) {
			thrust_vector.vector.x = - 0.6;
			thrust_vector.vector.z = - 0.6;
		} else if (t <= 36) {
			thrust_vector.vector.x =  0.6;
			thrust_vector.vector.z =  0.6;
		} else if (t <= 44) {
			thrust_vector.vector.x = - 0.6;
			thrust_vector.vector.z = - 0.6;
		} else if (t <= 48) {
			thrust_vector.vector.x =  0.6;
			thrust_vector.vector.z =  0.6;
		}
		else {
			thrust_vector.vector.x = 0.0;
			thrust_vector.vector.z = 0.0;
		}

		prev_secs = secs;

		_secs.data = secs;
		secs_pub.publish(_secs);


		torque.data = torq[3];
		rw_torque_pub.publish(torque);
		rw_trq_pub.publish(torque);

		thrust_pub.publish(thrust_vector);

		temp_msg.data = rw_position;
		rw_pos_pub.publish(temp_msg);
		temp_msg.data = rw_velocity;
		rw_vel_pub.publish(temp_msg);

		temp_msg.data = rw_vel;
		rw_vell_pub.publish(temp_msg);

		temp_msg.data = angular_vel_z;
		imu_angular_vel_pub.publish(temp_msg);
		temp_msg.data = ps_theta;
		ps_theta_pub.publish(temp_msg);

		temp_msg.data = imu_vel_x;
		imu_vel_x_pub.publish(temp_msg);
		temp_msg.data = imu_vel_y;
		imu_vel_y_pub.publish(temp_msg);
		temp_msg.data = imu_orient;
		imu_orientation_pub.publish(temp_msg);
		temp_msg.data = acc_x;
		imu_acc_x_pub.publish(temp_msg);
		temp_msg.data = acc_y;
		imu_acc_y_pub.publish(temp_msg);

		temp_msg.data = ps_theta_dot;
		ps_theta_dot_pub.publish(temp_msg);

		temp_msg.data = ps_x;
		cepheus_x_pub.publish(temp_msg);
		temp_msg.data = ps_y;
		cepheus_y_pub.publish(temp_msg);

		ps_theta_prev = ps_theta;
		rw_position_prev = rw_position;
		ps_theta_dot_prev = ps_theta_dot;

		main_queue.callAvailable(ros::WallDuration(0.0));
		loop_rate.sleep();
	}

	return 0;
}
