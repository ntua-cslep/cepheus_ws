#include <signal.h>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#define POS_FILTER 0.005
#define VEL_FILTER 0.05
#define TORQUE_LIMIT 0.001


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


double filter_torque(double torq) {
	if (torq == 0.0){
		torq = 0.00001;
		printf("CHANGED ZERO TORQUE\n");
	}
		
	return torq;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "alex_gripper_control_node", ros::init_options::NoSigintHandler);

	ros::NodeHandle nh;

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
	// final poza
	double q1_init = -60 * (M_PI / 180);
	double q2_init = 105 * (M_PI / 180);
	double q3_init = 45 * (M_PI / 180);

	double theta0_des = 90 * (M_PI / 180);
	double theta0_desdot = 0.0;

	double Kp = 0.06;
	double Kd = 0.006;
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
	// from launch file
	double qd1_init = 1.34;
	double qd2_init = 2.22;
	double qd3_init = 1.899951;

	ros::Time curr_time, t_beg = ros::Time::now();
	ros::Duration all_time;
	double prev_secs = 0.0;
	bool ls_moved = false;
	bool re_moved = false;
	bool kati = true;
	bool kati2 = true;
	bool kati3 = true;
	bool first_time_movement = true;
	double le_position_after_ls_init, re_position_after_ls_init;

	std_msgs::Float64 torque;
	std_msgs::Float64 offset;
	std_msgs::Float64 temp_msg;
	std_msgs::Float64 _secs;
	std_msgs::UInt16 lar;
	bool initialized = false;
	sleep(4);
	double secs, ls_time, le_time, re_time, move_time;

	while (ros::ok()) {

		for (int i = 0; i < 3; i++) {
			prev_torq[i] = torq[i];
		}

		curr_time = ros::Time::now();
		all_time = curr_time - t_beg;

		secs = all_time.sec + all_time.nsec * pow(10, -9);

		if (!initialized) {
			if (!ls_initialized) {
				// initialize ls
				// if (kati) {
				// 	ls_time = secs;
				// 	kati =false;
				// }
                // s = (0.0000002469135802469135 * pow(secs - ls_time, 5)) + (-0.00001851851851851852* pow(secs - ls_time, 4)) + (0.0003703703703703703* pow(secs - ls_time, 3))+(-5.921189464667501*pow(10,-17)* (secs - ls_time));
				// s_dot = (5 * 0.0000002469135802469135 * pow(secs - ls_time, 4)) + (4 * -0.00001851851851851852 * pow(secs - ls_time, 3)) + (3 * 0.0003703703703703703* pow(secs - ls_time, 2))-5.921189464667501*pow(10,-17);
				
				// qd[0] = (2.5) * s;
			    // qd_dot[0] = (2.5) * s_dot;

				// error_qdot[0] = qd_dot[0] + ls_velocity;
				// error_qdot[1] = 0 - le_velocity;
				// error_qdot[2] = 0 + re_velocity;

				// errorq[0] = qd[0] + ls_position;
				// errorq[1] = 0 - le_position;
				// errorq[2] = 0 + re_position;

				// torq[0] = - (Kp * errorq[0] + Kd * error_qdot[0]);
				// torq[1] = Kp/3 * errorq[1] + Kd/3 * error_qdot[1];
				// torq[2] = - (Kp * errorq[2] + Kd * error_qdot[2]);

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
					// initialize right elbow
					// if (kati2) {
					// 	ls_time = secs;
					// 	kati2 =false;
					// }
					// s = (0.0000002469135802469135 * pow(secs - ls_time, 5)) + (-0.00001851851851851852* pow(secs - ls_time, 4)) + (0.0003703703703703703* pow(secs - ls_time, 3))+(-5.921189464667501*pow(10,-17)* (secs - ls_time));
					// s_dot = (5 * 0.0000002469135802469135 * pow(secs - ls_time, 4)) + (4 * -0.00001851851851851852 * pow(secs - ls_time, 3)) + (3 * 0.0003703703703703703* pow(secs - ls_time, 2))-5.921189464667501*pow(10,-17);
					
					// qd[2] = (2.5) * s;
					// qd_dot[2] = (2.5) * s_dot;

					// error_qdot[0] = 0 - ls_velocity;
					// error_qdot[1] = 0 - le_velocity;
					// error_qdot[2] = qd_dot[2] + re_velocity;

					// errorq[0] = q1_init - ls_position;
					// errorq[1] = 0 - le_position;
					// errorq[2] = qd[2] + re_position;

					// torq[0] = - (Kp * errorq[0] + Kd * error_qdot[0]);
					// torq[1] = Kp * errorq[1] + Kd * error_qdot[1];
					// torq[2] = - (Kp * errorq[2] + Kd * error_qdot[2]);
  
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
					// initialize left elbow
					// if (kati3) {
					// 	ls_time = secs;
					// 	kati3 =false;
					// }
					// s = (0.0000002469135802469135 * pow(secs - ls_time, 5)) + (-0.00001851851851851852* pow(secs - ls_time, 4)) + (0.0003703703703703703* pow(secs - ls_time, 3))+(-5.921189464667501*pow(10,-17)* (secs - ls_time));
					// s_dot = (5 * 0.0000002469135802469135 * pow(secs - ls_time, 4)) + (4 * -0.00001851851851851852 * pow(secs - ls_time, 3)) + (3 * 0.0003703703703703703* pow(secs - ls_time, 2))-5.921189464667501*pow(10,-17);
					
					// qd[1] = (2.5) * s;
					// qd_dot[1] = (2.5) * s_dot;

					// error_qdot[0] = 0 - ls_velocity;
					// error_qdot[1] = qd_dot[1]  - le_velocity;
					// error_qdot[2] = 0 - re_velocity;

					// errorq[0] = q1_init - ls_position;
					// errorq[1] = qd[1] - le_position;
					// errorq[2] = q3_init - re_position;

					// torq[0] = - (Kp * errorq[0] + Kd * error_qdot[0]);
					// torq[1] = Kp * errorq[1] + Kd * error_qdot[1];
					// torq[2] = - (Kp * errorq[2] + Kd * error_qdot[2]);
						
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
			error_qdot[0] = 0 - ls_velocity;
			error_qdot[1] = 0 - le_velocity;
			error_qdot[2] = 0 - re_velocity;

			errorq[0] = q1_init - ls_position;
			errorq[1] = q2_init - le_position;
			errorq[2] = q3_init - re_position;

			torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
			torq[1] = 1.5*Kp * errorq[1] + 1.5*Kd * error_qdot[1];
			torq[2] = - (1.5*Kp * errorq[2] + 1.5*Kd * error_qdot[2]);

		} else {
			// calculate secs
			if (first_time_movement) {
				qd[0] = q1_init;
				qd[1] = q2_init;
				qd[2] = q3_init;
				move_time = secs;
				first_time_movement = false;
			}

			if ((secs - move_time) <= 10.0) {

				a = 0.143318141419581;
				b = 0.331623152993806;
				c = 0.257342875265441;
				d = 0.257361884747802;
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

				S = a*b*d2*sin(qd[0]) + b*c*d0*sin(qd[1]) - a*c*d1*sin(qd[0]+qd[1]);
				s_dot = (5 * 0.00006 * pow((secs - move_time), 4)) + (4 * -0.0015 * pow((secs - move_time), 3)) + (3 * 0.01 * pow((secs - move_time), 2));
				xe_desdot = - 0.1 * s_dot;

				theta0_desdot = ((b*d2*cos(theta0_des+qd[0])-c*d1*cos(theta0_des+qd[0]+qd[1]))*xe_desdot)/S;
				qd_dot[0] = ((-d2*(a*cos(theta0_des)+b*cos(theta0_des+qd[0]))+c*(d0+d1)*cos(theta0_des+qd[0]+qd[1]))*xe_desdot)/S; 
				qd_dot[1] = ((a*(d1+d2)*cos(theta0_des)-d0*(b*cos(theta0_des+qd[0])+c*cos(theta0_des+qd[0]+qd[1])))*xe_desdot)/S;
				qd_dot[2] = ((-a*d1*cos(theta0_des)+b*d0*cos(theta0_des+qd[0]))*xe_desdot)/S;

				qd[0] = qd_dot[0] * (secs - prev_secs) + qd[0];
				qd[1] = qd_dot[1] * (secs - prev_secs) + qd[1];
				qd[2] = qd_dot[2] * (secs - prev_secs) + qd[2];
				theta0_des = theta0_desdot * (secs - prev_secs) + theta0_des;
			}
			else {
				qd_dot[0] = 0.0;
				qd_dot[1] = 0.0;
				qd_dot[2] = 0.0;

				// PUBLISH LAR
				lar.data = 1;
				lar_pub.publish(lar);
			}

			ROS_INFO("secs: %lf   |   qd0: %lf,    qd1: %lf,    qd2: %lf", secs, qd[0], qd[1], qd[2]);

			errorq[0] = qd[0] - ls_position;
			errorq[1] = qd[1] - le_position;
			errorq[2] = qd[2] - re_position;

			error_qdot[0] = qd_dot[0] - ls_velocity;
			error_qdot[1] = qd_dot[1] - le_velocity;
			error_qdot[2] = qd_dot[2] - re_velocity;

			// TODO: check prosimo after offset
			torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
			torq[1] = 1.5*Kp * errorq[1] + 1.5*Kd * error_qdot[1];
			torq[2] = - (2.0*Kp * errorq[2] + 2.0*Kd * error_qdot[2]);
		}

		prev_secs = secs;
		
		// ROS_INFO("secs: %lf   |   errorq[0]: %lf,    error_qdot[0]: %lf", secs, errorq[0], error_qdot[0]);

		_secs.data = secs;
		secs_pub.publish(_secs);
		torque.data = filter_torque(torq[0]);
		torque.data = torq[0];
		ls_torque_pub.publish(torque);
		torque.data = filter_torque(torq[1]);
		torque.data = torq[1];
		le_torque_pub.publish(torque);
		torque.data = filter_torque(torq[2]);
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


		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
