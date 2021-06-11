#include <signal.h>
#include <stdio.h>
#include <math.h>

#include <sys/resource.h>
FILE *latency_fp;
#include <boost/bind.hpp>
#define RT_PRIORITY 95


#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <controller_manager/controller_manager.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>

#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>

#include <tf/transform_listener.h>
#include "cepheus_hardware.h"
#include "cepheus_ctrl.h"

//#include <cepheus_robot/RightCatchObjectAction.h>
//include <cepheus_robot/LeftCatchObjectAction.h>
#include <actionlib/server/simple_action_server.h>

//typedef actionlib::SimpleActionServer<cepheus_robot::RightCatchObjectAction> ActionServerRightArm;
//typedef actionlib::SimpleActionServer<cepheus_robot::LeftCatchObjectAction> ActionServerLeftArm;

#define SH_DUR 2
#define ELB_DUR 2

//Panagiotis Mavridis
//---Constants for fsr force controller
#define F_DES 3
#define KP 0.01
#define KI 1
#define F_HIGH 4
#define F_LOW 2
#define DURATION_NO_GRIP 4
#define FSR_AVG_THRESHOLD 2

//------------------------------------


CepheusHW robot;


double rw_torque = 0.0;
double rw_cur_vel = 0.0;
double rw_last_vel = 0.0;

bool standard_controllers_started = false;
bool left_shoulder_ctrl_started = false;
bool left_elbow_ctrl_started = false;
bool right_shoulder_ctrl_started = false;
bool right_elbow_ctrl_started = false;


bool ready_to_grip_left = false, ready_to_grip_right = false;
bool left_gripper_first_time = true, right_gripper_first_time = true;

//In order to be used from callbacks
ros::Publisher  left_shoulder_pub, left_elbow_pub, right_shoulder_pub, right_elbow_pub;



//-----------------------------Implemented in cepheus_arms_operations.cpp-----------------------------
double produce_trajectory_point(double time, double movement_duration, double init_pos, double set_point);
double produce_trajectory_point_wrist(double time, double movement_duration, double init_pos, double set_point);
double produce_sin_trajectory(double width, double period, double t);
double produce_sin_trajectory_wrist(double width, double period, double t);
void moveLeftArmSin(controller_manager::ControllerManager& cm,
					CepheusHW& robot,
					ros::Publisher right_shoulder_pub,
					ros::Publisher right_elbow_pub);
void move_left_arm(double set_point_shoulder,
					double set_point_elbow,
					double set_point_wrist,
					double movement_duration,
					controller_manager::ControllerManager& cm,
					CepheusHW& robot,
					ros::Publisher left_shoulder_pub,
					ros::Publisher left_elbow_pub);

void move_right_arm(double set_point_shoulder,
					double set_point_elbow,
					double set_point_wrist,
					double movement_duration,
					controller_manager::ControllerManager& cm,
					CepheusHW& robot,
					ros::Publisher right_shoulder_pub,
					ros::Publisher right_elbow_pub);

//void test_catch_object(double cmd_angle_to_catch, controller_manager::ControllerManager& cm, double xt, double yt);
//---------------------------------------------------------------------------------------------------------------------

//-----------------------------Implemented in cepheus_init_ctrls.cpp--------------------------
void start_standard_controllers(ros::NodeHandle& nh,
								controller_manager::ControllerManager& cm,
								ros::Rate& loop_rate);
void init_left_arm_and_start_controllers(ros::NodeHandle& nh,
										controller_manager::ControllerManager& cm,
										CepheusHW& robot,
										ros::Publisher left_shoulder_pub,
										ros::Publisher left_elbow_pub,
										ros::Rate& loop_rate);
void init_right_arm_and_start_controllers(ros::NodeHandle& nh,
										controller_manager::ControllerManager& cm,
										CepheusHW& robot,
										ros::Publisher right_shoulder_pub,
										ros::Publisher right_elbow_pub,
										ros::Rate& loop_rate);
// 2020 keep it simple
void init_left_elbow_and_start_controller(ros::NodeHandle& nh,
										controller_manager::ControllerManager& cm,
										CepheusHW& robot,
										ros::Publisher left_elbow_pub,
										ros::Rate& loop_rate);
void init_left_shoulder_and_start_controller(ros::NodeHandle& nh,
										controller_manager::ControllerManager& cm,
										CepheusHW& robot,
										ros::Publisher left_shoulder_pub,
										ros::Rate& loop_rate);
void init_right_elbow_and_start_controller(ros::NodeHandle& nh,
										controller_manager::ControllerManager& cm,
										CepheusHW& robot,
										ros::Publisher right_elbow_pub,
										ros::Rate& loop_rate);
//-------------------------------------------------------------------------------------------



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




void thrusterCallback(const geometry_msgs::Vector3Stamped::ConstPtr& cmd)
{
	double thrust[4];
	thrust[0] = (double)cmd->vector.x;
	thrust[1] = (double)cmd->vector.y;
	thrust[2] = (double)cmd->vector.z;
	thrust[3] = (double)0;
	//ROS_WARN("%lf %lf %lf", thrust[0], thrust[1], thrust[2]);
	robot.setThrustPwm(thrust, 0.001, 0.9);
	return;
}


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
	//ROS_WARN("left fsr val : %d",cmd->data);
	robot.set_left_fsr_value(cmd->data);
}


void rightFsrCallback(const std_msgs::UInt8::ConstPtr& cmd)
{
	//ROS_WARN("right fsr val : %d",cmd->data);
	robot.set_right_fsr_value(cmd->data);
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
	ROS_WARN("CMD GRIP %lf",cmd->data);
	if(cmd->data >=0 && cmd->data <= LEFT_FINGER_MAX_ANGLE){
		robot.setCmd(10, (double)cmd->data);
		//robot.writeMotors();
		//robot.heartbeat();
	}
	else
		ROS_WARN("Cmd to left gripper out of bounds!");

}

void leftGripperActionCallback(const std_msgs::Bool::ConstPtr& cmd)
{
	if(cmd->data){
		//ROS_WARN("ORDERED TO CLOSE LEFT_GRIPPER");
		ready_to_grip_left = true;
	}
	else{
		//ROS_WARN("ORDERED TO OPEN LEFT_GRIPPER");
		ready_to_grip_left = false;
		left_gripper_first_time = true;

		robot.init_left_finger();
	}

}

void rightGripperActionCallback(const std_msgs::Bool::ConstPtr& cmd)
{
	if(cmd->data){
		//ROS_WARN("ORDERED TO CLOSE RIGHT_GRIPPER");
		ready_to_grip_right = true;
	}
	else{
		//ROS_WARN("ORDERED TO OPEN RIGHT_GRIPPER");
		ready_to_grip_right = false;
		right_gripper_first_time = true;

		robot.init_right_finger();
	}
}


void setReactionWheelEffort(const std_msgs::Float64::ConstPtr& cmd)
{
	robot.setCmd(REACTION_WHEEL, cmd->data);
	robot.writeMotors();
}

void setLeftShoulderEffort(const std_msgs::Float64::ConstPtr& cmd)
{
	robot.setCmd(LEFT_SHOULDER, cmd->data);
	// ROS_INFO("GOT EFFORT FOR LEFT SHOULDER: %f", cmd->data);
	robot.writeMotors();
}

void setLeftElbowEffort(const std_msgs::Float64::ConstPtr& cmd)
{
	robot.setCmd(LEFT_ELBOW, cmd->data);
	// ROS_INFO("GOT EFFORT FOR LEFT ELBOW: %f", cmd->data);
	robot.writeMotors();
}

void setRightElbowEffort(const std_msgs::Float64::ConstPtr& cmd)
{
	robot.setCmd(RIGHT_ELBOW, cmd->data);
	// ROS_INFO("GOT EFFORT FOR RIGHT ELBOW: %f", cmd->data);
	robot.writeMotors();
}


void setLeftShoulderOffset(const std_msgs::Float64::ConstPtr& cmd)
{
	robot.setOffset(LEFT_SHOULDER, cmd->data);
	// ROS_INFO("GOT Offset FOR LEFT SHOULDER: %f", cmd->data);
	robot.writeMotors();
}

void setLeftElbowOffset(const std_msgs::Float64::ConstPtr& cmd)
{
	robot.setOffset(LEFT_ELBOW, cmd->data);
	// ROS_INFO("GOT Offset FOR LEFT ELBOW: %f", cmd->data);
	robot.writeMotors();
}

void setRightElbowOffset(const std_msgs::Float64::ConstPtr& cmd)
{
	robot.setOffset(RIGHT_ELBOW, cmd->data);
	// ROS_INFO("GOT Offset FOR RIGHT ELBOW: %f", cmd->data);
	robot.writeMotors();
}


//Trend line transforming force to d_theta for gripper
double fsr_trend_line(bool positive ,double pi_out)
{
	double rv = 0;
	rv = 0.25 * (double)pow(pi_out,3) - (double)4 * (double)pow(pi_out,2) + 20.4 * pi_out + 0.04;

	if(positive)
		return rv;
	else
		return -rv;
}

//Pnagiotis Mavridis
//PI controller for left gripper (force controll)
void left_fsr_update()
{
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

	if (left_gripper_first_time) {
		gripper_last_angle = (double)LEFT_FINGER_MAX_ANGLE;
		error_sum = 0;
		count = 0;
		init_time = ros::Time::now();
		left_gripper_first_time = false;
	}

	int8_t fsr_val = robot.get_left_fsr_val();
	int8_t error = fsr_val - (int8_t)F_DES ;

	if (error < -1 || error > 1) {
		//PI out (Kp * error + Ki * sum(error))
		error_sum += error;
		count ++;
		double avg = (double)error_sum/(double)count;
		ROS_WARN("AVG = %lf error_sum %d count %d",avg,error_sum, count);
		//For protection from overheating, if the fsr does not sense the target as expected ,so the error is not decreasing
		dur =  ros::Time::now() - init_time;
		if (dur.toSec() >= (double)DURATION_NO_GRIP && avg <= (double) -FSR_AVG_THRESHOLD ) {
			//open and try again
			robot.init_left_finger();
			sleep(1);
			init_time = ros::Time::now();
			error_sum = 0;
			count = 0;
			left_gripper_first_time = true;
			ready_to_grip_left = false;
		}

		//uint8_t pi_out = KP * error + KI * error_sum;
		pi_out = (double)KP * (double)error;

		//tranpose force to width in order to give the command
		if (fsr_val == 0) {
			d_theta = 1;
		}
		else if (fsr_val > 0 && fsr_val < F_DES){
			d_theta = fsr_trend_line(true, abs(pi_out));
		}
		else if (fsr_val >= F_DES) {
			d_theta = fsr_trend_line(false, abs(pi_out));
		}

		//holding the last position of the gripper
		gripper_last_angle -= d_theta;
		new_angle = gripper_last_angle;

		robot.setCmd(LEFT_GRIPPER, new_angle);

		ROS_WARN("FSR: %d, error: %d, PI_OUT: %lf, d_theta: %lf, new_angle: %lf", fsr_val, error, pi_out, d_theta, new_angle);
	}
}


void right_fsr_update()
{
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

	if(right_gripper_first_time){
		gripper_last_angle = (double)RIGHT_FINGER_MAX_ANGLE;
		error_sum = 0;
		count = 0;
		init_time = ros::Time::now();
		right_gripper_first_time = false;
	}

	int8_t fsr_val = robot.get_right_fsr_val();
	int8_t error = fsr_val - (int8_t)F_DES ;

	if(error < -1 || error > 1){
		//PI out (Kp * error + Ki * sum(error))
		error_sum += error;
		count ++;
		double avg = (double)error_sum/(double)count;
		//ROS_WARN("AVG = %lf error_sum %d count %d",avg,error_sum, count);
		//For protection from overheating, if the fsr does not sense the target as expected ,so the error is not decreasing
		dur =  ros::Time::now() - init_time;
		if (dur.toSec() >= (double)DURATION_NO_GRIP && avg <= (double) - FSR_AVG_THRESHOLD) {
			//open and try again
			robot.init_right_finger();
			sleep(1);
			init_time = ros::Time::now();
			error_sum = 0;
			count = 0;
			right_gripper_first_time = true;
			ready_to_grip_right = false;
			}

		//uint8_t pi_out = KP * error + KI * error_sum;
		pi_out = (double)KP * (double)error;

		//tranpose force to width in order to give the command
		if (fsr_val == 0) {
			d_theta = 1;
		}
		else if (fsr_val > 0 && fsr_val < F_DES) {
			d_theta = fsr_trend_line(true, abs(pi_out));
		}
		else if (fsr_val >= F_DES) {
			d_theta = fsr_trend_line(false, abs(pi_out));
		}

		//holding the last position of the gripper
		gripper_last_angle -= d_theta;
		new_angle = gripper_last_angle;

		robot.setCmd(RIGHT_GRIPPER, new_angle);

		//ROS_WARN("FSR: %d, error: %d, PI_OUT: %lf, d_theta: %lf, new_angle: %lf", fsr_val, error, pi_out, d_theta, new_angle);
	}
}

bool zero_called = false;

void leftElbowGoTo(const std_msgs::Bool::ConstPtr &msg,
		controller_manager::ControllerManager &cm)
{
	if(!zero_called){
		move_left_arm(0.5, 1.0, 110.0, 12.0, cm, robot, left_shoulder_pub, left_elbow_pub);
			// zero_called = true;
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

	double l1_limit_pos, l2_limit_pos, r1_limit_pos, r2_limit_pos;
	double max_thrust;
	double rw_max_torque, rw_max_speed, rw_max_power, rw_total_inertia;

	//in order to be able to know if feedback about the succesfull grip of the target have to be sent back to planner
	//exists because this node have to be able to operate correctly with or without a planner
	bool use_with_chase_planner;

	ros::param::param<double>("~thruster_force", max_thrust, 1.5); //the thrust of an open thruster in Newtons
	//ros::param::param<double>("~max_motor_current", max_cur, 1.72); //the max current of the motor
	ros::param::param<double>("~rw_max_torque", rw_max_torque, 0.5);
	ros::param::param<double>("~rw_max_speed",  rw_max_speed, 100);
	ros::param::param<double>("~rw_max_power",  rw_max_power, 60);
	ros::param::param<double>("~rw_total_inertia", rw_total_inertia, 0.00197265);

	ros::param::param<double>("~left_shoulder_limit_pos", l1_limit_pos, 2.4);
	ros::param::param<double>("~left_elbow_limit_pos", l2_limit_pos, 1.4);
	ros::param::param<double>("~right_shoulder_limit_pos", r1_limit_pos, -2.4);
	ros::param::param<double>("~right_elbow_limit_pos", r2_limit_pos, -1.4);

	ros::param::param<bool>("~use_with_chase_planner", use_with_chase_planner, false);

	//--------Panagiotis Mavridis 25/04/2018----------------

	struct sched_param schedParam;
	schedParam.sched_priority = RT_PRIORITY;
	int sched_policy = SCHED_RR;
	sched_setscheduler(0, sched_policy, &schedParam);



	double max_cur[8];
	max_cur[0] = 1.72;
	// 2020 updates, prev: 3.1 all (1 to 7)
	max_cur[1] = 3.31;
	max_cur[2] = 3.31;
	max_cur[3] = 3.31;
	max_cur[4] = 3.31;
	max_cur[5] = 3.31;
	max_cur[6] = 3.31;
	max_cur[7] = 3.31;

	robot.setParam(max_cur, max_thrust);


	controller_manager::ControllerManager cm(&robot);

	ros::Subscriber thrust_sub =  nh.subscribe("cmd_thrust", 1, thrusterCallback);
	ros::Subscriber torque_sub =  nh.subscribe("cmd_torque", 1, torqueCallback);

	//For reading the fsrs from the grippers
	ros::Subscriber left_fsr_sub =  nh.subscribe("left_fsr", 1, leftFsrCallback);
	ros::Subscriber right_fsr_sub =  nh.subscribe("right_fsr", 1, rightFsrCallback);

	//For giving cmds to the left wrist and gripper if nesessary
	ros::Subscriber left_wrist_sub =  nh.subscribe("left_wrist_cmd", 1, leftWristCallback);
	ros::Subscriber left_gripper_sub =  nh.subscribe("left_gripper_cmd", 1, leftGripperCallback);

	//ros::Subscriber move_left_arm_sub =  nh.subscribe<std_msgs::Float64MultiArray>("move_left_arm", 1, boost::bind(&moveLeftArmCallback, _1,  boost::ref(cm)));

	ros::Subscriber left_gripper_action_sub =  nh.subscribe("left_gripper_action", 1, leftGripperActionCallback);
	ros::Subscriber right_gripper_action_sub =  nh.subscribe("right_gripper_action", 1, rightGripperActionCallback);

	//ros::Subscriber left_arm_catch_object_sub =  nh.subscribe<geometry_msgs::PointStamped>("left_arm_catch_object", 1, boost::bind(&leftArmCatchObjectCallback, _1, boost::ref(cm)));
	//ros::Subscriber right_arm_catch_object_sub =  nh.subscribe<geometry_msgs::PointStamped>("right_arm_catch_object", 1, boost::bind(&rightArmInvKinCallback, _1, boost::ref(cm)));

	//Action for fripping test
	//ActionServerRightArm as_right (nh, "right_catch_object_action", boost::bind(&rightArmInvKinCallback, _1, boost::ref(as_right), boost::ref(cm)), false);
	//as_right.start();

	//ActionServerLeftArm as_left (nh, "left_catch_object_action", boost::bind(&leftArmCatchObjectCallback, _1, boost::ref(as_left), boost::ref(cm)), false);
	//as_left.start();


	//ros::Publisher  torque_pub =  nh.advertise<std_msgs::Float64>("reaction_wheel_velocity_controller/command", 1);
	ros::Publisher  torque_pub =  nh.advertise<std_msgs::Float64>("reaction_wheel_effort_controller/command", 1);

	//Puplishers to ROS-contol topics
//	left_shoulder_pub =  nh.advertise<std_msgs::Float64>("left_shoulder_position_controller/command", 1000);
//	left_elbow_pub =  nh.advertise<std_msgs::Float64>("left_elbow_position_controller/command", 1000);
//	right_shoulder_pub =  nh.advertise<std_msgs::Float64>("right_shoulder_position_controller/command", 1000);
//	right_elbow_pub =  nh.advertise<std_msgs::Float64>("right_elbow_position_controller/command", 1000);

	ros::Publisher ctl_pub = nh.advertise<std_msgs::String>("load_start_controllers", 10);


	ros::Time prev_time = ros::Time::now();

	robot.setHomePos(4, l1_limit_pos);
	robot.setHomePos(5, l2_limit_pos);
	robot.setHomePos(6, r1_limit_pos);
	robot.setHomePos(7, r2_limit_pos);

	// int err = readErr();
	// if (err) {
	// fixJointPos();
	// ROS_INFO("Fixed, now init...\n");
	// }

	//Initialize the  arms and start the ros controllers

	// 2020
	ros::Subscriber set_reaction_wheel_effort = nh.subscribe("set_reaction_wheel_effort", 1, setReactionWheelEffort);
	ros::Subscriber set_left_shoulder_effort = nh.subscribe("set_left_shoulder_effort", 1, setLeftShoulderEffort);
	ros::Subscriber set_left_elbow_effort = nh.subscribe("set_left_elbow_effort", 1, setLeftElbowEffort);
	ros::Subscriber set_right_elbow_effort =  nh.subscribe("set_right_elbow_effort", 1, setRightElbowEffort);
	ros::Subscriber set_left_shoulder_offset = nh.subscribe("set_left_shoulder_offset", 1, setLeftShoulderOffset);
	ros::Subscriber set_left_elbow_offset = nh.subscribe("set_left_elbow_offset", 1, setLeftElbowOffset);
	ros::Subscriber set_right_elbow_offset =  nh.subscribe("set_right_elbow_offset", 1, setRightElbowOffset);

	ros::Publisher ls_pos_pub = nh.advertise<std_msgs::Float64>("read_left_shoulder_position", 1);
	ros::Publisher le_pos_pub = nh.advertise<std_msgs::Float64>("read_left_elbow_position", 1);
	ros::Publisher re_pos_pub = nh.advertise<std_msgs::Float64>("read_right_elbow_position", 1);
	ros::Publisher rw_pos_pub = nh.advertise<std_msgs::Float64>("read_reaction_wheel_position", 1);
	ros::Publisher ls_vel_pub = nh.advertise<std_msgs::Float64>("read_left_shoulder_velocity", 1);
	ros::Publisher le_vel_pub = nh.advertise<std_msgs::Float64>("read_left_elbow_velocity", 1);
	ros::Publisher re_vel_pub = nh.advertise<std_msgs::Float64>("read_right_elbow_velocity", 1);
	ros::Publisher rw_vel_pub = nh.advertise<std_msgs::Float64>("read_reaction_wheel_velocity", 1);
	ros::Publisher ls_limit_pub = nh.advertise<std_msgs::UInt8>("read_left_shoulder_limit", 1);
	ros::Publisher le_limit_pub = nh.advertise<std_msgs::UInt8>("read_left_elbow_limit", 1);
	ros::Publisher re_limit_pub = nh.advertise<std_msgs::UInt8>("read_right_elbow_limit", 1);

	start_standard_controllers(nh, cm, loop_rate);
	// init_left_arm_and_start_controllers(nh, cm, robot, left_shoulder_pub, left_elbow_pub, loop_rate);
	// init_right_arm_and_start_controllers(nh, cm, robot, right_shoulder_pub, right_elbow_pub, loop_rate);

	sleep(1);
	// Move Right Hand to Ready to Grab Position
	// move_right_arm(-M_PI/2.0, 2.0 * M_PI/3.0, 110.0, 12.0, cm, robot, right_shoulder_pub, right_elbow_pub);
	// move_right_arm(M_PI, M_PI/3.0, 110.0, 30.0, cm, robot, right_shoulder_pub, right_elbow_pub);
	// move_left_arm(M_PI, M_PI/2.0, 110.0, 24.0, cm, robot, left_shoulder_pub, left_elbow_pub);

	// ros::ServiceServer init_first_and_third_joints_service = nh.advertiseService("init_first_and_third_joints", initFirstAndThirdJoints);
	// ros::ServiceServer init_first_joint_service = nh.advertiseService("init_first_joint", initFirstJoint);
	// ros::ServiceServer init_second_joint_service = nh.advertiseService("init_second_joint", initSecondJoint);
	// ros::ServiceServer init_third_joint_service = nh.advertiseService("init_third_joint", initThirdJoint);
	// ros::Subscriber go_to_zero_service = nh.subscribe<std_msgs::Bool>("left_elbow_go_to", 1, boost::bind(&leftElbowGoTo, _1, boost::ref(cm)));


	ROS_WARN("About to enter normal spinning...");
	ros::AsyncSpinner spinner(3);
	spinner.start();

	ros::Time curr_time;
	ros::Duration time_step;

	bool first_time = true;

	int a = 120;
	std_msgs::Float64 robot_info_msg;
	std_msgs::UInt8 limit_msg;

	while(!g_request_shutdown) {

		curr_time = ros::Time::now();

		if (first_time) {
			prev_time = curr_time;
			first_time = false;
			//** start - 2020 pelekoudas changes **//
			// std_msgs::Float64 stay_pos;

//			init_left_elbow_and_start_controller(nh, cm, robot, left_elbow_pub, loop_rate);
/*			robot.readEncoders(time_step);
			stay_pos.data = robot.getPos(LEFT_ELBOW);
			left_elbow_pub.publish(stay_pos);
			robot.writeMotors();
*/
//			init_right_elbow_and_start_controller(nh, cm, robot, right_elbow_pub, loop_rate);
/*			robot.readEncoders(time_step);
			stay_pos.data = robot.getPos(RIGHT_ELBOW);
			right_elbow_pub.publish(stay_pos);
			stay_pos.data = robot.getPos(LEFT_ELBOW);
			left_elbow_pub.publish(stay_pos);
			robot.writeMotors();
*/
//			init_left_shoulder_and_start_controller(nh, cm, robot, left_shoulder_pub, loop_rate);
/*			robot.readEncoders(time_step);
			stay_pos.data = robot.getPos(LEFT_SHOULDER);
			left_shoulder_pub.publish(stay_pos);
			stay_pos.data = robot.getPos(RIGHT_ELBOW);
			right_elbow_pub.publish(stay_pos);
			stay_pos.data = robot.getPos(LEFT_ELBOW);
			left_elbow_pub.publish(stay_pos);
			robot.writeMotors();
*/			// move_left_arm(0.0, 1.0, 110.0, 12.0, cm, robot, left_shoulder_pub, left_elbow_pub);
			// move_right_arm(0.0, 0.0, 110.0, 6.0, cm, robot, right_shoulder_pub, right_elbow_pub);
			// robot.writeMotors();

/*			std_msgs::String msg;
			msg.data = std::string(CMD_SWITCH_TO_EFFORT);
			ctl_pub.publish(msg);
*/			//** end - 2020 pelekoudas changes **//
		}

		time_step = curr_time - prev_time;
		prev_time = curr_time;

		//ros::spinOnce();

		robot.readEncoders(time_step);
		cm.update(curr_time, time_step);

		robot_info_msg.data = robot.getPos(LEFT_SHOULDER);
		ls_pos_pub.publish(robot_info_msg);
		robot_info_msg.data = robot.getPos(LEFT_ELBOW);
		le_pos_pub.publish(robot_info_msg);
		robot_info_msg.data = robot.getPos(RIGHT_ELBOW);
		re_pos_pub.publish(robot_info_msg);
		robot_info_msg.data = robot.getPos(REACTION_WHEEL);
		rw_pos_pub.publish(robot_info_msg);
		robot_info_msg.data = robot.getVel(LEFT_SHOULDER);
		ls_vel_pub.publish(robot_info_msg);
		robot_info_msg.data = robot.getVel(LEFT_ELBOW);
		le_vel_pub.publish(robot_info_msg);
		robot_info_msg.data = robot.getVel(RIGHT_ELBOW);
		re_vel_pub.publish(robot_info_msg);
		robot_info_msg.data = robot.getVel(REACTION_WHEEL);
		rw_vel_pub.publish(robot_info_msg);

		limit_msg.data = robot.isLimitReached(LEFT_SHOULDER);
		ls_limit_pub.publish(limit_msg);
		limit_msg.data = robot.isLimitReached(LEFT_ELBOW);
		le_limit_pub.publish(limit_msg);
		limit_msg.data = robot.isLimitReached(RIGHT_ELBOW);
		re_limit_pub.publish(limit_msg);
		/*
		if(ready_to_grip_left)
			left_fsr_update();

		if(ready_to_grip_right)
					right_fsr_update();
		*/

		robot.writeMotors();
		robot.heartbeat();

		//std::cout<<"rw_torque : "<<rw_torque<<std::endl;

		if (rw_torque!=0.0) {
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



//-------------------------------------------
//----------------------------------------------------------------


/*
//----Create trajectory (given a set_point) for the left arm
void moveLeftArmCallback(const std_msgs::Float64MultiArray::ConstPtr& cmd_array,
		controller_manager::ControllerManager& cm){

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

		robot.readEncoders(timer);
		cm.update(ros::Time::now(), timer);
		robot.writeMotors();
		robot.heartbeat();

		ros::spinOnce();


		timer = ros::Time::now() - init_time;
		loop_rate.sleep();
	}

}
*/
//---------------------------------------------------------------------------------------
//bool left_catch_object_one_time = true;
//bool right_catch_object_one_time = true;

// Inverse kinematics...calculate the set points of the joints of the left arm given an angle that you want ot catch a target
/*
void leftArmCatchObjectCallback(const cepheus_robot::LeftCatchObjectGoalConstPtr& goal, ActionServerLeftArm& as ,controller_manager::ControllerManager& cm ){


		//theta2 needs to be from -90 deg to 0 deg
		//TO DO...ADD COMMMEEEEENTS

		//The lengths of the joint of the left arm
		double l1 = 0.181004;	//shoulder
		double l2 = 0.1605;	//elbow
		double l3 = 0.0499;	//wrist

		//Used in order to transform the point to grip inj the left hand base, in order to calculate inverse kinematics

		tf::TransformListener listener;
		geometry_msgs::PointStamped transform;

		try{
			listener.transformPoint("/left_hand_base", goal->point_to_catch, transform);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}


		//Coordinates of target based on cepheus origin
		//double x = 0.21;
		//double y = -0.1;

		//some cm before the target
		//double x = transform.getOrigin().x() - 0.06;
		//double y = transform.getOrigin().y();
		double x = transform.point.x - 0.06;
				double y = transform.point.y;



		//the desired angle of the wrist translated to cepheus coordinates
		//double roll, pitch, yaw;
		//tf::Quaternion q(transform.getRotation());
		//tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
		//double yaw_to_deg = yaw*180.0/M_PI;
		//ROS_WARN("yaw_to_deg %lf",yaw_to_deg);

		//to rad
		//double phi = ((yaw_to_deg + 270.0)/180.0) * M_PI;



		double phi = atan2(y, x);

		//Need to transform the above coordinates to the base of the left arm
		//x = x - 0.17268;
		//y = y + 0.091404;

		double q31 ,q32;
		q31 = q32 = 0.0;

		//double yn = y - l3  * cos(phi);
		//double xn = x - l3 * sin(phi);
		double yn = y;
		double xn = x - l3;

		//calculating 2 results for each angle

		//For q2
		double q21 = acos( ((double)pow(yn, 2) + (double)pow(xn, 2) - (double)pow(l2, 2) - (double)pow(l1, 2)) / (2.0 * l1 * l2));
		double q22 = -q21;

		//For q1

		//q11
		double a = -l1 - l2 * cos(q21);
		double b = -l2 * sin(q21);
		double c = -yn;
		double d = -l2 * sin(q21);
		double e = l1 + l2 * cos(q21);
		double f = -xn;

		double s11 = ((c/a) - (f/d)) / ((e/d) - (b/a));
		double c11 = (b*s11/a)+ (c/a);

		double q11 = atan2(s11, c11);

		//q12
		a = -l1 - l2 * cos(q22);
		b = -l2 * sin(q22);
		c = -yn;
		d = -l2 * sin(q22);
		e = l1 + l2 * cos(q22);
		f = -xn;

		double s12 = ((c/a) - (f/d))/((e/d) - (b/a));
		double c12 = (b * s12/a) + (c/a);

		double q12 = atan2(s12, c12);

		//double q31 = phi - q11 - q21;
		//double q32 = phi - q12 - q22;

		double tuple1[3];
		double tuple2[3];

		tuple1[0] = q11;
		tuple1[1] = q21;
		tuple1[2] = q31;

		tuple2[0] = q12;
		tuple2[1] = q22;
		tuple2[2] = q32;


		ROS_WARN("q11= %lf,  q21= %lf,  q31= %lf",q11,q21,q31);
		ROS_WARN("q12= %lf,  q22= %lf,  q32= %lf",q12,q22,q32);

		//checking the results in order to discard odd angles
		if( (-M_PI/3 <= q31 && q31 <= M_PI/3.0) && (-2.0*M_PI/3.0 <= q21 && q21 <= 2.0*M_PI/3.0) && (M_PI/2.0 <= q11 && q11 <= M_PI) ){
			//solution is tuple1
			//translate wrist cmd  to (0 to 120)
			q31 = abs((q31 * 180.0 / M_PI) + 60.0);
			q11 = q11 - M_PI/2.0;

			ROS_WARN("Solution, q1= %lf,  q2= %lf,  q3= %lf",q11,q21,q31);
			move_left_arm(q11, q21, q31, 12.0, cm, robot, left_shoulder_pub, left_elbow_pub);

			//in order to invoke the fsr update callback in the master loop
			ready_to_grip_left = true;
			as.setSucceeded();
		}
		else if((-M_PI/3.0 <= q32 && q32 <= M_PI/3.0) && (-2.0 * M_PI/3.0 <= q22 && q22 <= 2.0*M_PI/3.0) && (M_PI/2.0  <= q12 && q12 <= M_PI)){
			//solution is tuple2
			q32 = abs((q32 * 180.0 / M_PI) + 60.0);
			q12 = q12 - M_PI/2.0;

			ROS_WARN("Solution, q1= %lf,  q2= %lf,  q3= %lf",q12,q22,q32);
			move_left_arm(q12, q22, q32, 12.0, cm, robot, left_shoulder_pub, left_elbow_pub);

			//in order to invoke the fsr update callback in the master loop
						ready_to_grip_left = true;
			as.setSucceeded();
		}
		else{
			//we see.....
			ROS_WARN("Out of left arm workspace");
		}

}
*/

/*
void rightArmCatchObjectCallback(const cepheus_robot::RightCatchObjectGoalConstPtr& goal, ActionServerRightArm& as ,controller_manager::ControllerManager& cm ){
   
	
		cepheus_robot::RightCatchObjectResult result;
				
		//theta2 needs to be from -90 deg to 0 deg
				//TO DO...ADD COMMMEEEEENTS

				//The lengths of the joint of the left arm
				double l1 = 0.181004;   //shoulder
				double l2 = 0.1605;     //elbow
				double l3 = 0.069;     //wrist

				//Used in order to transform the point to grip inj the left hand base, in order to calculate inverse kinematics

				tf::TransformListener listener;
				geometry_msgs::PoseStamped transform;

				try{
			sleep(5);
						listener.transformPose("/right_hand_base", goal->point_to_catch, transform);
				}
				catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
						ros::Duration(1.0).sleep();
				}

				double x = transform.pose.position.x - 0.06;
				double y = transform.pose.position.y;

				double phi = atan2(y, x);

				//Need to transform the above coordinates to the base of the left arm
				//x = x - 0.17268;
				//y = y + 0.091404;

				double q31 ,q32;
				q31 = q32 = 0.0;

				//double yn = y - l3  * cos(phi);
				//double xn = x - l3 * sin(phi);
				double yn = y;
				double xn = x - l3;


				//calculating 2 results for each angle



				//For q2
				double q21 = acos( ((double)pow(yn, 2) + (double)pow(xn, 2) - (double)pow(l2, 2) - (double)pow(l1, 2)) / (2.0 * l1 * l2));
				double q22 = -q21;

				//For q1

				//q11
				double a = -l1 - l2 * cos(q21);
				double b = -l2 * sin(q21);
				double c = -yn;
				double d = -l2 * sin(q21);
				double e = l1 + l2 * cos(q21);
				double f = -xn;

				double s11 = ((c/a) - (f/d)) / ((e/d) - (b/a));
				double c11 = (b*s11/a)+ (c/a);

				double q11 = atan2(s11, c11);

				//q12
				a = -l1 - l2 * cos(q22);
				b = -l2 * sin(q22);
				c = -yn;
				d = -l2 * sin(q22);
				e = l1 + l2 * cos(q22);
				f = -xn;

				double s12 = ((c/a) - (f/d))/((e/d) - (b/a));
				double c12 = (b * s12/a) + (c/a);

				double q12 = atan2(s12, c12);

				//double q31 = phi - q11 - q21;
				//double q32 = phi - q12 - q22;

				double tuple1[3];
				double tuple2[3];

				tuple1[0] = q11;
				tuple1[1] = q21;
				tuple1[2] = q31;


				tuple2[0] = q12;
				tuple2[1] = q22;
				tuple2[2] = q32;


				ROS_WARN("q11= %lf,  q21= %lf,  q31= %lf",q11,q21,q31);
				ROS_WARN("q12= %lf,  q22= %lf,  q32= %lf",q12,q22,q32);


				//checking the results in order to discard odd angles
				if( (-M_PI/3 <= q31 && q31 <= M_PI/3.0) && (-2.0*M_PI/3.0 <= q21 && q21 <= 2.0*M_PI/3.0) && (-M_PI <= q11 && q11 <= -M_PI/2.0) ){
						//solution is tuple1
						//translate wrist cmd  to (0 to 120)
						q31 = abs((q31 * 180.0 / M_PI) + 60.0);
						q11 = q11 - M_PI/2.0;

						ROS_WARN("Solution, q1= %lf,  q2= %lf,  q3= %lf",q11,q21,q31);
						//move_right_arm(q11, q21, q31, 12.0, cm, robot, right_shoulder_pub, right_elbow_pub);

			//in order to invoke the fsr update callback in the master loop
						ready_to_grip_right = true;
			result.success = true;
			as.setSucceeded(result);

				}
				else if((-M_PI/3.0 <= q32 && q32 <= M_PI/3.0) && (-2.0 * M_PI/3.0 <= q22 && q22 <= 2.0*M_PI/3.0) && (-M_PI  <= q12 && q12 <= -M_PI/2.0)){
						//solution is tuple2
						q32 = abs((q32 * 180.0 / M_PI) + 60.0);
						q12 = q12 - M_PI/2.0;

						ROS_WARN("Solution, q1= %lf,  q2= %lf,  q3= %lf",q12,q22,q32);
						//move_right_arm(q12, q22, q32, 12.0, cm, robot, right_shoulder_pub, right_elbow_pub);

			//in order to invoke the fsr update callback in the master loop
						ready_to_grip_right = true;
			result.success = true;
						as.setSucceeded(result);

				}
				else{
			result.success = false;
						as.setSucceeded(result);

						//we see.....
						ROS_WARN("Out of right arm workspace");
				}


}
*/
//bool right_catch_object_one_time = true;
/*
void rightArmInvKinCallback(const geometry_msgs::PointStamped::ConstPtr& point, controller_manager::ControllerManager& cm){
//void rightArmInvKinCallback(const cepheus_robot::RightCatchObjectGoalConstPtr& goal, ActionServerRightArm& as ,controller_manager::ControllerManager& cm){

		cepheus_robot::RightCatchObjectResult result;

		//if(right_catch_object_one_time){
				//right_catch_object_one_time = false;

				//theta2 needs to be from -90 deg to 0 deg
				//TO DO...ADD COMMMEEEEENTS

				//The lengths of the joint of the left arm
				double l1 = 0.181004;   //shoulder
				double l2 = 0.1605;     //elbow
				double l3 = 0.069;     //wrist
		double CIRCLE_RADIUS = 0.4;

				//Used in order to transform the point to grip inj the left hand base, in order to calculate inverse kinematics

				tf::TransformListener listener;
		tf::StampedTransform transform;

				try{
			listener.waitForTransform("/right_hand_base", "/assist_robot", ros::Time(0), ros::Duration(2.0));
						listener.lookupTransform("/right_hand_base", "/assist_robot", ros::Time(0),transform);
				}
				catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
						ros::Duration(1.0).sleep();
				}


				//some cm before the target
				double x = transform.getOrigin().x();
				double y = transform.getOrigin().y();

		double phi = atan2(y, x);

		x = x - CIRCLE_RADIUS * cos(phi);
		y = y - CIRCLE_RADIUS * sin(phi);

				double yn = y - l3  * cos(phi);
				double xn = x - l3 * sin(phi);

		std::cout<<"x "<<x<<" y "<<y<<" phi "<<phi<<std::endl;
				//calculating 2 results for each angle

				//For q2
				double q21 = acos( ((double)pow(yn, 2) + (double)pow(xn, 2) - (double)pow(l2, 2) - (double)pow(l1, 2)) / (2.0 * l1 * l2));
				double q22 = -q21;


				//For q1

		//For q1
		double theta = atan2(yn, xn);
		ROS_WARN("theta %lf",theta);

		double R = l2 * sin(q21);
		double S = l1 + l2 * cos(q21);
		double psi = atan2(R, S);

		double q11 = theta - psi;
		double q12 = theta + psi;

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


				ROS_WARN("q11= %lf,  q21= %lf,  q31= %lf",q11,q21,q31);
				ROS_WARN("q12= %lf,  q22= %lf,  q32= %lf",q12,q22,q32);

				//checking the results in order to discard odd angles
				if( (-M_PI/3.0 <= q31 && q31 <= M_PI/3.0) && (-2.0*M_PI/3.0 <= q21 && q21 <= 2.0*M_PI/3.0) && (- 2.0 * M_PI/3.0 <= q11 && q11 <= 0.0) ){
						//solution is tuple1
						//translate wrist cmd  to (0 to 120)
						q31 = abs((q31 * 180.0 / M_PI) - 60.0);
						//q11 = q11 - M_PI/2.0;

						ROS_WARN("Solution, q1= %lf,  q2= %lf,  q3= %lf",q11,q21,q31);

			move_right_arm(q11, q21, q31, 3.0, cm, robot, right_shoulder_pub, right_elbow_pub);
			ready_to_grip_right = true;
						result.success = true;
						as.setSucceeded(result);

				}
				else if((-M_PI/3.0 <= q32 && q32 <= M_PI/3.0) && (-2.0 * M_PI/3.0 <= q22 && q22 <= 2.0*M_PI/3.0) && (- 2.0 * M_PI / 3.0 <= q12 && q12 <= 0.0)){
						//solution is tuple2
						q32 = abs((q32 * 180.0 / M_PI) - 60.0);
						//q12 = q12 - M_PI/2.0;

						ROS_WARN("Solution, q1= %lf,  q2= %lf,  q3= %lf",q12,q22,q32);

						move_right_arm(q12, q22, q32, 3.0, cm, robot, right_shoulder_pub, right_elbow_pub);
			ready_to_grip_right = true;
						result.success = true;
						as.setSucceeded(result);

				}
				else{
						//we see.....

						ROS_WARN("Out of right arm workspace");
						result.success = false;
						as.setSucceeded(result);

				}

		//}
}
*/
