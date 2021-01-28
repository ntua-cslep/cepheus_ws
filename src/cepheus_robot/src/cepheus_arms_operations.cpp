#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "cepheus_hardware.h"


double
produce_trajectory_point(double time,
								double movement_duration,
								double init_pos,
								double set_point)
{
	double tj_p;
	double a0 = init_pos;
	double a1 = 0.0;
	double a2 = (3.0/(double)pow(movement_duration,2)) * (double)(set_point - init_pos);
	double a3 = - (2.0/(double)pow(movement_duration,3)) * (double)(set_point - init_pos);

	tj_p = a0 + a1 * time + a2 * (double)pow(time,2) + a3 * (double)pow(time,3);
	// ROS_INFO("trajectory value: %f", tj_p);
	return tj_p;
}


double
produce_trajectory_point_wrist(double time,
									double movement_duration,
									double init_pos,
									double set_point)
{

	if (set_point >= LEFT_WRIST_MIN_ANGLE && set_point <= LEFT_WRIST_MAX_ANGLE) {
		double tj_p;
		double a0 = init_pos;
		double a1 = 0.0;
		double a2 = (3.0/(double)pow(movement_duration,2)) * (double)(set_point - init_pos);
		double a3 = - (2.0/(double)pow(movement_duration,3)) * (double)(set_point - init_pos);

		tj_p = a0 + a1 * time + a2 * (double)pow(time,2) + a3 * (double)pow(time,3);
		return tj_p;
	}
	else
		ROS_WARN("Command to left wrist out of range");
}

double
produce_sin_trajectory(double width, double period, double t){

	double rv = width * sin((2.0 * M_PI) / period * t);
	return rv;
}

double
produce_sin_trajectory_wrist(double width, double period, double t){

	double rv = 60 + width * sin((2.0 * M_PI) / period * t);
	return rv;
}

void
moveLeftArmSin(controller_manager::ControllerManager& cm,
				CepheusHW& robot,
				ros::Publisher left_shoulder_pub,
				ros::Publisher left_elbow_pub)
{

	double movement_duration = 12;

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

		cmd_pos.data = produce_sin_trajectory(0.78, movement_duration, timer.toSec());
		left_shoulder_pub.publish(cmd_pos);

		cmd_pos.data = produce_sin_trajectory(-1.22, movement_duration, timer.toSec());
		left_elbow_pub.publish(cmd_pos);

		wrist_cmd = produce_sin_trajectory_wrist(-30, movement_duration, timer.toSec());
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


void
move_left_arm(double set_point_shoulder,
			double set_point_elbow,
			double set_point_wrist,
			double movement_duration,
			controller_manager::ControllerManager& cm,
			CepheusHW& robot,
			ros::Publisher left_shoulder_pub,
			ros::Publisher left_elbow_pub)
{

	ros::Time init_time = ros::Time::now();
	ros::Duration timer;
	timer = ros::Time::now() - init_time;

	robot.readEncoders(timer);

	double curr_pos_shoulder, curr_pos_elbow, curr_pos_wrist;
	double init_pos_shoulder = robot.getPos(LEFT_SHOULDER);
	double init_pos_elbow = robot.getPos(LEFT_ELBOW);
	double init_pos_wrist = robot.getCmd(LEFT_WRIST);

	ros::Rate loop_rate(400);

	std_msgs::Float64 cmd_pos;
	double wrist_cmd;

	while(timer.toSec() <= movement_duration){

		robot.readEncoders(timer);

		curr_pos_shoulder = robot.getPos(LEFT_SHOULDER);
		curr_pos_elbow = robot.getPos(LEFT_ELBOW);
		//curr_pos_wrist = robot.getCmd(LEFT_WRIST);

		cmd_pos.data = produce_trajectory_point(timer.toSec(), movement_duration, init_pos_shoulder, set_point_shoulder);
		//ROS_WARN("pos : %lf",cmd_pos.data);
		left_shoulder_pub.publish(cmd_pos);


		cmd_pos.data = produce_trajectory_point(timer.toSec(), movement_duration, init_pos_elbow, set_point_elbow);
		left_elbow_pub.publish(cmd_pos);
		/*
		wrist_cmd = produce_trajectory_point_wrist(timer.toSec(), movement_duration, init_pos_wrist, set_point_wrist);
		robot.setCmd(LEFT_WRIST, wrist_cmd);
		*/

		robot.readEncoders(timer);
		cm.update(ros::Time::now(), timer);
		robot.writeMotors();
		robot.heartbeat();

		ros::spinOnce();


		timer = ros::Time::now() - init_time;
		loop_rate.sleep();
	}
}


void
move_right_arm(double set_point_shoulder,
				double set_point_elbow,
				double set_point_wrist,
				double movement_duration,
				controller_manager::ControllerManager& cm,
				CepheusHW& robot,
				ros::Publisher right_shoulder_pub,
				ros::Publisher right_elbow_pub)
{

	ros::Time init_time = ros::Time::now();
	ros::Duration timer;
	timer = ros::Time::now() - init_time;

	robot.readEncoders(timer);

	double curr_pos_shoulder, curr_pos_elbow, curr_pos_wrist;
	double init_pos_shoulder = robot.getPos(RIGHT_SHOULDER);
	double init_pos_wrist = robot.getCmd(RIGHT_WRIST);
	double init_pos_elbow = robot.getPos(RIGHT_ELBOW);

	ros::Rate loop_rate(400);

	std_msgs::Float64 cmd_pos_sh;
	std_msgs::Float64 cmd_pos_elb;
	double wrist_cmd;

	//robot.setCmd(RIGHT_SHOULDER, 0.0);
	//robot.setCmd(RIGHT_ELBOW, 0.0);

	while(timer.toSec() <= movement_duration){

		robot.readEncoders(timer);

		curr_pos_shoulder = robot.getPos(RIGHT_SHOULDER);
		curr_pos_elbow = robot.getPos(RIGHT_ELBOW);
		curr_pos_wrist = robot.getCmd(RIGHT_WRIST);

		cmd_pos_sh.data = produce_trajectory_point(timer.toSec(), movement_duration, init_pos_shoulder, set_point_shoulder);
		cmd_pos_elb.data = produce_trajectory_point(timer.toSec(), movement_duration, init_pos_elbow, set_point_elbow);
		wrist_cmd = produce_trajectory_point_wrist(timer.toSec(), movement_duration, init_pos_wrist, set_point_wrist);

		std::cout<<cmd_pos_elb.data<<std::endl;

		right_elbow_pub.publish(cmd_pos_elb);
		right_shoulder_pub.publish(cmd_pos_sh);
		robot.setCmd(RIGHT_WRIST, wrist_cmd);

		cm.update(ros::Time::now(), timer);
		robot.writeMotors();
		robot.heartbeat();

		ros::spinOnce();

		timer = ros::Time::now() - init_time;
		loop_rate.sleep();
	}

	/*
	init_time = ros::Time::now();
	timer = ros::Time::now() - init_time;

	robot.readEncoders(timer);

	double init_pos_elbow = robot.getPos(RIGHT_ELBOW);
	while(timer.toSec() <= movement_duration){

		robot.readEncoders(timer);
		curr_pos_elbow = robot.getPos(RIGHT_ELBOW);

		cmd_pos.data = produce_trajectory_point(timer.toSec(), movement_duration, init_pos_elbow, set_point_elbow);
		right_elbow_pub.publish(cmd_pos);

		robot.readEncoders(timer);
		cm.update(ros::Time::now(), timer);
		robot.writeMotors();
		robot.heartbeat();

		ros::spinOnce();

		timer = ros::Time::now() - init_time;
		loop_rate.sleep();
	}
	*/
}

/*
void test_catch_object(double cmd_angle_to_catch, controller_manager::ControllerManager& cm, double xt, double yt){

	//theta2 needs to be from -90 deg to 0 deg


	//TO DO...ADD COMMMEEEEENTS

	//The lengths of the joint of the left arm
	double l1 = 0.181004;   //shoulder
	double l2 = 0.1605;     //elbow
	double l3 = 0.0499;     //wrist

	//Used in order to track the frame of the target
	tf::TransformListener target_listener;
	tf::StampedTransform transform;

	try{
		target_listener.lookupTransform("cepheus","gripper_target",
		ros::Time(0), transform);
	}
		catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
	}

	//position of target
	
	double x/ = transform.getOrigin().x();
	double y = transform.getOrigin().y();


	//Coordinates of target based on cepheus origin
	//double x = 0.21;
	//double y = -0.1;
	double x = xt;
	double y = yt;


	//the desired angle of the wrist translated to cepheus coordinates
	double phi = (90.0/180.0) * M_PI;


	//Need to transform the above coordinates to the base of the left arm
	x = x - 0.17268;
	y = y + 0.091404;

	double yn = y - l3  * cos(phi);
	double xn = x - l3 * sin(phi);

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
	if( (-M_PI/3 <= q31 && q31 <= M_PI/3.0) && (-2.0*M_PI/3.0 <= q21 && q21 <= 2.0*M_PI/3.0) && (M_PI/2.0 <= q11 && q11 <= M_PI) ){
		//solution is tuple1
		//translate wrist cmd (-60 to 60) to (0 to 120)
		q31 = abs((q31 * 180.0 / M_PI) - 60.0);
		q11 = q11 - M_PI/2.0;

		ROS_WARN("Solution, q1= %lf,  q2= %lf,  q3= %lf",q11,q21,q31);
		move_left_arm(q11, q21, q31, 12.0, cm);
	}
	else if((-M_PI/3.0 <= q32 && q32 <= M_PI/3.0) && (-2.0 * M_PI/3.0 <= q22 && q22 <= 2.0*M_PI/3.0) && (M_PI/2.0  <= q12 && q12 <= M_PI)){
		//solution is tuple2
		q32 = abs((q32 * 180.0 / M_PI) - 60.0);
		q12 = q12 - M_PI/2.0;

		ROS_WARN("Solution, q1= %lf,  q2= %lf,  q3= %lf",q12,q22,q32);
		move_left_arm(q12, q22, q32, 12.0, cm);
	}
	else{
		//we see.....
		ROS_WARN("Out of left arm workspace");
	}


}

*/
