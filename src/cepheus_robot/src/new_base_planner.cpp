/*

   About the chaser's movement:

   There are 3 velocity profiles depending on the target's position
   and the sign of the target's velocity
   We can have either of the 3 profiles in each axis idependent from each other

   Profile 1: (vel_profile = 1) means that the target is moving away from the chaser, or stands almost still and that the chaser is going to accelerate
   with full acceleration and the deaccelerate in order to reach the target

   Profile 2: (vel_profile = 2) means that the chaser is going to accelerate in the same direction of the target,
   with an acceleration calculated at that time in order to reach the
   target's velocity and then move with target's speed along with the target

   Profile 3: (vel_profile = 3) The chaser moves toward the target and in time it changes it's direction in order to allign with
   the target and move with it

 */

#include <signal.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <ros/ros.h>
#define RT_PRIORITY 95

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include "digital_filter.h"
#include "new_base_planner_utilities.h"

//For the grip action testing
//#include <cepheus_robot/RightCatchObjectAction.h>
//#include <cepheus_robot/LeftCatchObjectAction.h>
#include <actionlib/client/simple_action_client.h>
//typedef actionlib::SimpleActionClient<cepheus_robot::RightCatchObjectAction> ActionClientRight;
//typedef actionlib::SimpleActionClient<cepheus_robot::LeftCatchObjectAction> ActionClientLeft;



//These flags will be true when the vel prof is over and the controller must be disabled
//Both flags must be true in order to disable ctrl
bool disable_ctrl_X = false;
bool disable_ctrl_Y = false;

//for no creating new path when you follow target with staedy vel and you ready to grab it
bool update_path_X = true, update_path_Y = true;

//Create the constraints for the experiment
Geometric_Constraints constraints(1.897, 2.329, ROBOT_RADIUS, WS_RADIUS, 0.5);

//Creating 6 possbilly used profiles
//2 for each profile (x,y axis)
//Maybe there is a better solution...
Prf1 p1_X; Prf1 p1_Y;
Prf2 p2_X; Prf2 p2_Y;
Prf3 p3_X; Prf3 p3_Y;

//Some time constants needed in the decision process of the planning
const double TIME_TO_SMOOTH_ERROR = 0.5;//s
const unsigned int TIME_TO_OBSERVE_TARGET = 200;//ms

//In order to check if can grab target
const double REL_VEL_THRESHOLD = 0.01;

//In order to make sure that the heading is good before the conreoller is disabled
//Otherwise, the RW might try to correct the error, just a liitle before we disable the ctrl...
//...causing the base of the robot to turn unexpectidly. 
const double HEADING_ERROR_THRESHOLD = 0.05; 


//The duration of the inverse kinematic process
const double INV_KIN_DUR = 3.0;

//values {1,2,3}
//indcates which vel prof be used in each axis
short velocity_profile_X = 0, velocity_profile_Y = 0;

//The maximum acceleration of the chaser
double Fmax_X = 2.0*cos(M_PI/3.0) * FMAX_THRUST;
double A_MAX = Fmax_X / CHASER_MASS;


//The the acceleration in each axis
//sign of acceleration depends on the direction
double A_MAX_X, A_MAX_Y;

//distance from target used in second profile in order to reverse the
//orientation of the chaser's velocity
double L_X = 0.35;
//double L_X = WS_RADIUS;
double L_Y = 0.35;
//double L_Y = WS_RADIUS;

//The target's velcoty as observed form the chaser via the phase space system
double target_vel_X = 0.0, target_vel_Y = 0.0;
double rel_vel_x, rel_vel_y = 0.0;


//The chaser's init velocity in the begginning of every planning
double chaser_init_vel_X = 0.0, chaser_init_vel_Y = 0.0;

geometry_msgs::Vector3 target_real_pos;
geometry_msgs::Vector3 target_init_pos;
ros::Time target_pos_stamp;

geometry_msgs::Vector3 chaser_init_pos;
geometry_msgs::Vector3 chaser_real_pos;

//Distance between eges of the two objects
double xdes, ydes;

//The goal pos to move the chaser to
geometry_msgs::Vector3 des_pos;

bool start_planning = false;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
// Replacement SIGINT handler
void ctrl_C_Handler(int sig)
{
	g_request_shutdown = 1;
}

//The path created by the planner stored for any use
nav_msgs::Path path;


//---Helper functions------

void cut_digits(double &n, double dec_digit){

	//ROS_WARN("n b: %lf",n);
	double a = n * pow(10, dec_digit);
	n = (int)a / pow(10, dec_digit);
	//ROS_WARN("n a: %lf",n);
}

//------------------------



//-------Callbacks---------

bool chaser_first_time = false;
bool target_first_time = false;

void PhaseSpaceCallbackChaser(const geometry_msgs::TransformStamped::ConstPtr& msg)
{

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

	if(chaser_first_time){

		chaser_init_pos.x = temp.transform.translation.x;
		chaser_init_pos.y = temp.transform.translation.y;
		chaser_init_pos.z = yaw;

		//cut_digits(chaser_init_pos.x, 3);	
		//cut_digits(chaser_init_pos.y, 3);

		chaser_real_pos.x = temp.transform.translation.x;
		chaser_real_pos.y = temp.transform.translation.y;
		chaser_real_pos.z = yaw;

		//cut_digits(chaser_real_pos.x, 3);
		//cut_digits(chaser_real_pos.y, 3);


		chaser_first_time = false;
	}

	else{
		chaser_real_pos.x = temp.transform.translation.x;
		chaser_real_pos.y = temp.transform.translation.y;
		chaser_real_pos.z = yaw;

		//cut_digits(chaser_real_pos.x, 3);
		//cut_digits(chaser_real_pos.y, 3);

	}

	//ROS_INFO("phaseSpace called");
	return;
}

geometry_msgs::Quaternion target_rotation;

void PhaseSpaceCallbackTarget(const geometry_msgs::TransformStamped::ConstPtr& msg)
{

	geometry_msgs::TransformStamped temp;
	temp = *msg;

	double x = temp.transform.rotation.x;
	double y = temp.transform.rotation.y;
	double z = temp.transform.rotation.z;
	double w = temp.transform.rotation.w;
	double roll,pitch,yaw;

	target_rotation = temp.transform.rotation;

	tf::Quaternion q(x, y, z, w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll,pitch,yaw);

	if(target_first_time){
		target_init_pos.x = temp.transform.translation.x;
		target_init_pos.y = temp.transform.translation.y;
		target_init_pos.z = yaw;

		//cut_digits(target_init_pos.x, 3);
		//cut_digits(target_init_pos.y, 3);


		target_pos_stamp = temp.header.stamp;
		target_real_pos.x = temp.transform.translation.x;
		target_real_pos.y = temp.transform.translation.y;
		target_real_pos.z = yaw;

		//cut_digits(target_real_pos.x, 3);
		//cut_digits(target_real_pos.y, 3);

		target_first_time = false;
	}
	else{

		target_pos_stamp = temp.header.stamp;
		target_real_pos.x = temp.transform.translation.x;
		target_real_pos.y = temp.transform.translation.y;
		target_real_pos.z = yaw;

		//cut_digits(target_real_pos.x, 3);
		//cut_digits(target_real_pos.y, 3);

	}


	//std::cout<<target_pos_stamp.toSec()<<" "<<target_real_pos.x<<std::endl;;
	return;
}

void startChaseCallback(const std_msgs::Bool::ConstPtr& msg){
	start_planning = msg->data;	
}

double error_in_heading = 0.0;

void positionErrorCallback(const geometry_msgs::Vector3::ConstPtr& msg){
	error_in_heading = msg->z;
}

//---------------------------
void observe_target_velocity(const unsigned int ms, double& t_vel_X, double& t_vel_Y){


	static geometry_msgs::Vector3 target_prev_pos;


	target_prev_pos.x = target_real_pos.x;
	target_prev_pos.y = target_real_pos.y;

	unsigned int usecs_to_sleep = ms * 1000;

	//convert to secs to calculate velocity
	double dt = (double)ms/ (double)1000;


	//Sleep for microseconds
	usleep(usecs_to_sleep);

	ros::spinOnce();

	if(usecs_to_sleep > 0){
		//ROS_INFO("trp %lf tpp %lf" ,target_real_pos.x ,  target_prev_pos.x);

		t_vel_X = 0.0;
		t_vel_Y = -0.01;
		//t_vel_X = (target_real_pos.x - target_prev_pos.x)/dt;
		//t_vel_Y = (target_real_pos.y - target_prev_pos.y)/dt;

		//ROS_WARN("Uncut velx %lf, uncut vely %lf",t_vel_X, t_vel_Y);

		//cut_digits(t_vel_X, 3);
		//cut_digits(t_vel_Y, 3);

		std::cout<<"Observated target vel_X: "<<t_vel_X<<std::endl;
		std::cout<<"Observated target vel_Y: "<<t_vel_Y<<std::endl;

		double theta = atan2(target_real_pos.y - chaser_real_pos.y, target_real_pos.x - chaser_real_pos.x);

		//des_pos.x = target_real_pos.x - (WS_RADIUS + CIRCLE_RADIUS) * cos(theta);
		//des_pos.y = target_real_pos.y - (WS_RADIUS + CIRCLE_RADIUS) * sin(theta);

		des_pos.x = target_real_pos.x;
		des_pos.y = target_real_pos.y;		

		//std::cout<<"des_pos_X: "<<des_pos.x<<std::endl;
		//std::cout<<"des_pos_Y: "<<des_pos.y<<std::endl;
	}

}


void
setup_planning_parameters(double& A_MAX_AXIS, 
						double& L_AXIS,
						double ch_init_pos,
						double tar_init_pos,
						double v_des_axis,
						double v0_ch,
						double L)
{
	if(v_des_axis != 0.0){

		if(v_des_axis > 0.0){
			A_MAX_AXIS = A_MAX;
		}
		else{
			A_MAX_AXIS = -A_MAX;
		}

	}
	else if (ch_init_pos < tar_init_pos){
		A_MAX_AXIS = A_MAX;
	}
	else{
		A_MAX_AXIS = - A_MAX;
	}

	//Commmmeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeent on thiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiis
	L_AXIS = L;
	if (ch_init_pos > tar_init_pos){
		L_AXIS = - L_AXIS;
	}
}

bool
calc_vel_prof_1_params(const double& A_max,
					const double& V_DES,
					const double& INIT_CH,
					const double& V0_CH,
					const double& init_des,
					Prf1& res)
{
	ROS_WARN("CALC PROFILE 1");
	//ROS_WARN("A_max %lf V_DES %lf INIT_CH %lf V0_CH %lf init_des %lf",A_max, V_DES, INIT_CH, V0_CH, init_des);

	double t1,t2,xdes_target,xt1,vt1,xdes_chaser;

	double a = A_max;
	double b = 2.0 * (V0_CH - V_DES);
	double c = INIT_CH - init_des + 0.5 * pow(V_DES - V0_CH,2)/A_max;

	double delta = b*b - 4.0 * a * c;

	bool two_roots = false;

	//solutions 
	double s, s1, s2;

	if(delta < 0){
		//ROS_WARN("Cannot catch target. Delta < 0");
		return false;
	}
	else if(delta > 0){    

		two_roots = true;		

		s1 = (-b + sqrt(delta)) / (2.0 * a);
		s2 = (-b - sqrt(delta)) / (2.0 * a);


		if(s1 > 0){
			if(s1 < s2){
				t1 = s1;
			}
			else if(s2 > 0){
				t1 = s2;
			}
			else{
				t1 = s1;
			}
		}
		else if(s2 > 0){
			t1 = s2;
		}
		else{
			//ROS_WARN("Cannot catch target. Two neg solutions < 0");
			return false;
		}

		//ROS_WARN("a %lf b %lf c %lf  delta %lf s1 %lf s2 %lf t1 %lf",a,b,c, delta, s1,s2,t1);

	}
	else{

		s = -b / (2.0 * a);
		if(s > 0){
			t1 = s;
		}
		else{
			//ROS_WARN("Cannot catch target. Double neg solution");
			return false;
		}

		//ROS_WARN("delta %lf s1 %lf t1 %lf", delta, s ,t1);
	}



	t2 = 2.0 * t1 - (V_DES - V0_CH) / A_max;

	//Select the other root for t1, even if it is bigger than the first
	//cause the problem does not a have a solution for that value of t1
	if(t2 <= 0.0){ 

		if(two_roots){

			if(t1 == s1 && s2 > 0)
				t1 = s2;
			else if (t1 == s2 && s1 > 0)
				t1 = s1;

			//...and recalculate t2
			t2 = 2.0 * t1 - (V_DES - V0_CH) / A_max;

			if(t2 <= 0.0){
				//ROS_WARN("Cannot catch target. t2 <= 0 EVEN for 2nd solution of t1");
				return false;
			}

		}
		else{
			//ROS_WARN("Cannot catch target. t2 <= 0");
			return false;
		}
	}

	xdes_target = V_DES * t2 + init_des;
	xt1 = INIT_CH + V0_CH * t1 + 0.5*A_max*(t1*t1);
	vt1 = V0_CH + A_max * t1;
	xdes_chaser = xt1 + vt1 * (t2 - t1) - 0.5 * A_max * pow(t2 - t1,2);

	res.set_vals(t1, t2, xdes_target, V0_CH, xt1, vt1, xdes_chaser);
}

bool
calc_vel_prof_2_params(const double& INIT_CH,
					const double& V0_CH,
					const double& INIT_TAR,
					const double&  V_DES,
					Prf2& res)
{
	double t1,a_ch,xdes_chaser;

	ROS_WARN("CALC PROFILE 2");

	//Condition in order to prevent prof 2 when moving in other dir
	if( ( (V0_CH > 0 && V_DES < 0) || (V0_CH < 0 && V_DES > 0) ) && fabs(V0_CH) > fabs(V_DES) ){
		return false;
	}

	t1 = 2.0 * (INIT_CH - INIT_TAR)/(V_DES - V0_CH);

	if(t1 <= 0.0){
		//ROS_WARN("NEGATIVE t1 vel prof 2");
		return false;
	}

	a_ch = (V_DES - V0_CH)/t1;

	if(fabs(a_ch) > fabs(A_MAX)){
		//ROS_WARN("ACC too bog vel prof 2");
		if(A_MAX > 0 )
			a_ch = A_MAX;
		else
			a_ch = -A_MAX;		
	}

	xdes_chaser = INIT_CH + V0_CH*t1 + 0.5 *a_ch*pow(t1,2);

	res.set_vals(t1, xdes_chaser, V0_CH, a_ch);
	//res.print();
	return true;
}

bool
calc_vel_prof_3_params(const double& A_MAX,
					const double& V_DES,
					const double& INIT_CH,
					const double& V0_CH,
					const double& INIT_TAR,
					const double& L,
					Prf3& res)
{
	ROS_WARN("CALC PROFILE 3");

	double t1,t2,t3,a3,Xt1,Xt2,Vt1,xdes_chaser,xdes_target;

	bool two_roots = false;
	double s1,s2; //the two possible solutions

	double a_max;
	if(V_DES > 0){
		a_max = -A_MAX;
	}
	else{
		a_max = A_MAX;
	}

	double a = a_max;
	double b = 2.0*V0_CH - 2.0*V_DES;
	double c = INIT_CH - INIT_TAR + (0.5 * pow(V0_CH,2)/a_max) - (V_DES * V0_CH/a_max) + L;

	double delta = b*b - 4.0 * a * c;

if (delta < 0){
		//ROS_WARN("Delta < 0");
		return false;
	} 
	else if (delta > 0){   

		two_roots = true;

		s1 = (-b + sqrt(delta)) / (2.0 * a);
		s2 = (-b - sqrt(delta)) / (2.0 * a);

		if(s1 > 0){
			if(s1 < s2){
				t1 = s1;
			}
			else if(s2 > 0){
				t1 = s2;
			}
			else{
				t1 = s1;
			}
		}
		else if(s2 > 0){
			t1 = s2;
		}
		else{
			//ROS_WARN("Cannot catch target. Two neg solutions < 0");
			return false;
		}    

	}
	else{

		double s1 = -b / (2.0 * a);

		if(s1 > 0){
			t1 = s1;
		}
		else{
			//ROS_WARN("Cannot catch target. Double neg solution");
			return false;
		}
	}

	t2 = V0_CH/a_max + 2.0 * t1;

	t3 = ((-2.0 *L)/V_DES) + t2;

	//Select the other root for t1, even if it is bigger than the first
	//cause the problem does not a have a solution for that value of t1
	if((t2 <= 0.0 || t3 <= 0.0)){

		if(two_roots){

			if(t1 == s1 && s2 > 0)
				t1 = s2;
			else if (t1 == s2 && s1 > 0)
				t1 = s1;

			//...and recalculate t2 and t3
			t2 = V0_CH/a_max + 2.0 * t1;
			t3 = ((-2.0 *L)/V_DES) + t2;

			if((t2 <= 0.0 || t3 <= 0.0)){
				//ROS_WARN("Cannot catch target. t2 <= 0 or t3 <=0 EVEN for 2nd solution of t1");
				return false;
			}
		}
		else{
			//ROS_WARN("Cannot catch target. t2 <= 0 ");
			return false;
		}
	}

	a3 = -0.5 * pow(V_DES,2)/L;

	if(V_DES > 0){
		a_max = -A_MAX;
	}
	else{
		a_max = A_MAX;
	}


	Vt1 = V0_CH + a_max * t1;
	Xt1 = INIT_CH + V0_CH * t1 + 0.5 * a_max * pow(t1,2);
	Xt2 = Xt1 + Vt1 * (t2 - t1) - 0.5 * a_max * pow(t2 - t1,2);


	if(t3 <= 0.0){
		//ROS_WARN("NEGATIVE t3 vel prof 3");
		return false;
	}

	if(fabs(a3) > fabs(A_MAX)){
		//ROS_WARN("ACC too bog vel prof 2");

		if(a3 > 0 )
			a3 = A_MAX;
		else
			a3 = -A_MAX;
	}

	xdes_chaser = Xt2 + 0.5 * a3 * pow(t3 - t2,2);

	xdes_target = V_DES * t3 + INIT_TAR;

	res.set_vals(t1, t2, t3, a3, Vt1, Xt1, Xt2, xdes_chaser, V0_CH, xdes_target);
	//res.print();

	return true;
}


void decide_plan_of_action_X()
{
	//-----------FOR X AXIS------------

	ROS_WARN("Prof X: ");

	//Target almost still
	if(target_vel_X == 0.0){

		bool rv_1, in_c_vp1 = false;

		rv_1 = calc_vel_prof_1_params(A_MAX_X, target_vel_X, chaser_init_pos.x, chaser_init_vel_X, des_pos.x, p1_X);

		//check constraints
		//bool in_c_vp1 = constraints.in_constraints_for_X(p1_X.xdes_chaser);

		if(!rv_1){
			//ROS_WARN("MEETING POINT OUT OF CONSTRAINTS!");
			velocity_profile_X = (short)VEL_PROF_0;
		}
		else{
			velocity_profile_X = (short)VEL_PROF_1;
			p1_X.print();
		}
	}
	//Target moving away
	else if((chaser_init_pos.x <= des_pos.x && target_vel_X > 0) || (chaser_init_pos.x >= des_pos.x && target_vel_X  < 0)){


		bool rv_1, in_c_vp1 = false;

		rv_1 = calc_vel_prof_1_params(A_MAX_X, target_vel_X, chaser_init_pos.x, chaser_init_vel_X, des_pos.x, p1_X);
		//check constraints
		//bool in_c_vp1 = constraints.in_constraints_for_X(p1_X.xdes_chaser);

		if(!rv_1){
			//ROS_WARN("");
			velocity_profile_X = (short)VEL_PROF_0;;
		}
		else{
			velocity_profile_X = (short)VEL_PROF_1;
			p1_X.print();
		}
	}

	//Target is aproacing the chaser
	//In order to decide the velocity profile for the movement of the chaser, both Vel Prof parameters will be calculated.
	//Then the meeting point with the target will be compared with the constraints developed in 'base_planner_utilities.h'
	//If both profiles satisfy the constraints ,both durations, wiht active ctrl, for Prof 2 and Prof 3 will be compared...
	//...and the Prof with the shortest duration we be chosen
	else{
		bool rv_2, rv_3, in_c_vp2, in_c_vp3 = false;
		//....Vel Prof 2 params
		rv_2 = calc_vel_prof_2_params(chaser_init_pos.x, chaser_init_vel_X, des_pos.x, target_vel_X, p2_X);

		//....Vel Prof 3 params
		rv_3 = calc_vel_prof_3_params(A_MAX_X, target_vel_X, chaser_init_pos.x, chaser_init_vel_X, des_pos.x, L_X, p3_X);

		if(!rv_2 && !rv_3){
			//ROS_WARN("NO SOLUTION FOR VEL PROF 1 & 2.");
			velocity_profile_X = (short)VEL_PROF_0;
		}
		//Vel Prof 2
		else if(rv_2 && !rv_3){
			velocity_profile_X = (short)VEL_PROF_2;
			p2_X.print();
		}
		//Vel Prof 3
		else if(!rv_2 && rv_3){
			velocity_profile_X = (short)VEL_PROF_3;
			p3_X.print();
		}
		//compare durations to choose profile
		else{
			//Vel Prof 3    
			if(p3_X.duration_with_active_ctrl <= p2_X.duration_with_active_ctrl){
				velocity_profile_X = (short)VEL_PROF_3;
				p3_X.print();
			}
			//Vel Prof 2
			else{
				velocity_profile_X = (short)VEL_PROF_2;
				p2_X.print();
			}
		}
	}
}

void decide_plan_of_action_Y(){

	//------------FOR Y AXIS--------------

	ROS_WARN("Prof Y: ");

	//Target almost still
	if(target_vel_Y == 0.0){

		bool rv_1, in_c_vp1 = false;

		rv_1 = calc_vel_prof_1_params(A_MAX_Y, target_vel_Y, chaser_init_pos.y, chaser_init_vel_Y, des_pos.y, p1_Y);

		//check constraints
		//bool in_c_vp1 = constraints.in_constraints_for_Y(p1_Y.xdes_chaser);

		if(!rv_1){
			//ROS_WARN("MEETING POINT OUT OF CONSTRAINTS! ABORTING.............");
			velocity_profile_Y = (short)VEL_PROF_0;
		}
		else{
			velocity_profile_Y = (short)VEL_PROF_1;
			p1_Y.print();
		}
	}

	//Target is moving away
	else if((chaser_init_pos.y <= des_pos.y && target_vel_Y > 0) || (chaser_init_pos.y >= des_pos.y && target_vel_Y  < 0)){

		bool rv_1, in_c_vp1 = false;

		rv_1 = calc_vel_prof_1_params(A_MAX_Y, target_vel_Y, chaser_init_pos.y, chaser_init_vel_Y, des_pos.y, p1_Y);

		//check constraints
		//bool in_c_vp1 = constraints.in_constraints_for_Y(p1_Y.xdes_chaser);

		if(!rv_1){
			//ROS_WARN("MEETING POINT OUT OF CONSTRAINTS! ABORTING.............");
			velocity_profile_Y = (short)VEL_PROF_0;
		}
		else{
			velocity_profile_Y = (short)VEL_PROF_1;
			p1_Y.print();
		}
	}

	//Target is aproacing the chaser
	//In order to decide the velocity profile for the movement of the chaser, both Vel Prof parameters will be calculated.
	//Then the meeting point with the target will be compared with the constraints developed in 'base_planner_utilities.h'
	//If both profiles satisfy the constraints ,both durations, wiht active ctrl, for Prof 2 and Prof 3 will be compared...
	//...and the Prof with the shortest duration we be chosen
	else{
		bool rv_2, rv_3, in_c_vp2, in_c_vp3 = false;

		//....Vel Prof 2 params
		rv_2 = calc_vel_prof_2_params(chaser_init_pos.y, chaser_init_vel_Y, des_pos.y, target_vel_Y, p2_Y);
		//....Vel Prof 3 params
		rv_3 = calc_vel_prof_3_params(A_MAX_Y, target_vel_Y, chaser_init_pos.y, chaser_init_vel_Y, des_pos.y, L_Y, p3_Y);


		if(!rv_2 && !rv_3){
			//ROS_WARN("MEETING POINT OUT OF CONSTRAINTS! ABORTING.............");
			velocity_profile_Y = (short)VEL_PROF_0;
		}
		//Vel Prof 2
		else if(rv_2 && !rv_3){
			velocity_profile_Y = (short)VEL_PROF_2;
			p2_Y.print();
		}
		//Vel Prof 3
		else if(!rv_2 && rv_3){
			velocity_profile_Y = (short)VEL_PROF_3;
			p3_Y.print();
		}
		//compare durations to choose profile
		else{
			//Vel Prof 3    
			if(p3_Y.duration_with_active_ctrl <= p2_Y.duration_with_active_ctrl){
				velocity_profile_Y = (short)VEL_PROF_3;
				p3_Y.print();
			}
			//Vel Prof 2
			else{
				velocity_profile_Y = (short)VEL_PROF_2;
				p2_Y.print();
			}
		}
	}
}

void
produce_chaser_trj_points_and_vel_prof_1 (const double& t,
										const double& INIT_CH,
										const double& V0_CH,
										const double& A_max,
										const double& V_DES,
										const Prf1& prof_params,
										double& cmd_pos,
										double& cmd_vel,
										double& cmd_acc)
{


	if (t <= prof_params.t1){
		//ROS_INFO("Accelerate =>");
		//ROS_INFO("INIT_CH %lf", INIT_CH);
		cmd_pos = INIT_CH + V0_CH * t + 0.5 * A_max * pow(t,2);
		//ROS_INFO("cmd_pos %lf", cmd_pos);
		cmd_vel = V0_CH + A_max * t;
		cmd_acc = A_max;
	}
	else if (t > prof_params.t1 && t <= prof_params.t2){
		//ROS_INFO("Deaccelerate =>");
		cmd_pos = prof_params.xt1 + prof_params.vt1*(t - prof_params.t1) - 0.5 * A_max * pow(t - prof_params.t1,2);
		cmd_vel = prof_params.vt1 - A_max * (t - prof_params.t1); 
		cmd_acc = -A_max; 
	}
	else if (t > prof_params.t2 && t <= prof_params.duration_with_active_ctrl){
		//ROS_INFO("Stand Still...");
		cmd_pos = prof_params.xdes_chaser + V_DES * (t - prof_params.t2);
		cmd_vel = V_DES;
		cmd_acc = 0.0;
	}
}

void
produce_chaser_trj_points_and_vel_prof_2 (const double& t,
										const double& INIT_CH,
										const double& V0_CH,
										const double& V_DES,
										const Prf2& prof_params,
										double& cmd_pos,
										double& cmd_vel,
										double& cmd_acc)
{

	if (t <= prof_params.t1){
		cmd_pos = INIT_CH + V0_CH*t + 0.5 * prof_params.a_ch * pow(t,2);
		cmd_vel = V0_CH * prof_params.a_ch * t;
		cmd_acc = prof_params.a_ch;
	}
	else if(t > prof_params.t1 && t <= prof_params.duration_with_active_ctrl){
		cmd_pos = prof_params.xdes_chaser + V_DES*(t - prof_params.t1);
		cmd_vel = V_DES;
		cmd_acc = 0.0;
	}
}

void
produce_chaser_trj_points_and_vel_prof_3 (const double& t,
										const double& INIT_CH,
										const double& V0_CH,
										const double& A_MAX,
										const double& V_DES,
										const Prf3& prof_params,
										double& cmd_pos,
										double& cmd_vel,
										double& cmd_acc)
{

	double a_max_axis;

	if (t <= prof_params.t1){

		if(V_DES > 0){
			a_max_axis = -A_MAX;
		}
		else{
			a_max_axis = A_MAX;
		}

		cmd_pos = INIT_CH + V0_CH*t + 0.5 * a_max_axis * pow(t,2);
		cmd_vel = V0_CH + a_max_axis*t;
		cmd_acc = a_max_axis;
	}
	else if (t <= prof_params.t2){

		if(V_DES > 0){
			a_max_axis = -A_MAX;
		}
		else{
			a_max_axis = A_MAX;
		}

		cmd_pos = prof_params.Xt1 + prof_params.Vt1 * (t - prof_params.t1) - 0.5 * a_max_axis * pow(t - prof_params.t1,2);
		cmd_vel = prof_params.Vt1 - a_max_axis * (t - prof_params.t1);
		cmd_acc = a_max_axis; 

	}    
	else if (t <= prof_params.t3){
		cmd_pos = prof_params.Xt2 + 0.5 * prof_params.a3 * pow(t - prof_params.t2, 2);
		cmd_vel = prof_params.a3 * (t - prof_params.t2); // 0 initial speed
		cmd_acc = prof_params.a3;
	}
	else if(t > prof_params.t3 && t <= prof_params.duration_with_active_ctrl){
		cmd_pos = prof_params.xdes_chaser + V_DES * (t - prof_params.t3);
		cmd_vel = V_DES;
		cmd_acc = 0.0;
	}
}

void wait_to_smooth_error(const double dur){

	ros::Duration timer_norm;
	ros::Time init_time_norm = ros::Time::now();
	//in order to normalize pos
	do{
		ros::spinOnce();
		timer_norm = ros::Time::now() - init_time_norm;

	} while(timer_norm.toSec() < dur);

	target_first_time = chaser_first_time = true;
}


void set_commands(const double& t_X,
		const double& t_Y,
		double& new_x,
		double& new_y,
		double& new_vel_x,
		double& new_vel_y,
		double& new_acc_x,
		double& new_acc_y)
{

	static bool one_time_X = true, one_time_Y = true;
	static double last_chaser_pos_X, last_chaser_pos_Y;

	if(velocity_profile_X == (short)VEL_PROF_1){
		
		if(t_X <= p1_X.t2 + p1_X.duration_with_active_ctrl){
			produce_chaser_trj_points_and_vel_prof_1(t_X, chaser_init_pos.x, chaser_init_vel_X, A_MAX_X, target_vel_X, p1_X , new_x, new_vel_x, new_acc_x);
			one_time_X = true;
		}

		if(t_X > p1_X.t2 && t_X <= p1_X.duration_with_active_ctrl){
			update_path_X = false;
		}

		if(t_X > p1_X.duration_with_active_ctrl){

			disable_ctrl_X = true;

			ROS_WARN("p1_x dc");

			if(!disable_ctrl_Y){

				if(one_time_X){
					last_chaser_pos_X = chaser_real_pos.x;
					one_time_X = false;
				}

				new_x = last_chaser_pos_X + target_vel_X * (t_X - p1_X.duration_with_active_ctrl);
				new_vel_x = target_vel_X;
				new_acc_x = 0.0;				
			}
		}
	}
	else if(velocity_profile_X == (short)VEL_PROF_2){

		if(t_X <= p2_X.t1 + p2_X.duration_with_active_ctrl){
			produce_chaser_trj_points_and_vel_prof_2(t_X, chaser_init_pos.x, chaser_init_vel_X, target_vel_X, p2_X , new_x, new_vel_x, new_acc_x);

			one_time_X = true;
		}
		if(t_X > p2_X.t1 && t_X <= p2_X.duration_with_active_ctrl){
			update_path_X = false;
		}

		if(t_X > p2_X.duration_with_active_ctrl){

			disable_ctrl_X = true;

			ROS_WARN("p2_x dc");

			if(!disable_ctrl_Y){

				if(one_time_X){
					last_chaser_pos_X = chaser_real_pos.x;
					one_time_X = false;
				}

				new_x = last_chaser_pos_X + target_vel_X * (t_X - p2_X.duration_with_active_ctrl);
				new_vel_x = target_vel_X;
				new_acc_x = 0.0;
			}
		}

	}
	else if(velocity_profile_X == (short)VEL_PROF_3){

		if(t_X <= p3_X.t3 + p3_X.duration_with_active_ctrl){
			produce_chaser_trj_points_and_vel_prof_3(t_X, chaser_init_pos.x, chaser_init_vel_X, A_MAX_X, target_vel_X, p3_X , new_x, new_vel_x, new_acc_x);
			one_time_X = true;

		}
		if(t_X > p3_X.t3 && t_X <= p3_X.duration_with_active_ctrl){
			update_path_X = false;
		}

		if(t_X > p3_X.duration_with_active_ctrl){

			disable_ctrl_X = true;

			ROS_WARN("p3_x dc");

			if(!disable_ctrl_Y){

				if(one_time_X){
					last_chaser_pos_X = chaser_real_pos.x;
					one_time_X = false;
				}

				new_x = last_chaser_pos_X + target_vel_X * (t_X - p3_X.duration_with_active_ctrl);
				new_vel_x = target_vel_X;
				new_acc_x = 0.0;
			}
		}


	}
	//Vel prof 0...Move with target speed (time step 0.005 secs, as loop rate)
	else{
		if(fabs(xdes - new_x) <= WS_RADIUS){
			new_x = new_x + target_vel_X * 0.005;
			new_vel_x = target_vel_X;
			new_acc_x = 0.0;
		}
		else{
			new_x = new_x - target_vel_X * 0.005;
			new_vel_x = -target_vel_X;
			new_acc_x = 0.0;
		}
	}

	if(velocity_profile_Y == (short)VEL_PROF_1){

		if(t_Y <= p1_Y.t2 + p1_Y.duration_with_active_ctrl){

			produce_chaser_trj_points_and_vel_prof_1(t_Y, chaser_init_pos.y, chaser_init_vel_Y, A_MAX_Y, target_vel_Y, p1_Y, new_y, new_vel_y, new_acc_y);
			one_time_Y = true;
		}

		if(t_Y > p1_Y.t2 && t_Y <= p1_Y.duration_with_active_ctrl){
			update_path_Y = false;
		}

		if(t_Y > p1_Y.duration_with_active_ctrl){

			disable_ctrl_Y = true;

			ROS_WARN("p1_y dc");

			if(!disable_ctrl_X){

				if(one_time_Y){
					last_chaser_pos_Y = chaser_real_pos.y;
					one_time_Y = false;
				}

				new_y = last_chaser_pos_Y + target_vel_Y *  (t_Y - p1_Y.duration_with_active_ctrl);
				new_vel_y = target_vel_Y;
				new_acc_y = 0.0;
			}

		}
	}
	else if(velocity_profile_Y == (short)VEL_PROF_2){

		if(t_Y <= p2_Y.t1 + p2_Y.duration_with_active_ctrl){

			produce_chaser_trj_points_and_vel_prof_2(t_Y, chaser_init_pos.y, chaser_init_vel_Y, target_vel_Y, p2_Y , new_y, new_vel_y, new_acc_y);
			one_time_Y = true;
		}

		if(t_Y > p2_Y.t1 && t_Y <= p2_Y.duration_with_active_ctrl){
			update_path_Y = false;
		}

		if(t_Y > p2_Y.duration_with_active_ctrl){

			disable_ctrl_Y = true;

			ROS_WARN("p2_y dc");

			if(!disable_ctrl_X){

				if(one_time_Y){
					last_chaser_pos_Y = chaser_real_pos.y;
					one_time_Y = false;
				}

				new_y = last_chaser_pos_Y + target_vel_Y *  (t_Y - p2_Y.duration_with_active_ctrl);
				new_vel_y = target_vel_Y;
				new_acc_y = 0.0;
			}

		}


	}
	else if(velocity_profile_Y == (short)VEL_PROF_3){

		if(t_Y <= p3_Y.t3 + p3_Y.duration_with_active_ctrl){

			produce_chaser_trj_points_and_vel_prof_3(t_Y, chaser_init_pos.y, chaser_init_vel_Y, A_MAX_Y, target_vel_Y, p3_Y , new_y, new_vel_y, new_acc_y);
			one_time_Y = true;
		}
		if(t_Y > p3_Y.t3 && t_Y <= p3_Y.duration_with_active_ctrl){
			update_path_Y = false;
		}

		if(t_Y > p3_Y.duration_with_active_ctrl){

			disable_ctrl_Y = true;

			ROS_WARN("p3_y dc");

			if(!disable_ctrl_X){

				if(one_time_Y){
					last_chaser_pos_Y = chaser_real_pos.y;
					one_time_Y = false;
				}

				new_y = last_chaser_pos_Y + target_vel_Y *  (t_Y - p3_Y.duration_with_active_ctrl);
				new_vel_y = target_vel_Y;
				new_acc_y = 0.0;
			}
		}
	}
	//Vel prof 0...Move with target speed (time step 0.005 secs, as loop rate)
	else{
		if(fabs(ydes - new_y) <= WS_RADIUS){
			new_y = new_y + target_vel_Y * 0.005;
			new_vel_y = target_vel_Y;
			new_acc_y = 0.0;
		}
		else{
			new_y = new_y - target_vel_Y * 0.005;
			new_vel_y = -target_vel_Y;
			new_acc_y = 0.0;			
		}
	}

}

/*
bool in_chaser_workspace(){

	ros::spinOnce();

	double dist = sqrt( pow(target_real_pos.x - chaser_real_pos.x, 2) + pow(target_real_pos.y - chaser_real_pos.y, 2) ) - CIRCLE_RADIUS;

	//ROS_WARN("dist %lf ws %lf", dist,WS_RADIUS);
	//WS radius 10 cm bigger than the normal in order to be easier to cath the target if it moves a bit away
	return (dist <= (WS_RADIUS + 2)  );
}
*/

void check_if_can_grab(double new_vel_x, 
			double new_vel_y){

	double theta = atan2(target_real_pos.y - chaser_real_pos.y, target_real_pos.x - chaser_real_pos.x);	

	double xdes = target_real_pos.x - CIRCLE_RADIUS * cos(theta);
	double ydes = target_real_pos.y - CIRCLE_RADIUS * sin(theta);

	double xchaser = chaser_real_pos.x + ROBOT_RADIUS * cos(theta);
	double ychaser = chaser_real_pos.y + ROBOT_RADIUS * sin(theta);	

	//double abs_dist = sqrt( pow(xdes - xchaser ,2) + pow(ydes - ychaser ,2) );
	double abs_dist = sqrt( pow(target_real_pos.x - chaser_real_pos.x ,2) + pow(target_real_pos.y - chaser_real_pos.y ,2) );
	ROS_WARN("DIST : %lf", abs_dist);

	/*
	if(abs_dist <= WS_RADIUS){
		//same sign
		if( target_vel_X * new_vel_x >= 0.0 && target_vel_Y * new_vel_y >= 0.0 ){
			std::cout<<"same sign"<<std::endl;
			rel_vel_x = target_vel_X - new_vel_x;
			rel_vel_y = target_vel_Y - new_vel_y;
			//rel vel constraints
			if(fabs(rel_vel_x) <= REL_VEL_THRESHOLD && fabs(rel_vel_y) <= REL_VEL_THRESHOLD){
				std::cout<<"rel vel below thresh"<<std::endl;
				//small rotational error constraint
				if(error_in_heading <= HEADING_ERROR_THRESHOLD){
					std::cout<<"heading error below thresh"<<std::endl;
					disable_ctrl_X = true;
					disable_ctrl_Y = true;
				}
			}
		}
	}
	*/

	if((abs_dist <= 0.65) && (error_in_heading <= HEADING_ERROR_THRESHOLD)){
		disable_ctrl_X = true;
		disable_ctrl_Y = true;
	}
}


bool check_if_desired_pos_too_close(double current_x, 
					double current_y,
					double& xdes,
					double& ydes){

	double theta = atan2(target_real_pos.y - chaser_real_pos.y, target_real_pos.x - chaser_real_pos.x);

	xdes = target_real_pos.x - CIRCLE_RADIUS * cos(theta);
	ydes = target_real_pos.y - CIRCLE_RADIUS * sin(theta);

	double dist_des = sqrt( pow(des_pos.x - xdes ,2) + pow(des_pos.y - ydes ,2) );
	double dist_abs = sqrt( pow(xdes - current_x ,2) + pow(ydes - current_y,2) ) - ROBOT_RADIUS/2.0;

	if(dist_des >= dist_abs){
		return true;
	}		

	return false;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "new_base_planner_node", ros::init_options::NoSigintHandler);
	signal(SIGINT, ctrl_C_Handler);


	//Node Handler for global callback queue
	ros::NodeHandle nh;

	struct sched_param schedParam;
	schedParam.sched_priority = RT_PRIORITY;
	int sched_policy = SCHED_RR;
	sched_setscheduler(0, sched_policy, &schedParam);

	//Create the constraints for the experiment
	Geometric_Constraints constraints(1.897, 2.329, 0.15, 0.3, 0.5);

	ros::ServiceClient controller_srv_client = nh.serviceClient<std_srvs::SetBool>("controller_cmd");

	ros::Subscriber phase_space_sub_target =  nh.subscribe("map_to_assist_robot", 1, PhaseSpaceCallbackTarget);
	ros::Subscriber phase_space_sub_chaser =  nh.subscribe("map_to_cepheus", 1, PhaseSpaceCallbackChaser);
	ros::Subscriber start_planning_sub =  nh.subscribe("start_chase", 1, startChaseCallback);

	//For keeping track of the error, mainly in rotation
	ros::Subscriber error_sub =  nh.subscribe("error", 1, positionErrorCallback);

	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("new_path", 1000);

	//Send cmds to base controller node
	ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("planner_pos", 1);
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("planner_vel", 1);
	ros::Publisher acc_pub = nh.advertise<geometry_msgs::Vector3>("planner_acc", 1);

	//Send command to cepheus interface to start invert kinematics in order to catch the target
	//ros::Publisher left_arm_grip_pub = nh.advertise<geometry_msgs::PointStamped>("left_arm_catch_object", 1);
	//ros::Publisher right_arm_grip_pub = nh.advertise<geometry_msgs::PointStamped>("right_arm_catch_object", 1);

	//Action testing for right grip!!!!!!!!!!
	/*
	ActionClientRight cl_right("right_catch_object_action", true); // true -> don't need ros::spin()
	cl_right.waitForServer();
	cepheus_robot::RightCatchObjectGoal right_goal;
	right_goal.point_to_catch.header.frame_id = "/assist_robot";
	*/
	/*
	ActionClientLeft cl_left("left_catch_object_action", true); // true -> don't need ros::spin()
	cl_left.waitForServer();
	cepheus_robot::LeftCatchObjectGoal left_goal;
	*/


	//Trajectory point produced ,command velocity and command acceleration
	double new_x, new_y, new_vel_x, new_vel_y, new_acc_x, new_acc_y = 0.0;
	geometry_msgs::PoseStamped new_pos;
	geometry_msgs::PoseStamped prev_pos;	
	geometry_msgs::TwistStamped new_vel;
	geometry_msgs::Vector3 new_acc;

	new_pos.header.frame_id = "/cepheus";
	new_vel.header.frame_id = "/cepheus";

	//The listener for des_pos
	tf::TransformListener des_pos_listener;
	tf::StampedTransform des_pos_transform;

	bool new_path = true;
	path.header.frame_id = "/map";


	bool ws_warn = true;

	//For enabling/disabling base controller
	std_srvs::SetBool srv;

	double heading;
	double prev_heading = 0.0;


	tf::Quaternion qq;
	ros::Rate loop_rate(200);

	//Global timer
	ros::Time prev_time_glb;
	ros::Time init_time_glb;
	ros::Duration timer_glb;
	ros::Duration dt;

	//Timer for the planning in X axis
	ros::Time init_time_X;
	ros::Duration timer_X;

	//Timer for the planning in Y axis
	ros::Time init_time_Y;
	ros::Duration timer_Y;


	bool head_o_t = true;

	int counter = 0;

	bool inv_kin_successs = false;

	while(!g_request_shutdown){

		counter++;

		if(!start_planning){

			//in order to reinitialize the robots' init positions
			chaser_first_time = true;
			target_first_time = true;

			//In order not to terminate the program when you wan to rerun-------
			ROS_WARN("Planner is waiting for cmd to start in topic \"/start_chase\"...................");
			while(!start_planning){
				ros::spinOnce();
				sleep(1);
			}

			ROS_WARN("Planner is starting the observation and decision process...................");

			wait_to_smooth_error(TIME_TO_SMOOTH_ERROR);
			observe_target_velocity(TIME_TO_OBSERVE_TARGET, target_vel_X, target_vel_Y);
			
			setup_planning_parameters(A_MAX_X, 
						L_X, 
						chaser_init_pos.x,
						target_init_pos.x,
						target_vel_X,
						chaser_init_vel_X,
						L_X);
			
			setup_planning_parameters(A_MAX_Y,
										L_Y,
										chaser_init_pos.y,
										target_init_pos.y,
										target_vel_Y,
										chaser_init_vel_Y,
										L_Y);


			decide_plan_of_action_X();
			decide_plan_of_action_Y();

			ROS_WARN("Planner is starting to produce the trajecory");

			prev_time_glb = ros::Time::now();
			init_time_glb = ros::Time::now();

			init_time_X = ros::Time::now();

			init_time_Y = ros::Time::now();


			new_path = true;
		}


		if (new_path) {

			new_path=false;

			//request to enable controller
			srv.request.data = true;
			if (controller_srv_client.call(srv)) {
				ROS_INFO_STREAM("Controller response: " << srv.response.message);
			}
			else{
				ROS_ERROR("Failed to call Controller");
			}

			prev_pos.pose.position.x = chaser_init_pos.x;
			prev_pos.pose.position.y = chaser_init_pos.y;
		}


		//check if the distnce and the relative velocities are appropriate in order to grab the target
		check_if_can_grab(new_vel_x, new_vel_y);

		//both velocity profiles traj is over 
		if(disable_ctrl_X && disable_ctrl_Y){
			//ROS_WARN("REQUEST_TO_DISABLE_CTRL");
			//request to disable controller
			srv.request.data = false;
			if (controller_srv_client.call(srv)) {
				//ROS_INFO_STREAM("Controller response: " << srv.response.message);
			}
			else{
				ROS_ERROR("Failed to call Controller");
			}
			/*
			//check if target is in chaser's workspace
			if(!inv_kin_successs){

				ROS_WARN("The chaser will reach to grab the target!");

				//calculate one last time the best point to grip the target	
				ros::spinOnce();

				double theta = atan2(target_real_pos.y - chaser_real_pos.y, target_real_pos.x - chaser_real_pos.x);

				//Include the relative movement of the 2 objects
				double x = (target_real_pos.x - (WS_RADIUS + CIRCLE_RADIUS) * cos(theta)) + rel_vel_x * INV_KIN_DUR;
				double y = (target_real_pos.y - (WS_RADIUS + CIRCLE_RADIUS) * sin(theta)) + rel_vel_y * INV_KIN_DUR;

				//double x = (target_real_pos.x - (WS_RADIUS + CIRCLE_RADIUS) * cos(theta));
				//double y = (target_real_pos.y - (WS_RADIUS + CIRCLE_RADIUS) * sin(theta));

				//command to catch object in interface
				right_goal.point_to_catch.pose.position.x = x;
				right_goal.point_to_catch.pose.position.y = y;
				right_goal.point_to_catch.pose.orientation = target_rotation;

				cl_right.sendGoal(right_goal);
				cl_right.waitForResult(ros::Duration(8.0));

				if (cl_right.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
					printf("Cepheus completed inverse kinematics");
					inv_kin_successs = true;
				}
				else{
					ROS_WARN("Cepheus failed to complete inverse kinematics");
				}

				printf("Current State: %s\n", cl_right.getState().toString().c_str());

			}
			else{
				if(ws_warn){
					ROS_WARN("The target is out of chaser's workspace!");
					ws_warn = false;
				}
			}
			*/			

		}


		dt = ros::Time::now() - prev_time_glb;
		prev_time_glb = ros::Time::now();

		timer_X = ros::Time::now() - init_time_X;
		timer_Y = ros::Time::now() - init_time_Y;
		timer_glb = ros::Time::now() - init_time_glb;

		ros::spinOnce();



		//Calculate again the target's velocity every a ceratain amount of time in order to adjust the path if the velocity changes
		//200 loops is 1 seconds
		if(counter == 200 ){

			counter = 0;

			double obs_vel_X, obs_vel_Y;
			bool too_close;

			ROS_INFO("Recalculating Target's Velocity");
			ros::spinOnce();
			observe_target_velocity(TIME_TO_OBSERVE_TARGET, obs_vel_X, obs_vel_Y);

			/*
			too_close = check_if_desired_pos_too_close(new_x, new_y, xdes, ydes);

			if(too_close){

				velocity_profile_X = 0;
				velocity_profile_Y = 0;

				target_vel_X = obs_vel_X;
				target_vel_Y = obs_vel_Y;	
			}
			else{*/

				if(update_path_X){

					double diff_X = fabs(obs_vel_X - target_vel_X);

					ROS_INFO("t_vel_X %lf , obs_vel_X %lf, diff_X %lf" , target_vel_X, obs_vel_X, diff_X);

					if( diff_X > 0.007 ){

						target_vel_X = obs_vel_X;

						ROS_INFO("Found change in target's velocity! \n Recalculating path.....");

						//store the last velocity of the chaser ,in order to pass the init vel of the chaser when new path is calculated
						//chaser_init_pos.x = chaser_real_pos.x;
						//chaser_init_pos.y = chaser_real_pos.y;

						//For simulation ONLY WITH RVIZ
						chaser_init_pos.x = new_x;
						chaser_init_vel_X = new_vel_x;

						ros::spinOnce();

						target_init_pos.x = target_real_pos.x;

						setup_planning_parameters(A_MAX_X,
													L_X,
													chaser_init_pos.x,
													target_init_pos.x,
													target_vel_X,
													chaser_init_vel_X,
													L_X);

						decide_plan_of_action_X();

						//reset timer for this axis
						init_time_X = ros::Time::now();
					}

				}

				if(update_path_Y){

					double diff_Y = fabs(obs_vel_Y - target_vel_Y);

					ROS_INFO("t_vel_Y %lf , obs_vel_Y %lf, diff_Y %lf", target_vel_Y ,obs_vel_Y, diff_Y);

					if( diff_Y > 0.007 ){

						target_vel_Y = obs_vel_Y;

						ROS_INFO("Found change in target's velocity in Y axis! \n Recalculating path.....");

						//store the last velocity of the chaser ,in order to pass the init vel of the chaser when new path is calculated
						//chaser_init_pos.y = chaser_real_pos.y;

						//For simulation ONLY WITH RVIZ
						chaser_init_pos.y = new_y;
						chaser_init_vel_Y = new_vel_y;

						ros::spinOnce();

						target_init_pos.y = target_real_pos.y;

						setup_planning_parameters(A_MAX_Y,
                                                                        L_Y,
                                                                        chaser_init_pos.y,
                                                                        target_init_pos.y,
                                                                        target_vel_Y,
                                                                        chaser_init_vel_Y,
                                                                        L_Y);

						decide_plan_of_action_Y();

						//reset timer for this axis
						init_time_Y = ros::Time::now();
					}
				}

			//}

			continue;
		}

		//while the controller is enabled produce path
		if(!disable_ctrl_X && !disable_ctrl_Y){		

			//Produce the cmd_pos, cmd_vel, cmd_acc
			set_commands(timer_X.toSec(), timer_Y.toSec(), new_x, new_y, new_vel_x, new_vel_y, new_acc_x, new_acc_y);

			//For position and orientation
			new_pos.pose.position.x = new_x;
			new_pos.pose.position.y = new_y;

			if(head_o_t && new_x != prev_pos.pose.position.x && new_y != prev_pos.pose.position.y){
				heading = atan2(target_real_pos.y - chaser_real_pos.y , target_real_pos.x - chaser_real_pos.x);
				head_o_t = false;
			}

			prev_pos.pose.position.x = new_x;
			prev_pos.pose.position.y = new_y;

			qq = tf::createQuaternionFromYaw(heading);
			//qq = tf::createQuaternionFromYaw(theta_des);		

			new_pos.pose.orientation.x = qq.x() ;
			new_pos.pose.orientation.y = qq.y() ;
			new_pos.pose.orientation.z = qq.z() ;
			new_pos.pose.orientation.w = qq.w() ;

			//For velocity
			new_vel.twist.linear.x = new_vel_x;
			new_vel.twist.linear.y = new_vel_y;

			//std::cout<<"new_vel_x "<<new_vel_x<<" new_vel_y"<<new_vel_y<<std::endl;

			/*
			   if(dt.toSec() != 0.0)
			   new_vel.twist.angular.z = (heading - prev_heading) / dt.toSec();
			   else
			   new_vel.twist.angular.z = 0.0;
			 */
			new_vel.twist.angular.z = 0.0;

			prev_heading = heading;		

			//std::cout<<"heading "<<heading<<std::endl;

			//For acceleration
			new_acc.x = new_acc_x;
			new_acc.y = new_acc_y;
			new_acc.z = 0.0;

			//Send commands to base controller
			pos_pub.publish(new_pos);
			vel_pub.publish(new_vel);
			acc_pub.publish(new_acc);

			//For visualizing the path in rviz
			path.poses.push_back(new_pos);
			new_pos.header.stamp = ros::Time::now();	
			path_pub.publish(path);

		}

		loop_rate.sleep();

		//std::cout<<"timer_X"<<timer_X<<" timer_Y"<<timer_Y<<" timer_glb"<<timer_glb<<std::endl;
	}

}
