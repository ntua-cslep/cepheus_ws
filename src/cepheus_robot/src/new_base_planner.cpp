/*

   About the chaser's movement:

   There are 3 velocity profiles depending on the target's position
   and the sign of the target's velocity
   We can have either of the 3 profiles in each axis idependent from each other

   Profile 1: (vel_profile = 1) means that the target is moving away form the chaser and that the chaser is going to accelerate
   with full acceleration and the deaccelerate in order to reach the target

   Profile 2: (vel_profile = 2) means that the chaser is going to accelerate
   with an acceleration calculated at that time in order to reach the
   target's velocity and then move with target's speed along with the target

   Profile 3: (vel_profile = 3) The chaser moves toward the target and in time it changes it's direction in order to allign with
   the target and move with it

 */

#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

//for clock_gettime and monotonic clocks
#include <time.h>
#define NANO_TO_MICRO_DIVISOR 1000
#include <ros/ros.h>
#define RT_PRIORITY 95

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>

#include "digital_filter.h"
#include "new_base_planner_utilities.h"

DigitalFilter tar_x_fir(10, 0.0);
DigitalFilter tar_y_fir(10, 0.0);
DigitalFilter ch_x_fir(10, 0.0);
DigitalFilter ch_y_fir(10, 0.0);

DigitalFilter tar_xd_fir(10, 0.0);
DigitalFilter tar_yd_fir(10, 0.0);


//These flags will be true when the vel prof is over and the controller must be disabled
//Both flags must be true in order to disable ctrl
bool disable_ctrl_X = false;
bool disable_ctrl_Y = false;


//Create the constraints for the experiment
Geometric_Constraints constraints(1.897, 2.329, ROBOT_RADIUS, WS_RADIUS, 0.5);

//Creating 6 possbilly used profiles
//2 for each profile (x,y axis)
//Maybe there is a better solution...
Prf1 p1_X; Prf1 p1_Y;
Prf2 p2_X; Prf2 p2_Y;
Prf3 p3_X; Prf3 p3_Y;

//values {1,2,3}
//indcates which vel prof be used in each axis
short velocity_profile_X = 0, velocity_profile_Y = 0;

//The maximum acceleration of the chaser
double A_MAX;

//The the acceleration in each axis
//sign of acceleration depends on the direction
double A_MAX_X, A_MAX_Y;

//distance from target used in second profile in order to reverse the
//orientation of the chaser's velocity
double L_X = 1;
double L_Y = 1;

double target_vel_X = 0.0, target_vel_Y = 0.0;

geometry_msgs::Vector3 target_real_pos;
geometry_msgs::Vector3 target_init_pos;
ros::Time target_pos_stamp;

geometry_msgs::Vector3 chaser_init_pos;
geometry_msgs::Vector3 chaser_real_pos;

//The goal pos to move the chaser to
geometry_msgs::Vector3 des_pos;


bool calculated_velocity_of_target = false;
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

double theta_des = 0.0;

void update_des_pos(tf::TransformListener& des_pos_listener, tf::StampedTransform& des_pos_transform){

	double roll, pitch, yaw;

	try{
		des_pos_listener.lookupTransform("/map","/gripper_target", ros::Time(0), des_pos_transform);

		tf::Quaternion q = des_pos_transform.getRotation();
		tf::Matrix3x3 m(q);
		m.getRPY(roll,pitch,yaw);

		//ROS_WARN("yaw %lf",yaw);

		theta_des = M_PI + yaw;		
		double mod = std::fmod(theta_des, 2.0 * M_PI);

		if(mod <= M_PI && mod >= 0 ){
			theta_des = mod;
		}
		else{
			theta_des = mod - 2.0 * M_PI;
		}


		des_pos.x = des_pos_transform.getOrigin().x() + (WS_RADIUS + ROBOT_RADIUS) * cos(yaw);
		des_pos.y = des_pos_transform.getOrigin().y() + (WS_RADIUS + ROBOT_RADIUS) * sin(yaw);
		//des_pos.x = des_pos_transform.getOrigin().x();
		//des_pos.y = des_pos_transform.getOrigin().y();


		//ROS_WARN("des pos_x : %lf, des_pos_y %lf", des_pos.x, des_pos.y);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
	}
}


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

		cut_digits(chaser_init_pos.x, 3);	
		cut_digits(chaser_init_pos.y, 3);


		chaser_real_pos.x = temp.transform.translation.x;
		chaser_real_pos.y = temp.transform.translation.y;
		chaser_real_pos.z = yaw;

		cut_digits(chaser_real_pos.x, 3);
		cut_digits(chaser_real_pos.y, 3);


		chaser_first_time = false;
	}

	else{
		chaser_real_pos.x = temp.transform.translation.x;
		chaser_real_pos.y = temp.transform.translation.y;
		chaser_real_pos.z = yaw;

		chaser_real_pos.x = ch_x_fir.filter(chaser_real_pos.x);
		chaser_real_pos.y = ch_y_fir.filter(chaser_real_pos.y);

		cut_digits(chaser_real_pos.x, 3);
		cut_digits(chaser_real_pos.y, 3);

	}

	//ROS_INFO("phaseSpace called");
	return;
}

void PhaseSpaceCallbackTarget(const geometry_msgs::TransformStamped::ConstPtr& msg)
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

	if(target_first_time){
		target_init_pos.x = temp.transform.translation.x;
		target_init_pos.y = temp.transform.translation.y;
		target_init_pos.z = yaw;

		cut_digits(target_init_pos.x, 3);
		cut_digits(target_init_pos.y, 3);


		target_pos_stamp = temp.header.stamp;
		target_real_pos.x = temp.transform.translation.x;
		target_real_pos.y = temp.transform.translation.y;
		target_real_pos.z = yaw;

		cut_digits(target_real_pos.x, 3);
		cut_digits(target_real_pos.y, 3);

		target_first_time = false;
	}
	else{

		target_pos_stamp = temp.header.stamp;
		target_real_pos.x = temp.transform.translation.x;
		target_real_pos.y = temp.transform.translation.y;
		target_real_pos.z = yaw;

		target_real_pos.x = tar_x_fir.filter( target_real_pos.x );
		target_real_pos.y = tar_y_fir.filter( target_real_pos.y );

		cut_digits(target_real_pos.x, 3);
		cut_digits(target_real_pos.y, 3);

	}


	//std::cout<<target_pos_stamp.toSec()<<" "<<target_real_pos.x<<std::endl;;
	return;
}

void startChaseCallback(const std_msgs::Bool::ConstPtr& msg){
	start_planning = msg->data;	
}


//---------------------------
void observate_target_velocity(double dt, double& target_vel_X, double& target_vel_Y){

	static geometry_msgs::Vector3 target_prev_pos;


	target_prev_pos.x = target_real_pos.x;
	target_prev_pos.y = target_real_pos.y;

	sleep(dt);

	ros::spinOnce();

	if(dt > 0){
		ROS_INFO("trp %lf tpp %lf" ,target_real_pos.x ,  target_prev_pos.x);

		target_vel_X = (target_real_pos.x - target_prev_pos.x)/dt;
		target_vel_Y = (target_real_pos.y - target_prev_pos.y)/dt;

		cut_digits(target_vel_X, 2);
		cut_digits(target_vel_Y, 2);

                if(target_vel_X <= 0.01 && target_vel_X >= -0.01){
                        target_vel_X = 0.0;
                }

                if(target_vel_Y <= 0.01 && target_vel_Y >= -0.01){
                        target_vel_Y = 0.0;
                }


		std::cout<<"Observated target vel_X: "<<target_vel_X<<std::endl;
		std::cout<<"Observated target vel_Y: "<<target_vel_Y<<std::endl;
	}
}

void calculate_target_velocity(double dt, double& target_vel_X, double& target_vel_Y){

	static bool first_time = true;
	static geometry_msgs::Vector3 target_prev_pos;
	static ros::Time target_prev_pos_stamp;
	double x,y,z;


	if(first_time){

		target_prev_pos.x = target_real_pos.x;
		target_prev_pos.y = target_real_pos.y;
		target_prev_pos_stamp = target_pos_stamp; 
		first_time = false;
	}


	x = target_real_pos.x;
	y = target_real_pos.y;

	//ROS_WARN("dt %lf",dt);

	if(dt != 0){
		target_vel_X = (x - target_prev_pos.x)/dt;
		target_vel_Y = (y - target_prev_pos.y)/dt;
		//Real vel
		//std::cout<<target_pos_stamp.toSec()<<" "<<target_vel_X<<std::endl;

		target_vel_X = tar_xd_fir.filter(target_vel_X);
		target_vel_Y = tar_yd_fir.filter(target_vel_Y);
		//Filtered vel
		//std::cout<<" "<<target_vel_X<<std::endl;

		cut_digits(target_vel_X, 2);
		cut_digits(target_vel_Y, 2);

		if(target_vel_X <= 0.01 && target_vel_X >= -0.01){
			target_vel_X = 0.0;
		}

                if(target_vel_Y <= 0.01 && target_vel_Y >= -0.01){
                        target_vel_Y = 0.0;
                }


		//std::cout<<target_vel_X<<std::endl;	
	}

	target_prev_pos.x = x;
	target_prev_pos.y = y;
	target_prev_pos_stamp = target_pos_stamp; 

	//Real pos
	//std::cout<<target_pos_stamp.toSec()<<" "<<target_real_pos.x<<std::endl;
	//Filtered pos
	//std::cout<<target_pos_stamp.toSec()<<" "<<x<<std::endl;


	//ROS_INFO("vel x: %lf, vel y: %lf", target_vel_X, target_vel_Y);
}


void setup_planning_parameters()
{
	double Fmax_X = 2.0*cos(M_PI/3.0) * FMAX_THRUST;
	A_MAX = Fmax_X / CHASER_MASS;

	if(target_vel_X != 0){

		if(target_vel_X > 0){
			A_MAX_X = A_MAX;
		}
		else{
			A_MAX_X = -A_MAX;
		}

	}
	else if (chaser_init_pos.x < target_init_pos.x){
		A_MAX_X = A_MAX;
	}
	else{
		A_MAX_X = - A_MAX;
	}

	if(target_vel_Y != 0){

		if(target_vel_Y > 0){
			A_MAX_Y = A_MAX;
		}
		else{
			A_MAX_Y = -A_MAX;
		}

	}
	else if (chaser_init_pos.y < target_init_pos.y){
		A_MAX_Y = A_MAX;
	}
	else{
		A_MAX_Y = -A_MAX;
	}



	if (chaser_init_pos.x > target_init_pos.x){
		L_X = - L_X;
	}

	if (chaser_init_pos.y > target_init_pos.y){
		L_Y = - L_Y;
	}

	ROS_INFO("Planning params: A_MAX = %lf , A_MAX_X = %lf, A_MAX_Y = %lf  L_X = %lf , L_Y = %lf", A_MAX, A_MAX_X, A_MAX_Y, L_X, L_Y);
}

void calc_vel_prof_1_params(const double& A_max,
		const double& V_DES,
		const double& INIT_CH,
		const double& init_des,
		Prf1& res)
{
	ROS_WARN("A_max %lf V_DES %lf INIT_CH %lf init_des %lf",A_max, V_DES, INIT_CH, init_des);

	double t1,t2,xdes_target,xt1,vt1,xdes_chaser;

	double a = A_max;
	double b = -2.0 * V_DES;
	double c = INIT_CH - init_des + 0.5 * pow(V_DES,2)/A_max;

	double delta = b*b - 4.0 * a * c;

	if(delta < 0){
		ROS_WARN("Cannot catch target. Delta < 0");
		exit(1);
	}
	else if(delta > 0){    

		double s1 = (-b + sqrt(delta)) / (2.0 * a);
		double s2 = (-b - sqrt(delta)) / (2.0 * a);


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
			ROS_WARN("Cannot catch target. Two neg solutions < 0");
			exit(2);
		}

		//ROS_WARN("a %lf b %lf c %lf  delta %lf s1 %lf s2 %lf t1 %lf",a,b,c, delta, s1,s2,t1);

	}
	else{

		double s1 = -b / (2.0 * a);
		if(s1 > 0){
			t1 = s1;
		}
		else{
			ROS_WARN("Cannot catch target. Double neg solution");
			exit(3);
		}

		//ROS_WARN("delta %lf s1 %lf t1 %lf", delta, s1 ,t1);
	}



	t2 = 2.0 * t1 - V_DES / A_max;

	xdes_target = V_DES * t2 + init_des;

	xt1 = INIT_CH + 0.5*A_max*(t1*t1);
	vt1 = A_max * t1;
	xdes_chaser = xt1 + vt1 * (t2 - t1) - 0.5 * A_max * pow(t2 - t1,2);

	res.set_vals(t1, t2, xdes_target, xt1, vt1, xdes_chaser);
}

void calc_vel_prof_2_params(const double& INIT_CH,
		const double& INIT_TAR,
		const double&  V_DES,
		Prf2& res)
{
	double t1,a_ch,xdes_chaser;

	t1 = 2.0 * (INIT_CH - INIT_TAR)/V_DES;
	a_ch = V_DES/t1;
	xdes_chaser = INIT_CH + 0.5 *a_ch*pow(t1,2);

	res.set_vals(t1, xdes_chaser, a_ch);
}

void calc_vel_prof_3_params(const double& A_MAX,
		const double& V_DES,
		const double& INIT_CH,
		const double& INIT_TAR,
		const double& L,
		const double& init_des,
		Prf3& res)
{
	double t1,t2,t3,a3,Xt1,Xt2,Vt1,xdes_chaser,xdes_target;


	double a_max;
	if(V_DES > 0){
		a_max = -A_MAX;
	}
	else{
		a_max = A_MAX;
	}

	double a = a_max / 4.0;
	double b = -V_DES;
	double c = INIT_CH - INIT_TAR + L - pow(V_DES,2) / 2.0*a_max;

	double delta = b*b - 4.0 * a * c;

	if (delta < 0){
		ROS_WARN("Delta < 0");
		exit(6);
	} 
	else if (delta > 0){   

		double s1 = (-b + sqrt(delta)) / (2.0 * a);
		double s2 = (-b - sqrt(delta)) / (2.0 * a);

		if(s1 > 0){
			if(s1 < s2){
				t2 = s1;
			}
			else if(s2 > 0){
				t2 = s2;
			}
			else{
				t2 = s1;
			}
		}
		else if(s2 > 0){
			t2 = s2;
		}
		else{
			ROS_WARN("Cannot catch target. Two neg solutions < 0");
			exit(4);
		}    

	}
	else{

		double s1 = -b / (2.0 * a);

		if(s1 > 0){
			t2 = s1;
		}
		else{
			ROS_WARN("Cannot catch target. Double neg solution");
			exit(5);
		}
	}


	t1 = t2/2.0;
	t3=(L - 0.5 * V_DES * t2)/(-0.5 * V_DES);
	a3 = (L + V_DES * (t3 - t2))/(0.5 * pow(t3 - t2,2) );

	if(V_DES > 0){
		a_max = -A_MAX;
	}
	else{
		a_max = A_MAX;
	}

	Vt1 = t1 * a_max;
	Xt1 = INIT_CH + 0.5 * a_max * pow(t1,2);
	Xt2 = Xt1 + Vt1 * (t2 - t1) - 0.5 * a_max * pow(t2 - t1,2);
	xdes_chaser = INIT_TAR + V_DES * t3;

	xdes_target=V_DES*t3+init_des;

	res.set_vals(t1, t2, t3, a3, Vt1, Xt1, Xt2, xdes_chaser, xdes_target);
}


void decide_plan_of_action()
{
	//for checking the table contraints
	double meet_point_x, meet_point_y;

	//-----------FOR X AXIS------------

	//Target almost still 
	if(target_vel_X == 0.0){

		calc_vel_prof_1_params(A_MAX_X, target_vel_X, chaser_init_pos.x, des_pos.x, p1_X);
		meet_point_x = p1_X.xdes_chaser;
		velocity_profile_X = (short)VEL_PROF_1;
		p1_X.print();
	}
	//target moving away
	else if((chaser_init_pos.x <= target_init_pos.x && target_vel_X > 0) || (chaser_init_pos.x >= target_init_pos.x && target_vel_X  < 0)){

                calc_vel_prof_1_params(A_MAX_X, target_vel_X, chaser_init_pos.x, des_pos.x, p1_X);
                meet_point_x = p1_X.xdes_chaser;
                velocity_profile_X = (short)VEL_PROF_1;
                p1_X.print();
        }


	// 	Target is aproacing the chaser
	// In order to decide the velocity profile for the movement of the chaser
	// we have to calculate the time needed for the target to reach the chaser
	// if the time needed is smaller than a threshold we coose we have to choose the second profile 
	// else the third
	else{

	}


	//------------FOR Y AXIS--------------

	//Target almost still
	if(target_vel_Y == 0.0){

                calc_vel_prof_1_params(A_MAX_Y, target_vel_Y, chaser_init_pos.y, des_pos.y, p1_Y);
                meet_point_y = p1_Y.xdes_chaser;
                velocity_profile_Y = (short)VEL_PROF_1;
                p1_Y.print();

        }

	//Target is moving away
	else if((chaser_init_pos.y <= target_init_pos.y && target_vel_Y > 0) || (chaser_init_pos.y >= target_init_pos.y && target_vel_Y  < 0)){

		calc_vel_prof_1_params(A_MAX_Y, target_vel_Y, chaser_init_pos.y, des_pos.y, p1_Y);
		meet_point_y = p1_Y.xdes_chaser;
		velocity_profile_Y = (short)VEL_PROF_1;
		p1_Y.print();

	}

	// 	Target is aproacing the chaser
	// In order to decide the velocity profile for the movement of the chaser
	// we have to calculate the time needed for the target to reach the chaser
	// if the time needed is smaller than a threshold we coose we have to choose the second profile 
	// else the third
	else{

	}
	/*
	   if( !(constraints.in_constraints(meet_point_x, meet_point_y)) ){
	   ROS_WARN("MEETING POINT OUT OF LIMITS! ABORTING.....");
	   exit(12);
	   }
	   else{
	   ROS_WARN("CEPHEUS DECIDED: X-axis -> VEL_PROF: %d ,Y-axis -> VEL_PROF: %d chaser_init pos_X %lf chaser_init pos_Y %lf", velocity_profile_X, velocity_profile_Y, chaser_init_pos.x, chaser_init_pos.y);
	   }
	 */
}

void  produce_chaser_trj_points_and_vel_prof_1 (const double& t,
		const double& INIT_CH,
		const double& A_max,
		const double& V_DES,
		const Prf1& prof_params,
		double& cmd_pos,
		double& cmd_vel,
		double& cmd_acc)
{

	if (t <= prof_params.t1){
		ROS_INFO("Accelerate =>");
		cmd_pos = INIT_CH + 0.5 * A_max * pow(t,2);
		cmd_vel = A_max * t;//u0 = 0
		cmd_acc = A_max;
	}
	else if (t > prof_params.t1 && t <= prof_params.t2){
		ROS_INFO("Deaccelerate =>");
		cmd_pos = prof_params.xt1 + prof_params.vt1*(t - prof_params.t1) - 0.5 * A_max * pow(t - prof_params.t1,2);
		cmd_vel = prof_params.vt1 - A_max * (t - prof_params.t1); 
		cmd_acc = -A_max; 
	}
	else if (t > prof_params.t2 && t <= prof_params.duration_with_active_ctrl){
		ROS_INFO("Stand Still...");
		cmd_pos = prof_params.xdes_chaser + V_DES * (t - prof_params.t2);
		cmd_vel = V_DES;
		cmd_acc = 0.0;
	}
	else{
	//	ROS_WARN("PROF1 MUST DISABLE CTRL!");
	}
}

void produce_chaser_trj_points_and_vel_prof_2 (const double& t,
		const double&INIT_CH,
		const double& V_DES,
		const Prf2& prof_params,
		double& cmd_pos,
		double& cmd_vel,
		double& cmd_acc)
{

	if (t <= prof_params.t1){
		cmd_pos = INIT_CH + 0.5 * prof_params.a_ch * pow(t,2);
		cmd_vel = prof_params.a_ch * t;
		cmd_acc = prof_params.a_ch;
	}
	else if(t > prof_params.t1 && t <= prof_params.duration_with_active_ctrl){
		cmd_pos = prof_params.xdes_chaser + V_DES*(t - prof_params.t1);
		cmd_vel = V_DES;
		cmd_acc = 0.0;
	}
	else{
	//	ROS_WARN("PROF2 MUST DISABLE CTRL!");
	}	
}

void produce_chaser_trj_points_and_vel_prof_3 (const double& t,
		const double& INIT_CH,
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

		cmd_pos = INIT_CH + 0.5 * a_max_axis * pow(t,2);
	}    
	else if (t <= prof_params.t2){

		if(V_DES > 0){
			a_max_axis = -A_MAX;
		}
		else{
			a_max_axis = A_MAX;
		}

		cmd_pos = prof_params.Xt1 + prof_params.Vt1 * (t - prof_params.t1) - 0.5 * a_max_axis * pow(t - prof_params.t1,2);
	}    
	else if (t <= prof_params.t3){
		cmd_pos = prof_params.Xt2 + 0.5 * prof_params.a3 * pow(t - prof_params.t2, 2);
	}
	else if(t > prof_params.t3 && t <= prof_params.duration_with_active_ctrl){
		cmd_pos = prof_params.xdes_chaser + V_DES * (t - prof_params.t3);
	}
	else{
	//	ROS_WARN("PROF3 MUST DISABLE CTRL!");
	}

}

void wait_to_smooth_error(double dur){

	ros::Duration timer_norm;
	ros::Time init_time_norm = ros::Time::now();
	//in order to normalize pos
	do{
		ros::spinOnce();
		timer_norm = ros::Time::now() - init_time_norm;

	}while(timer_norm.toSec() < dur);

	target_first_time = chaser_first_time = true;
}


void set_commands(const double& t,
		double& new_x,
		double& new_y,
		double& new_vel_x,
		double& new_vel_y,
		double& new_acc_x,
		double& new_acc_y)
{

	if(velocity_profile_X == (short)VEL_PROF_1){

		produce_chaser_trj_points_and_vel_prof_1(t, chaser_init_pos.x, A_MAX_X, target_vel_X, p1_X , new_x, new_vel_x, new_acc_x);
		if(t > p1_X.duration_with_active_ctrl){
			disable_ctrl_X = true;
		}
	}
	else if(velocity_profile_X == (short)VEL_PROF_2){

		produce_chaser_trj_points_and_vel_prof_2(t, chaser_init_pos.x, target_vel_X, p2_X , new_x, new_vel_x, new_acc_x);
		if(t > p2_X.duration_with_active_ctrl){
			disable_ctrl_X = true;
		}
	}
	else if(velocity_profile_X == (short)VEL_PROF_3){

		produce_chaser_trj_points_and_vel_prof_3(t, chaser_init_pos.x, A_MAX_X, target_vel_X, p3_X , new_x, new_vel_x, new_acc_x);
		if(t > p3_X.duration_with_active_ctrl){
			disable_ctrl_X = true;
		}
	}

	if(velocity_profile_Y == (short)VEL_PROF_1){

		produce_chaser_trj_points_and_vel_prof_1(t, chaser_init_pos.y, A_MAX_Y, target_vel_Y, p1_Y, new_y, new_vel_y, new_acc_y);
		if(t > p1_Y.duration_with_active_ctrl){
			disable_ctrl_Y = true;
		}
	}
	else if(velocity_profile_Y == (short)VEL_PROF_2){

		produce_chaser_trj_points_and_vel_prof_2(t, chaser_init_pos.y, target_vel_Y, p2_Y , new_y, new_vel_y, new_acc_y);
		if(t > p2_Y.duration_with_active_ctrl){
			disable_ctrl_Y = true;
		}

	}
	else if(velocity_profile_Y == (short)VEL_PROF_3){

		produce_chaser_trj_points_and_vel_prof_3(t, chaser_init_pos.y, A_MAX_Y, target_vel_Y, p3_Y , new_y, new_vel_y, new_acc_y);
		if(t > p3_Y.duration_with_active_ctrl){
			disable_ctrl_Y = true;
		}

	}


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

	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("new_path", 1000);

	//Send cmds to base controller node
	ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("planner_pos", 1);
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("planner_vel", 1);
	ros::Publisher acc_pub = nh.advertise<geometry_msgs::Vector3>("planner_acc", 1);


	//Trajectory point produced ,command velocity and command acceleration
	double new_x, new_y, new_vel_x, new_vel_y, new_acc_x, new_acc_y;
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

        //For enabling/disabling base controller
        std_srvs::SetBool srv;

        double heading;
        tf::Quaternion qq;

	ROS_WARN("Planner is waiting for cmd to start in topic \"/start_chase\"...................");
	while(!start_planning){
		ros::spinOnce();
		sleep(1);
	}	

	ROS_WARN("Planner is starting the observation and decision process...................");

	wait_to_smooth_error(0.5);
	observate_target_velocity(1,target_vel_X, target_vel_Y);
	setup_planning_parameters();
	update_des_pos(des_pos_listener, des_pos_transform);
	decide_plan_of_action();

	ROS_WARN("Planner is starting to produce the trajecory");

	ros::Rate loop_rate(200);

        ros::Duration dt;
        ros::Time prev_time = ros::Time::now();

        ros::Time init_time = ros::Time::now();
        ros::Duration timer;

	while(!g_request_shutdown){

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

		//both velocity profiles traj is over 
		if(disable_ctrl_X && disable_ctrl_Y){
			ROS_WARN("REQUEST_TO_DISABLE_CTRL");
			//request to disable controller
			srv.request.data = false;
			if (controller_srv_client.call(srv)) {
				ROS_INFO_STREAM("Controller response: " << srv.response.message);
			}
			else{
				ROS_ERROR("Failed to call Controller");
			}
		}


		dt = ros::Time::now() - prev_time;
		prev_time = ros::Time::now();
		timer = ros::Time::now() - init_time;

		ros::spinOnce();

		calculate_target_velocity(dt.toSec(), target_vel_X, target_vel_Y);
		update_des_pos(des_pos_listener, des_pos_transform);

		set_commands(timer.toSec(), new_x, new_y, new_vel_x, new_vel_y, new_acc_x, new_acc_y);
		//ROS_WARN("new_x %lf new_y %lf new_vel_x %lf new_vel_y %lf new_acc_x %lf new_acc_y %lf",new_x, new_y, new_vel_x, new_vel_y, new_acc_x, new_acc_y);


		//For position and orientation
		new_pos.pose.position.x = new_x;
		new_pos.pose.position.y = new_y;
		//ROS_WARN("new y %lf prev y %lf ", new_y,  prev_pos.pose.position.y);

		//std::cout<<timer.toSec()<<" "<<new_y<<std::endl;
		//heading = atan2(new_y - prev_pos.pose.position.y ,new_x - prev_pos.pose.position.x);
		//ROS_WARN("dy %lf dx %lf heading %lf", new_y - prev_pos.pose.position.y,  new_x - prev_pos.pose.position.x, heading);
		prev_pos.pose.position.x = new_x;
		prev_pos.pose.position.y = new_y;

		//qq = tf::createQuaternionFromYaw(heading);
		qq = tf::createQuaternionFromYaw(theta_des);		

		new_pos.pose.orientation.x = qq.x() ;
		new_pos.pose.orientation.y = qq.y() ;
		new_pos.pose.orientation.z = qq.z() ;
		new_pos.pose.orientation.w = qq.w() ;

		//ROS_WARN("new_x %lf , new_y %lf",new_x, new_y);

		//For velocity
		new_vel.twist.linear.x = new_vel_x;
		new_vel.twist.linear.y = new_vel_y;
		//new_vel.twist.angular.z = 0.0;

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
		

		loop_rate.sleep();
	}

}


