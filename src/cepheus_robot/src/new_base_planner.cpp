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


#include "digital_filter.h"

DigitalFilter x_fir(10, 0.0);
DigitalFilter y_fir(10, 0.0);
DigitalFilter xd_fir(10, 0.0);
DigitalFilter yd_fir(10, 0.0);



#define VEL_PROF_1 1
#define VEL_PROF_2 2
#define VEL_PROF_3 3

//Structs containing the nessesary information for every velocity profile used
typedef struct Prf1{

	double t1;
	double t2;
	double xdes_target;
	double xt1;
	double vt1;
	double xdes_chaser;

	void set_vals(	double t1,
			double t2,
			double xdes_target,
			double xt1,
			double vt1,
			double xdes_chaser)
	{
		this->t1 = t1;
		this->t2 = t2;
		this->xdes_target = xdes_target;
		this->xt1 = xt1;
		this->vt1 = vt1;
		this-> xdes_chaser =  xdes_chaser;
	}
}Prf1;

typedef struct Prf2{

	double t1; 
	double a_ch;

	void set_vals(  double t1,
			double a_ch)
	{
		this->t1 = t1;
		this-> a_ch = a_ch;
	}

}Prf2;

typedef struct Prf3{

	double t1;
	double t2;
	double t3;
	double a3;
	double Vt1;
	double Xt1;
	double Xt2;
	double xdes_chaser;
	double xdes_target;

	void set_vals(  double t1,
			double t2,
			double t3,
			double a3,
			double Vt1,
			double Xt1,
			double Xt2,
			double xdes_chaser,
			double xdes_target)
	{
		this->t1 = t1;
		this->t2 = t2;
		this->t3 = t3;
		this->a3 = a3;
		this->Xt1 = Xt1;
		this->Xt2 = Xt2;
		this->Vt1 = Vt1;
		this-> xdes_chaser =  xdes_chaser;
		this->xdes_target = xdes_target;
	}

}Prf3;

//Creating 6 possbilly used profiles
//2 for each profile (x,y axis)
//Maybe there is a better solution...
Prf1 p1_X; Prf1 p1_Y;
Prf2 p2_X; Prf2 p2_Y;
Prf3 p3_X; Prf3 p3_Y;

//values {1,2,3}
//indcates which vel prof be used in each axis
short velocity_profile_X = 0, velocity_profile_Y = 0;

const double Fmax_thrust = 0.6;//Newton
const double Chaser_mass = 13.5;//kg

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

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
// Replacement SIGINT handler
void ctrl_C_Handler(int sig)
{
	g_request_shutdown = 1;
}

//The path created by the planner stored for any use
nav_msgs::Path path;

//-------Callbacks---------

void PhaseSpaceCallbackChaser(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	static bool first_time = true;

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

	if(first_time){

		chaser_init_pos.x = temp.transform.translation.x;
		chaser_init_pos.y = temp.transform.translation.y;
		chaser_init_pos.z = yaw;

		chaser_real_pos.x = temp.transform.translation.x;
		chaser_real_pos.y = temp.transform.translation.y;
		chaser_real_pos.z = yaw;

		first_time = false;
	}

	else{
		chaser_real_pos.x = temp.transform.translation.x;
		chaser_real_pos.y = temp.transform.translation.y;
		chaser_real_pos.z = yaw;
	}

	//ROS_INFO("phaseSpace called");
	return;
}

void PhaseSpaceCallbackTarget(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	static bool first_time = true;

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

	if(first_time){
		target_init_pos.x = temp.transform.translation.x;
		target_init_pos.y = temp.transform.translation.y;
		target_init_pos.z = yaw;

		target_pos_stamp = temp.header.stamp;
		target_real_pos.x = temp.transform.translation.x;
		target_real_pos.y = temp.transform.translation.y;
		target_real_pos.z = yaw;

		first_time = false;
	}
	else{

		target_pos_stamp = temp.header.stamp;
		target_real_pos.x = temp.transform.translation.x;
		target_real_pos.y = temp.transform.translation.y;
		target_real_pos.z = yaw;
	}


	//std::cout<<target_pos_stamp.toSec()<<" "<<target_real_pos.x<<std::endl;;
	return;
}

//---------------------------


void calculate_target_velocity(double dt, double& target_vel_X, double& target_vel_Y){

	static bool first_time = true;
	static geometry_msgs::Vector3 target_prev_pos;
	static ros::Time target_prev_pos_stamp;
	double x,y,z;


	if(first_time){

		target_prev_pos.x = x_fir.filter(target_real_pos.x);
		target_prev_pos.y = y_fir.filter(target_real_pos.y);
		target_prev_pos_stamp = target_pos_stamp; 
		first_time = false;
	}


	x = x_fir.filter(target_real_pos.x);
	y = y_fir.filter(target_real_pos.y);

	//ROS_WARN("dt %lf",dt);

	if(dt != 0){
		target_vel_X = (x - target_prev_pos.x)/dt;
		target_vel_Y = (y - target_prev_pos.y)/dt;
		//Real vel
		//std::cout<<target_pos_stamp.toSec()<<" "<<target_vel_X<<std::endl;

		target_vel_X = xd_fir.filter(target_vel_X);
		target_vel_Y = yd_fir.filter(target_vel_Y);
		//Filtered vel
                std::cout<<" "<<target_vel_X<<std::endl;
	}

	target_prev_pos.x = x;
	target_prev_pos.y = y;
	target_prev_pos_stamp = target_pos_stamp; 

	//Real pos
	//std::cout<<target_pos_stamp.toSec()<<" "<<target_real_pos.x<<std::endl;
	//Filtered pos
	//std::cout<<target_pos_stamp.toSec()<<" "<<x<<std::endl;
	

}


void setup_planning_parameters()
{
	double Fmax_X = 2*cos(M_PI/6) * Fmax_thrust;
	A_MAX = Fmax_X / Chaser_mass;

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
	double t1,t2,xdes_target,xt1,vt1,xdes_chaser;

	double a = A_max;
	double b = -2 * V_DES;
	double c = INIT_CH - init_des + 1/2 * pow(V_DES,2)/A_max;

	double delta = b*b - 4 * a * c;

	if(delta < 0){
		ROS_WARN("Cannot catch target. Delta < 0");
		return;
	}
	else if(delta > 0){    

		double s1 = (-b + sqrt(delta))/2*a;
		double s2 = (-b - sqrt(delta))/2*a;

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
			return;
		}
	}
	else{

		double s1 = -b /2*a;
		if(s1 > 0){
			t1 = s1;
		}
		else{
			ROS_WARN("Cannot catch target. Double neg solution");
			return;
		}
	}



	t2 = 2*t1 - V_DES / A_max;

	xdes_target = V_DES * t2 + init_des;

	xt1 = INIT_CH + 1/2*A_max*(t1*t1);
	vt1 = A_max * t1;
	xdes_chaser = xt1 + vt1 * (t2 - t1)-1/2*A_max*pow(t2 - t1,2);

	res.set_vals(t1, t2, xdes_target, xt1, vt1, xdes_chaser);
}

void calc_vel_prof_2_params(const double& INIT_CH,
		const double& INIT_TAR,
		const double&  V_DES,
		Prf2& res)
{
	double t1,a_ch;

	t1 = 2*(INIT_CH - INIT_TAR)/V_DES;
	a_ch = V_DES/t1;

	res.set_vals(t1, a_ch);
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

	double a = a_max/4;
	double b = -V_DES;
	double c = INIT_CH - INIT_TAR + L - pow(V_DES,2)/2*a_max;

	double delta = b*b - 4 * a * c;

	if (delta < 0){
		ROS_WARN("Delta < 0");
		return;
	} 
	else if (delta > 0){   

		double s1 = (-b + sqrt(delta))/2*a;
		double s2 = (-b - sqrt(delta))/2*a;

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
			return;
		}    

	}
	else{

		double s1 = -b /2*a;

		if(s1 > 0){
			t2 = s1;
		}
		else{
			ROS_WARN("Cannot catch target. Double neg solution");
			return;
		}
	}


	t1 = t2/2;

	t3=(L - 1/2 * V_DES * t2)/(-1/2 * V_DES);
	a3 = (L + V_DES * (t3 - t2))/(1/2 * pow(t3 - t2,2) );

	if(V_DES > 0){
		a_max = -A_MAX;
	}
	else{
		a_max = A_MAX;
	}

	Vt1 = t1 * a_max;
	Xt1 = INIT_CH + 1/2 * a_max * pow(t1,2);
	Xt2 = Xt1 + Vt1 * (t2 - t1) - 1/2 * a_max * pow(t2 - t1,2);
	xdes_chaser = INIT_TAR + V_DES * t3;

	xdes_target=V_DES*t3+init_des;

	res.set_vals(t1, t2, t3, a3, Vt1, Xt1, Xt2, xdes_chaser, xdes_target);
}


void decide_plan_of_action()
{
	//-----------FOR X AXIS------------

	//Target is moving away or stands still
	if((chaser_init_pos.x <= target_init_pos.x && target_vel_X > 0) || (chaser_init_pos.x >= target_init_pos.x && target_vel_X  < 0)){

		calc_vel_prof_1_params(A_MAX_X, target_vel_X, target_init_pos.x, des_pos.x, p1_X);
		velocity_profile_X = (short)VEL_PROF_1;

		 
	}

	// 	Target is aproacing the chaser
	// In order to decide the velocity profile for the movement of the chaser
	// we have to calculate the time needed for the target to reach the chaser
	// if the time needed is smaller than a threshold we coose we have to choose the second profile 
	// else the third
	else{

	}


	//------------FOR Y AXIS--------------

	//Target is moving away or stands still
	if((chaser_init_pos.y <= target_init_pos.y && target_vel_Y > 0) || (chaser_init_pos.y >= target_init_pos.y && target_vel_Y  < 0)){

		calc_vel_prof_1_params(A_MAX_Y, target_vel_Y, target_init_pos.y, des_pos.y, p1_Y);
		velocity_profile_Y = (short)VEL_PROF_1;
	}
	//The target stands still so the chaser has to approach
	else if(true){



	}

	// 	Target is aproacing the chaser
	// In order to decide the velocity profile for the movement of the chaser
	// we have to calculate the time needed for the target to reach the chaser
	// if the time needed is smaller than a threshold we coose we have to choose the second profile 
	// else the third
	else{

	}


	ROS_WARN("CEPHEUS DECIDED: X-axis -> VEL_PROF: %d ,Y-axis -> VEL_PROF: %d", velocity_profile_X, velocity_profile_Y);
}

double  produce_chaser_trj_points_prof_1 (const double& t,
		const double& INIT_CH,
		const double& A_max,
		const double& V_DES,
		const Prf1& prof_params)
{

	if (t <= prof_params.t1){
		return(INIT_CH + 1/2 * A_max * pow(t,2));
	}
	else if (t > prof_params.t1 && t <= prof_params.t2){
		return(prof_params.xt1 + prof_params.vt1*(t - prof_params.t1) - 1/2 * A_max * pow(t - prof_params.t1,2));
	}
	else{
		return(prof_params.xdes_chaser + V_DES * (t - prof_params.t2));
	}

}

void produce_chaser_trj_points_prof_2 (const double& t,
		const double& t2,
		const double&INIT_CH,
		const double& a_ch,
		const double& V_DES,
		double& x_chaser)
{
	double xdes_chaser = INIT_CH+1/2*a_ch*pow(t2,2);

	if (t<=t2){
		x_chaser = INIT_CH+1/2*a_ch*pow(t,2);
	}
	else{
		x_chaser = xdes_chaser + V_DES*(t-t2);
	}
}

void produce_chaser_trj_points_prof_3 (const double& t,
		const double& t1,
		const double& t2,
		const double& t3,
		const double& Xt1,
		const double& Xt2,
		const double& Vt1,
		const double& A_MAX,
		const double& a3,
		const double& a_max_axis,
		const double& INIT_CH,
		const double& V_DES,
		const double& xdes_chaser,
		double& x_chaser)
{

	if (t<=t1){

		double a_max_axis;

		if(V_DES > 0){
			a_max_axis = -A_MAX;
		}
		else{
			a_max_axis = A_MAX;
		}

		x_chaser = INIT_CH+1/2*a_max_axis*pow(t,2);
	}    
	else if (t<=t2){
		double a_max_axis;

		if(V_DES > 0){
			a_max_axis = -A_MAX;
		}
		else{
			a_max_axis = A_MAX;
		}

		x_chaser = Xt1 + Vt1 * (t - t1) - 1/2 * a_max_axis * pow(t - t1,2);
	}    
	else if (t<=t3){
		x_chaser = Xt2 + 1/2 * a3 * pow(t - t2,2);
	}
	else{
		x_chaser = xdes_chaser + V_DES*(t-t3);
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

	ros::Subscriber phase_space_sub_target =  nh.subscribe("map_to_assist_robot", 1, PhaseSpaceCallbackTarget);
	ros::Subscriber phase_space_sub_chaser =  nh.subscribe("map_to_cepheus", 1, PhaseSpaceCallbackChaser);

	//Trajectory point
	double new_x, new_y;
	geometry_msgs::PoseStamped new_pos;

	ROS_WARN("Planner is starting the observation and decision process...................");
	//calculate_target_velocity(target_vel_X, target_vel_Y);
	setup_planning_parameters();
	decide_plan_of_action();

	ros::Rate loop_rate(200);	
	ros::Duration time;
	ros::Time prev_time = ros::Time::now();

	ROS_WARN("Planner is starting to produce the trajecory");
	while(!g_request_shutdown){

		time = ros::Time::now() - prev_time;
		prev_time = ros::Time::now();

		ros::spinOnce();
		//ROS_INFO("%lf",target_real_pos.x);
		calculate_target_velocity(time.toSec(), target_vel_X, target_vel_Y);

		/*
		   time = ros::Time::now()- init_time;

		   if(velocity_profile_X == (short)VEL_PROF_1){
		   new_x = produce_chaser_trj_points_prof_1(time.toSec(), chaser_init_pos.x, A_MAX_X, target_vel_X, p1_X);
		   }

		   if(velocity_profile_Y == (short)VEL_PROF_1){
		   new_y = produce_chaser_trj_points_prof_1(time.toSec(), chaser_init_pos.y, A_MAX_Y, target_vel_Y, p1_Y);
		   }

		   new_pos.pose.position.x = new_x;
		   new_pos.pose.position.y = new_y;

		   path.poses.push_back(new_pos);
		 */
		loop_rate.sleep();
	}

}


