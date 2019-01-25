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

DigitalFilter tar_x_fir(10, 0.0);
DigitalFilter tar_y_fir(10, 0.0);
DigitalFilter ch_x_fir(10, 0.0);
DigitalFilter ch_y_fir(10, 0.0);

DigitalFilter tar_xd_fir(10, 0.0);
DigitalFilter tar_yd_fir(10, 0.0);



#define VEL_PROF_1 1
#define VEL_PROF_2 2
#define VEL_PROF_3 3

//Restriction added due to the experimental constraints (table size)
//Table size: 2.329 x 1.897
//The constraint is the length of the diagonal
//If the the chaser calculate a meeting point greater thatn this constriaint..
//...it will abort and never move towards the target
double DISTANSE_LIMIT = 2.8; //meters


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

	void print(){

		ROS_INFO("Profile 1 Params:");
		std::cout<<"\t t1: "<<t1<<std::endl;
		std::cout<<"\t t2: "<<t2<<std::endl;
		std::cout<<"\t xt1: "<<xt1<<std::endl;
		std::cout<<"\t vt1: "<<vt1<<std::endl;
		std::cout<<"\t xdes_chaser: "<<xdes_chaser<<std::endl;
		std::cout<<"\t xdes_target: "<<xdes_target<<std::endl;
		
		 std::cout<<"\t Total Time: "<<t1+t2<<"\n"<<std::endl;
	}
}Prf1;

typedef struct Prf2{

	double t1;
	double a_ch;
	double xdes_chaser; 

	void set_vals(  double t1,
			double xdes_chaser,
			double a_ch)
	{
		this->t1 = t1;
		this->xdes_chaser = xdes_chaser;
		this-> a_ch = a_ch;
	}

        void print(){

                ROS_INFO("Profile 2 Params:");
                std::cout<<"\t t1: "<<t1<<std::endl;
		std::cout<<"\t xdes_chaser: "<<xdes_chaser<<std::endl;
                std::cout<<"\t a_ch: "<<a_ch<<std::endl;

		 std::cout<<"\t Total Time: "<<t1<<"\n"<<std::endl;
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

        void print(){

                ROS_INFO("Profile 3 Params:");
                std::cout<<"\t t1: "<<t1<<std::endl;
                std::cout<<"\t t2: "<<t2<<std::endl;
		std::cout<<"\t t3: "<<t3<<std::endl;
		std::cout<<"\t a3: "<<a3<<std::endl;
                std::cout<<"\t Xt1: "<<Xt1<<std::endl;
		std::cout<<"\t Xt2: "<<Xt2<<std::endl;
                std::cout<<"\t Vt1: "<<Vt1<<std::endl;
                std::cout<<"\t xdes_chaser: "<<xdes_chaser<<std::endl;
                std::cout<<"\t xdes_target: "<<xdes_target<<std::endl;
       
		std::cout<<"\t Total Time: "<<t1+t2+t3<<"\n"<<std::endl;
		
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

double WS_RADIUS = 30.0;
double theta_des = 0.0;

void update_des_pos(tf::TransformListener& des_pos_listener, tf::StampedTransform& des_pos_transform){

	double roll, pitch, yaw;

	try{
		des_pos_listener.lookupTransform("/map", "/gripper_target", ros::Time(0), des_pos_transform);

		tf::Quaternion q = des_pos_transform.getRotation();
		tf::Matrix3x3 m(q);
        	m.getRPY(roll,pitch,yaw);

		theta_des = -yaw;		

		des_pos.x = des_pos_transform.getOrigin().x() + WS_RADIUS * cos(yaw);
		des_pos.y = des_pos_transform.getOrigin().y() + WS_RADIUS * sin(yaw);

		//ROS_WARN("yaw : %lf\n", yaw);
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

		std::cout<<target_vel_X<<std::endl;	
	}

	target_prev_pos.x = x;
	target_prev_pos.y = y;
	target_prev_pos_stamp = target_pos_stamp; 

	//Real pos
	//std::cout<<target_pos_stamp.toSec()<<" "<<target_real_pos.x<<std::endl;
	//Filtered pos
	//std::cout<<target_pos_stamp.toSec()<<" "<<x<<std::endl;


	ROS_INFO("vel x: %lf, vel y: %lf", target_vel_X, target_vel_Y);
}


void setup_planning_parameters()
{
	double Fmax_X = 2.0*cos(M_PI/6.0) * Fmax_thrust;
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
	xdes_chaser = xt1 + vt1 * (t2 - t1)-0.5*A_max*pow(t2 - t1,2);

	if (xdes_chaser >= DISTANSE_LIMIT){
		ROS_WARN("MEETING POINT >= DISTANSE_LIMIT IN VEL _PROF_1");
		exit(8);
	}

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

	if (xdes_chaser >= DISTANSE_LIMIT){
                ROS_WARN("MEETING POINT >= DISTANSE_LIMIT IN VEL _PROF_2");
                exit(9);
        }


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

	if (xdes_chaser >= DISTANSE_LIMIT){
                ROS_WARN("MEETING POINT >= DISTANSE_LIMIT IN VEL _PROF_3");
                exit(10);
        }


	xdes_target=V_DES*t3+init_des;

	res.set_vals(t1, t2, t3, a3, Vt1, Xt1, Xt2, xdes_chaser, xdes_target);
}


void decide_plan_of_action()
{
	//-----------FOR X AXIS------------

	//Target is moving away or stands still
	if((chaser_init_pos.x <= target_init_pos.x && target_vel_X > 0) || (chaser_init_pos.x >= target_init_pos.x && target_vel_X  < 0)){

		calc_vel_prof_1_params(A_MAX_X, target_vel_X, chaser_init_pos.x, target_real_pos.x, p1_X);
		velocity_profile_X = (short)VEL_PROF_1;
		p1_X.print();
	}
	//The target stands still so the chaser has to approach
	else if(target_vel_X == 0.0){

		calc_vel_prof_1_params(A_MAX_X, target_vel_X, chaser_init_pos.x, target_real_pos.x, p1_X);
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

	//Target is moving away or stands still
	if((chaser_init_pos.y <= target_init_pos.y && target_vel_Y > 0) || (chaser_init_pos.y >= target_init_pos.y && target_vel_Y  < 0)){

		calc_vel_prof_1_params(A_MAX_Y, target_vel_Y, chaser_init_pos.y, target_real_pos.y, p1_Y);
		velocity_profile_Y = (short)VEL_PROF_1;
		p1_Y.print();

	}
	//The target stands still so the chaser has to approach
	else if(target_vel_Y == 0.0){

		calc_vel_prof_1_params(A_MAX_Y, target_vel_Y, chaser_init_pos.y, target_real_pos.y, p1_Y);
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


	ROS_WARN("CEPHEUS DECIDED: X-axis -> VEL_PROF: %d ,Y-axis -> VEL_PROF: %d", velocity_profile_X, velocity_profile_Y);
}

void  produce_chaser_trj_points_and_vel_prof_1 (const double& t,
		const double& INIT_CH,
		const double& A_max,
		const double& V_DES,
		const Prf1& prof_params,
		double &cmd_pos,
		double& cmd_vel)
{

	if (t <= prof_params.t1){
		cmd_pos = INIT_CH + 0.5 * A_max * pow(t,2);
		cmd_vel = A_max * t;//u0 = 0
	}
	else if (t > prof_params.t1 && t <= prof_params.t2){
		cmd_pos = prof_params.xt1 + prof_params.vt1*(t - prof_params.t1) - 0.5 * A_max * pow(t - prof_params.t1,2);
		cmd_vel = prof_params.vt1 - A_max * t; // sign correct ? 
	}
	else{
		cmd_pos = prof_params.xdes_chaser + V_DES * (t - prof_params.t2);
		cmd_vel = V_DES;
	}

}

void produce_chaser_trj_points_prof_2 (const double& t,
		const double&INIT_CH,
		const double& V_DES,
		const Prf2& prof_params,
		double& cmd_pos,
		double& cmd_vel)
{

	if (t <= prof_params.t1){
		cmd_pos = INIT_CH + 0.5 * prof_params.a_ch * pow(t,2);
		cmd_vel = prof_params.a_ch * t;
	}
	else{
		cmd_pos = prof_params.xdes_chaser + V_DES*(t - prof_params.t1);
		cmd_vel = V_DES;
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

		x_chaser = INIT_CH + 0.5 * a_max_axis*pow(t,2);
	}    
	else if (t<=t2){
		double a_max_axis;

		if(V_DES > 0){
			a_max_axis = -A_MAX;
		}
		else{
			a_max_axis = A_MAX;
		}

		x_chaser = Xt1 + Vt1 * (t - t1) - 0.5 * a_max_axis * pow(t - t1,2);
	}    
	else if (t<=t3){
		x_chaser = Xt2 + 0.5 * a3 * pow(t - t2,2);
	}
	else{
		x_chaser = xdes_chaser + V_DES*(t-t3);
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
	ros::Subscriber start_planning_sub =  nh.subscribe("start_chase", 1, startChaseCallback);

	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("new_path", 1000);
        
	//Send cmds to base controller node
	ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("planner_pos", 1);
        ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("planner_vel", 1);

	//Trajectory point produced and command velocity
	double new_x, new_y, new_vel_x, new_vel_y;
	geometry_msgs::PoseStamped new_pos;
	geometry_msgs::TwistStamped new_vel;
	tf::Quaternion tf_orientation;
	geometry_msgs::Quaternion new_orientation;

	//The listener for des_pos
	tf::TransformListener des_pos_listener;
	tf::StampedTransform des_pos_transform;


	ROS_WARN("Planner is waiting for cmd to start in topic \"/start_chase\"...................");
	while(!start_planning){
		ros::spinOnce();
		sleep(1);
	}	

	ROS_WARN("Planner is starting the observation and decision process...................");

	wait_to_smooth_error(0.5);
	observate_target_velocity(1,target_vel_X, target_vel_Y);
	setup_planning_parameters();
	decide_plan_of_action();

	ros::Rate loop_rate(200);	
	
	ros::Duration dt;
	ros::Time prev_time = ros::Time::now();
	
	ros::Time init_time = ros::Time::now();
	ros::Duration timer;

	ROS_WARN("Planner is starting to produce the trajecory");
	while(!g_request_shutdown){

		dt = ros::Time::now() - prev_time;
		prev_time = ros::Time::now();
		timer = ros::Time::now() - init_time;

		ros::spinOnce();
		
		calculate_target_velocity(dt.toSec(), target_vel_X, target_vel_Y);
		update_des_pos(des_pos_listener, des_pos_transform);

		if(velocity_profile_X == (short)VEL_PROF_1){
			produce_chaser_trj_points_and_vel_prof_1(timer.toSec(), chaser_init_pos.x, A_MAX_X, target_vel_X, p1_X , new_x, new_vel_x);
		}

		if(velocity_profile_Y == (short)VEL_PROF_1){
			produce_chaser_trj_points_and_vel_prof_1(timer.toSec(), chaser_init_pos.y, A_MAX_Y, target_vel_Y, p1_Y, new_y, new_vel_y);
		}

		//For position and orientation
		new_pos.pose.position.x = new_x;
		new_pos.pose.position.y = new_y;
		//ROS_WARN("%lf %lf %lf", new_x, new_y, timer.toSec());
		tf_orientation.setRPY( 0, 0, theta_des );
		quaternionTFToMsg(tf_orientation , new_orientation);
		new_pos.pose.orientation =  new_orientation;


		//For velocity
		new_vel.twist.linear.x = new_vel_x;
		new_vel.twist.linear.y = new_vel_y;


		pos_pub.publish(new_pos);
		vel_pub.publish(new_vel);
		

		path.poses.push_back(new_pos);
		
		/*new_pos.header.frame_id = "/cepheus";
		new_pos.header.stamp = ros::Time::now();	
		path_pub.publish(path);*/
		
		loop_rate.sleep();
	}

}


