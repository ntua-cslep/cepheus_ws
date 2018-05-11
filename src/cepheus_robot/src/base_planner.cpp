#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

//for clock_gettime and monotonic clocks
#include <time.h>
#define NANO_TO_MICRO_DIVISOR 1000
FILE *latency_fp;
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

geometry_msgs::TransformStamped ps_transform;
geometry_msgs::PoseStamped cmd_pose;
double probe_offset;

tf::TransformListener* listener;

nav_msgs::Path path;
Eigen::MatrixXd path_matrix;

double path_t_step;
double target_speed;
double target_accel;
bool new_path=false;

void PhaseSpaceCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	ps_transform = *msg;


 //-------------------------------------------------
  //---Panagiotis Mavridis 24/04/2018
  /*
  CALCULATING MONOTONIC CLOCK TIME DIFFERENCE 
  */
        static int m_counter = 0;
      struct timespec ts_arrived;
      clock_gettime(CLOCK_MONOTONIC_RAW, &ts_arrived);

      long latency = ((ts_arrived.tv_sec - ps_transform.header.stamp.sec) * 1000000000 + (ts_arrived.tv_nsec - ps_transform.header.stamp.nsec))/NANO_TO_MICRO_DIVISOR;
      //std::cout << "LATENCY IN PHASE_SPACE->CONTROLLER  : "<< latency << std::endl;

        m_counter++;
        fprintf(latency_fp,"%ld\n",latency);

  //--i----------------------------------------------

	return;
}

void moveBaseSimpleCallback(const geometry_msgs::PoseStamped::ConstPtr& cmd_p)
{   
	ROS_INFO_STREAM("move simple begins!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	cmd_pose = *cmd_p;
	cmd_pose.header.stamp = ros::Time::now() - ros::Duration(0.01);
	geometry_msgs::PoseStamped cmd_pose_in_drogue;

	tf::StampedTransform ps_transform_in_drogue, probe2cepheus;
	try{
		listener->transformPose("drogue",cmd_pose,cmd_pose_in_drogue);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ROS_ERROR_STREAM("Planner requested Pose Couldn't translated from: " << cmd_pose.header.frame_id << "frame to: drogue");
	}
	try{
		listener->lookupTransform("drogue", "cepheus", ros::Time(0), ps_transform_in_drogue);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ROS_ERROR_STREAM("requested tf Couldn't found");
	}

	geometry_msgs::PoseStamped step_pose;
	step_pose.header.frame_id = "drogue";

	path.header.frame_id = "drogue";
	path.header.stamp = cmd_pose_in_drogue.header.stamp;
	//produce the path


	tf::Quaternion qf(cmd_pose_in_drogue.pose.orientation.x, cmd_pose_in_drogue.pose.orientation.y, cmd_pose_in_drogue.pose.orientation.z, cmd_pose_in_drogue.pose.orientation.w);
	tf::Matrix3x3 mf(qf);
	double qr,qp,thf; 
	mf.getRPY(qr, qp, thf);
	double xf = cmd_pose_in_drogue.pose.position.x - probe_offset*cos(thf);
	double yf = cmd_pose_in_drogue.pose.position.y - probe_offset*sin(thf);

	double x0 = ps_transform_in_drogue.getOrigin().x();
	double y0 = yf + probe_offset*sin(thf); //ps_transform_in_drogue.transform.translation.y;
	double heading = atan2(yf-y0 , xf-x0);

	double t_total = (xf - x0) / (target_speed*cos(heading));

	ROS_INFO_STREAM("t_total: " << t_total);

	int n = (int)(t_total/path_t_step) + 1;
	ROS_INFO_STREAM("Number of position: " << n);

	path_matrix.resize(9, n);

	int count=0;
	for(double t=0; t < t_total; t+=path_t_step)
	{	

		path_matrix(0,count) = x0 + target_speed*t*cos(heading);//x0 + const amount of accel + speed*t-ta
		path_matrix(1,count) = yf;
		path_matrix(2,count) = thf;
		path_matrix(3,count) = target_speed*cos(heading);
		path_matrix(4,count) = 0.0;
		path_matrix(5,count) = 0.0;
		path_matrix(6,count) = 0.0;
		path_matrix(7,count) = 0.0;
		path_matrix(8,count) = 0.0;

		step_pose.header.seq = (uint)count;
		step_pose.header.frame_id = path.header.frame_id;
		step_pose.header.stamp = path.header.stamp + ros::Duration(t);

		step_pose.pose.position.x = path_matrix(0, count);
		step_pose.pose.position.y = path_matrix(1, count);
		step_pose.pose.position.z = 0.0;
		tf::Quaternion q = tf::createQuaternionFromYaw( path_matrix(2,count) );
		step_pose.pose.orientation.x = q.x();
		step_pose.pose.orientation.y = q.y();
		step_pose.pose.orientation.z = q.z();
		step_pose.pose.orientation.w = q.w();
		path.poses.push_back(step_pose);

		count++;
		//ROS_INFO_STREAM(count);
	}

	new_path = true;
	return;
}

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
// Replacement SIGINT handler
void ctrl_C_Handler(int sig)
{
	g_request_shutdown = 1;
}

bool giveGoal(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res)
{

	ROS_INFO_STREAM("give_goal begins!!!!!!!!~~~~~~~~~~~~~~~~~~!!!!@@@@@@@@@@@@");
	tf::StampedTransform transform_f;
	ros::Time now = ros::Time::now();
	try{
		// listener.waitForTransform("assist/assist_robot", "drogue", now, ros::Duration(3.0));
		listener->lookupTransform("/map", "drogue", ros::Time(0), transform_f);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ROS_ERROR("drogue didnt found");
	}

	geometry_msgs::PoseStamped step_pose;
	path.header.frame_id = "drogue";
	path.header.stamp = now;
	//produce the path
	double x0 = ps_transform.transform.translation.x;
	double y0 = ps_transform.transform.translation.y;

	double xf = transform_f.getOrigin().x();
	double yf = transform_f.getOrigin().y();   

	double heading = atan2(yf-y0 , xf-x0);

	xf = xf - probe_offset*cos(heading);
	yf = yf - probe_offset*sin(heading);

	double thf=heading; 

	double ax = target_accel*cos(heading);
	double ay = target_accel*sin(heading);
	double t_acc = target_speed/target_accel;

	double t_no_acc = ( xf - x0 - (0.5)*ax*pow(t_acc,2) ) / (target_speed*cos(heading));
	double t_total = t_acc + t_no_acc;

	ROS_INFO_STREAM("t_total: " << t_total << " = "<< t_acc << " + " << t_no_acc);

	int n = (int)(t_total/path_t_step) + 1;
	ROS_INFO_STREAM("Number of position: " << n);

	path_matrix.resize(9, n);

	int count=0;
	for(double t=0; t < t_total; t+=path_t_step)
	{	
		if (t < t_acc) //pose during acceleration
		{
			path_matrix(0,count) = x0 + 0.5*ax*pow(t,2);
			path_matrix(1,count) = y0 + 0.5*ay*pow(t,2);
			path_matrix(2,count) = thf;
			path_matrix(3,count) = ax*t;
			path_matrix(4,count) = ay*t;
			path_matrix(5,count) = 0.0;
			path_matrix(6,count) = ax;
			path_matrix(7,count) = ay;
			path_matrix(8,count) = 0.0;
		}
		else //pose during constant speed
		{

			path_matrix(0,count) = x0 + (0.5)*ax*pow(t_acc,2) + ax*t_acc*(t-t_acc);//x0 + const amount of accel + speed*t-ta
			path_matrix(1,count) = y0 + (0.5)*ay*pow(t_acc,2) + ay*t_acc*(t-t_acc);
			path_matrix(2,count) = thf;
			path_matrix(3,count) = ax*t_acc;
			path_matrix(4,count) = ay*t_acc;
			path_matrix(5,count) = 0.0;
			path_matrix(6,count) = 0.0;
			path_matrix(7,count) = 0.0;
			path_matrix(8,count) = 0.0;
		}
		step_pose.header.seq = (uint)count;
		step_pose.header.frame_id = path.header.frame_id;
		step_pose.header.stamp = path.header.stamp + ros::Duration(t);

		step_pose.pose.position.x = path_matrix(0, count);
		step_pose.pose.position.y = path_matrix(1, count);
		step_pose.pose.position.z = 0.0;
		tf::Quaternion q = tf::createQuaternionFromYaw( path_matrix(2,count) );
		step_pose.pose.orientation.x = q.x();
		step_pose.pose.orientation.y = q.y();
		step_pose.pose.orientation.z = q.z();
		step_pose.pose.orientation.w = q.w();
		path.poses.push_back(step_pose);

		count++;
		//ROS_INFO_STREAM(count);
	}

	new_path = true;
	return true;
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "base_planner_node", ros::init_options::NoSigintHandler);
	signal(SIGINT, ctrl_C_Handler);

   //-----Panagiotis Mavridis 08/05/2018----------   

    //Node Handler for global callback queue
    ros::NodeHandle nh;

    //....and a handler for phase space callback queue  
    //ros::NodeHandle nh_ps;

    //ros::CallbackQueue phase_space_callback_queue;
    //---------------------------------------------
	
       //--------Panagiotis Mavridis 25/04/2018----------------

        struct sched_param schedParam;
        schedParam.sched_priority = RT_PRIORITY;
        int sched_policy = SCHED_RR;
        sched_setscheduler(0, sched_policy, &schedParam);

        //--------------------------------------------------

	tf::TransformListener lr(ros::Duration(3));
	listener = &lr;

	ros::ServiceClient controller_srv_client = nh.serviceClient<std_srvs::SetBool>("controller_cmd");
	ros::ServiceServer set_position = nh.advertiseService("give_goal", giveGoal);

	ros::param::param<double>("~time_step" , path_t_step , 0.1); 
	ros::param::param<double>("~target_speed", target_speed, 0.01); 
	ros::param::param<double>("~target_accel", target_accel, 0.02); 
	ros::param::param<double>("~probe_offset", probe_offset, 0.525); 

	ros::Duration time_step(path_t_step);

	ros::Subscriber cmd_pose_sub = nh.subscribe("/move_base_simple/goal", 1, moveBaseSimpleCallback);

	//ros::Subscriber phase_space_sub =  nh_ps.subscribe("map_to_cepheus", 1, PhaseSpaceCallback);
	ros::Subscriber phase_space_sub =  nh.subscribe("map_to_cepheus", 1, PhaseSpaceCallback);
        //ros::Subscriber phase_space_sub =  nh.subscribe("map_to_cepheus", 1, PhaseSpaceCallback,ros::TransportHints().tcpNoDelay(true));

	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1000);

        //--------Panagiotis Mavridis 25/04/2018----------------
        //----Create file with Phase Space message latencies in order to plot---


        latency_fp = fopen("/home/mrrobot/Desktop/track_cepheus-to-base_planner-latencies.txt","w");

        //------------------------------------------------------ 


	geometry_msgs::PoseStamped cmd_pos;
	geometry_msgs::TwistStamped cmd_vel;
	cmd_pos.header.frame_id = "drogue";
	cmd_vel.header.frame_id = "drogue";
	geometry_msgs::Vector3 cmd_acc;

	ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("planner_pos", 1);
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("planner_vel", 1);
	ros::Publisher acc_pub = nh.advertise<geometry_msgs::Vector3>("planner_acc", 1);

	bool path_running=false;
	int cnt;

  //-----------------Panagiotis Mavridis 08/05/2018-----


    // 1 thread to answer phase space callbacks from the dedicated queue...
    //ros::AsyncSpinner phase_space_spinner(1, &phase_space_callback_queue); // Use 1 thread
    //phase_space_spinner.start();


 //--------------------------------------------------------


	while(!g_request_shutdown)
	{
		if (new_path) {
			ROS_INFO("New Path calculated");
			new_path=false;
			path_running=true;
			cnt = 0;
			//path_pub.publish(path);

			//request to enable controller
			std_srvs::SetBool srv;
			srv.request.data = true;
			if (controller_srv_client.call(srv)) {
				ROS_INFO_STREAM("Controller response: " << srv.response.message);
			}
			else{
				ROS_ERROR("Failed to call Controller");
			}
		}


		if(path_running) 
		{
			cmd_pos.pose.position.x = path_matrix(0,cnt);
			cmd_pos.pose.position.y = path_matrix(1,cnt);
			cmd_pos.pose.position.z =0.0;
			cmd_pos.pose.orientation = tf::createQuaternionMsgFromYaw(path_matrix(2,cnt));

			cmd_vel.twist.linear.x = path_matrix(3,cnt);
			cmd_vel.twist.linear.y = path_matrix(4,cnt);
			cmd_vel.twist.angular.z = path_matrix(5,cnt);

			cmd_acc.x = path_matrix(6,cnt);
			cmd_acc.y = path_matrix(7,cnt);
			cmd_acc.z = path_matrix(8,cnt);

			//------------------------------------------
			/*Panagiotis Mavridis 24/04/2018
				PUTTING MONOTONIC CLOCK TIMESTAMP IN THE MESSAGE
				IN ORDER TO CALCULATE COMMUNCATION LATENCY 
				BETWEEN THE ROS NODES*/

			//ONLY FOR POS AND VEL FOR NOW	

			struct timespec ts_pos;
			clock_gettime(CLOCK_MONOTONIC_RAW, &ts_pos);
			cmd_pos.header.stamp.sec = ts_pos.tv_sec;
			cmd_pos.header.stamp.nsec = ts_pos.tv_nsec;

			struct timespec ts_vel;
			clock_gettime(CLOCK_MONOTONIC_RAW, &ts_vel);
			cmd_vel.header.stamp.sec = ts_vel.tv_sec;
			cmd_vel.header.stamp.nsec = ts_vel.tv_nsec;
				std::cout<<"TO PUBLISH"<<std::endl;
			//-----------------------------------------
			pos_pub.publish(cmd_pos);
			vel_pub.publish(cmd_vel);
			acc_pub.publish(cmd_acc);	

			cnt++;
			if(cnt >= path_matrix.cols()) {
				path_running = false;

				//request to disable controller
				std_srvs::SetBool srv;
				srv.request.data = false;
				if (controller_srv_client.call(srv)) {
					ROS_INFO_STREAM("Controller response: " << srv.response.message);
				}
				else{
					ROS_ERROR("Failed to call Controller");
				}
			}
		}

	       //---------Panagiotis Mavridis 08/05/2018---------
	        //phase_space_callback_queue.callOne(ros::WallDuration());
	        //------------------------------------------------


		ros::spinOnce(); 
		//wait one time step
		time_step.sleep();
	}

	fclose(latency_fp);
	return 0;
}
