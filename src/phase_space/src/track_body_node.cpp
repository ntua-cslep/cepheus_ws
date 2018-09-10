// standard headers
#include <fstream>
#include <iostream>
#include <vector>

// ROS headers
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Eigen>

// local headers
#include "PoseTrackingFilter.h"
#include "phase_space/PhaseSpaceMarkerArray.h"

//---Panagiotis Mavridis----
//---For monotonic clock----
#include <time.h>
#define NANO_TO_MICRO_DIVISOR 1000
FILE *latency_fp;
#include <sched.h>
#define RT_PRIORITY 95

namespace phase_space {

	class PoseTracker
	{
		private:
			// the node handle
			ros::NodeHandle nh_;

			// ode handle in the private namespace
			ros::NodeHandle priv_nh_;

			// subscribers
			ros::Subscriber sub_phase_space_markers_;

			//publisher
			ros::Publisher pub_transform_;

			tf::TransformListener tf_listener_;
			tf::TransformBroadcaster tf_broadcaster_;

			std::string reference_frame_;
			std::string object_frame_;
			std::vector<int> marker_id;

			// data points, in local reference and measured ones
			int Nleds_;
			Eigen::MatrixXd local_points_;

			// the tracking filter
			PoseTrackingFilter filter_;

			Eigen::VectorXd transfParameters_;

			// the parameters loaded to param server
			XmlRpc::XmlRpcValue leds_;

			// helper functions
			void parseParameters(const XmlRpc::XmlRpcValue &leds);

			void computeOptimalRigidTransformation(Eigen::MatrixXd startP, Eigen::MatrixXd finalP);

		public:

			// callback functions
			void estimatePose(const phase_space::PhaseSpaceMarkerArray & msg);

			// constructor
			PoseTracker(ros::NodeHandle nh, int argc,char** argv) : nh_(nh), priv_nh_("~"), filter_()
		{
			priv_nh_.param<std::string>("reference_frame", reference_frame_, "");
			priv_nh_.param<std::string>("object_frame", object_frame_, "");

			// get the led location in local frame from ros param server
			nh_.getParam(object_frame_ + std::string("_leds"), leds_);
			parseParameters(leds_);

			transfParameters_.resize(7);

			// initialize subscriber
			sub_phase_space_markers_ = nh_.subscribe(nh_.resolveName("/phase_space_markers"), 10, &PoseTracker::estimatePose,this);
			//sub_phase_space_markers_ = nh_.subscribe(nh_.resolveName("/phase_space_markers"), 10, &PoseTracker::estimatePose,this,ros::TransportHints().tcpNoDelay(true)); 
			string tansform_name =reference_frame_+"_to_"+object_frame_;
			pub_transform_= nh.advertise<geometry_msgs::TransformStamped>(nh.resolveName(tansform_name.c_str()), 10);
		}

			//! Empty stub
			~PoseTracker() {}

	};



	void PoseTracker::parseParameters(const XmlRpc::XmlRpcValue &leds)
	{
		ROS_ASSERT(leds.getType() == XmlRpc::XmlRpcValue::TypeArray);

		// if parameters is good, resize the matrix to account for number of leds used to track 
		// 4 =  id + 3Dposition
		local_points_=Eigen::MatrixXd::Zero(leds.size(),4);
		Nleds_ = leds.size();

		for( int i = 0; i < Nleds_; ++i)
		{
			XmlRpc::XmlRpcValue current_led = leds[i];

			// parse ID
			ROS_ASSERT(current_led.getType() == XmlRpc::XmlRpcValue::TypeStruct);
			if( current_led.hasMember("id") )
			{
				ROS_ASSERT(current_led["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
				int id = current_led["id"];
				local_points_(i, 0) = (double) id;
			}
			else
			{
				ROS_ERROR("No id value for the current led. Check the yaml configuration for this object");
				return;
			}

			// parse position
			std::vector<double> position;
			if( current_led.hasMember("position") )
			{
				for (int j = 0; j < current_led["position"].size(); ++j) 
				{
					ROS_ASSERT(current_led["position"][j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
					position.push_back( current_led["position"][j] );
				}
				local_points_(i, 1) = position[0];
				local_points_(i, 2) = position[1];
				local_points_(i, 3) = position[2];
			}
			else
			{
				ROS_ERROR("No double value for the position current led. Check the yaml configuration for this object, remember to use . dots to ensure double format");
				return;
			}
		}

		ROS_INFO("Succesfully parsed all LED parameters!");
		// std::cout << local_points_ << std::endl;

		return;
	}

	void PoseTracker::estimatePose(const phase_space::PhaseSpaceMarkerArray & msg)
	{
		//-------------------------------------------------
		//---Panagiotis Mavridis 24/04/2018
		/*
		   CALCULATING MONOTONIC CLOCK TIME DIFFERENCE 

		   static int m_counter = 0;
		   struct timespec ts_arrived;
		   clock_gettime(CLOCK_MONOTONIC_RAW, &ts_arrived);

		   long latency = ((ts_arrived.tv_sec - msg.header.stamp.sec) * 1000000000 + (ts_arrived.tv_nsec - msg.header.stamp.nsec))/NANO_TO_MICRO_DIVISOR;
		//std::cout << "LATENCY IN PHASE_SPACE->CONTROLLER  : "<< latency << std::endl;

		m_counter++;
		fprintf(latency_fp,"%ld\n",latency);
		 */
		//--i---------------------------------------------


		// measured points is (at most) equal to the number of leds in the model
		// 4 =  id + 3Dposition
		std::vector<Eigen::Vector3d> measured_points;

		// found leds are copied here
		std::vector<Eigen::Vector3d> found_points;
		int counter = 0;

		// apply match criteria (the led id) in order to provide the 
		// available measured points. measured_points are filled in the same order
		// as they appear in local_points
		// is there any other way to do this? 
		for (int i = 0; i < Nleds_; ++i)
		{
			for(size_t j = 0; j < msg.markers.size(); ++j)
			{
				// criteria, there should be only one match per i iteration
				if(local_points_(i,0) == msg.markers[j].id)
				{
					measured_points.push_back( Eigen::Vector3d(msg.markers[j].point.x, -msg.markers[j].point.z, msg.markers[j].point.y).transpose() );
					found_points.push_back( Eigen::Vector3d(local_points_(i,1), local_points_(i,2), local_points_(i,3)).transpose() );
					counter++;
				}
			}
		}

		Eigen::MatrixXd startP, finalP;
		startP.resize(counter, 3);
		finalP.resize(counter, 3);

		for(int i = 0; i < counter; ++i)
		{
			startP.row(i) = found_points[i];
			finalP.row(i) = measured_points[i];
		}

		if (filter_.isInitialized() == 0)
		{	
			filter_.initialize(1);
			computeOptimalRigidTransformation(startP, finalP);
			filter_.setKalman_x(transfParameters_);
		}		
		else
		{	
			filter_.prediction(startP);
			transfParameters_ = filter_.update(finalP);
		}

		// fill and publish optimal pose
		tf::Transform pose;
		pose.setOrigin( tf::Vector3(transfParameters_(0),transfParameters_(1),transfParameters_(2)));
		pose.setRotation( tf::Quaternion(transfParameters_(3),transfParameters_(4),transfParameters_(5),transfParameters_(6)));
		tf_broadcaster_.sendTransform(tf::StampedTransform(pose, ros::Time::now(), reference_frame_.c_str(), object_frame_.c_str()));

		geometry_msgs::TransformStamped msg_t;

		//------------------------------------------
		/*Panagiotis Mavridis 24/04/2018
		  PUTTING MONOTONIC CLOCK TIMESTAMP IN THE MESSAGE
		  IN ORDER TO CALCULATE COMMUNCATION LATENCY 
		  BETWEEN THE ROS NODES*/

		//ONLY FOR POS AND VEL FOR NOW  

		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
		msg_t.header.stamp.sec = ts.tv_sec;
		msg_t.header.stamp.nsec = ts.tv_nsec;
		//--------------------------------------------------

		//---Reaplce above with this if needed
		//msg_t.header.stamp = ros::Time::now();

		msg_t.header.frame_id = reference_frame_;
		msg_t.child_frame_id = object_frame_;
		tf::transformTFToMsg(pose,msg_t.transform);
		pub_transform_.publish(msg_t);


	}


	// computes the optimal rigid body transformation given a set of points
	void PoseTracker::computeOptimalRigidTransformation(Eigen::MatrixXd startP, Eigen::MatrixXd finalP)
	{	
		Eigen::Matrix4d transf;

		if (startP.rows()!=finalP.rows())
		{	
			ROS_ERROR("We need that the rows be the same at the beggining");
			exit(1);
		}

		Eigen::RowVector3d centroid_startP = Eigen::RowVector3d::Zero(); 
		Eigen::RowVector3d centroid_finalP = Eigen::RowVector3d::Zero(); //= mean(B);
		Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

		//calculate the mean
		for (int i=0;i<startP.rows();i++)
		{	
			centroid_startP = centroid_startP+startP.row(i);
			centroid_finalP = centroid_finalP+finalP.row(i);
		}

		centroid_startP = centroid_startP/startP.rows();
		centroid_finalP = centroid_finalP/startP.rows();

		for (int i=0;i<startP.rows();i++)
			H = H + (startP.row(i)-centroid_startP).transpose()*(finalP.row(i)-centroid_finalP);

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

		Eigen::MatrixXd U = svd.matrixU();
		Eigen::MatrixXd V = svd.matrixV();

		if (V.determinant()<0)
			V.col(2)=-V.col(2)*(-1);

		Eigen::MatrixXd R = V*U.transpose();

		Eigen::Matrix4d C_A = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d C_B = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d R_new = Eigen::Matrix4d::Identity();

		C_A.block<3,1>(0,3) = -centroid_startP.transpose();
		R_new.block<3,3>(0,0) = R;

		C_B.block<3,1>(0,3) = centroid_finalP.transpose();


		transf = C_B * R_new * C_A;

		Eigen::Quaterniond mat_rot(transf.block<3,3>(0,0));

		Eigen::Vector3d trasl = transf.block<3,1>(0,3).transpose();

		transfParameters_ << trasl(0), trasl(1), trasl(2), mat_rot.x(), mat_rot.y(), mat_rot.z(), mat_rot.w();

	}

} // namespace phase_space

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PoseTracker");

	ros::NodeHandle nh;

	//--------Panagiotis Mavridis 25/04/2018----------------
	//----Create file with Phase Space message latencies in order to plot---
	/*struct sched_param schedParam;
	  schedParam.sched_priority = RT_PRIORITY;
	  int sched_policy = SCHED_RR;
	  sched_setscheduler(0, sched_policy, &schedParam);
	 */

	//latency_fp = fopen("/home/mrrobot/Desktop/phase_space-to-track_body_node-latencies.txt","w");

	//------------------------------------------------------ 


	phase_space::PoseTracker node(nh,argc,argv);

	ros::spin();
	//ros::AsyncSpinner spinner(1); // Use 1 thread
	//spinner.start();

	//ros::MultiThreadedSpinner spinner(2); // Use 2 threads
	//spinner.spin(); 

	//-------Panagiotis Mavridis-----------------
	//fclose(latency_fp);	
	//------------------------------------------

	return 0;
}
