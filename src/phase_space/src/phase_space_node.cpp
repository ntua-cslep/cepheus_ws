
// standard headers
#include <stdio.h>
#include <vector>

// ROS headers
#include <ros/ros.h>
//#include <ros/message_operations.h>
#include <geometry_msgs/PointStamped.h>

// local headers
#include "owl.h"
#include "phase_space/PhaseSpaceMarkerArray.h"

//--Panagiotis Mavridis
//--For monotonic clock
#include <time.h>
#define NANO_TO_MICRO_DIVISOR 1000
FILE *latency_fp;
#include <std_msgs/Time.h>
#include <sched.h>
#define RT_PRIORITY 95

//-----------------------

namespace phase_space {

	class PhaseSpaceClient
	{
		public:

			// the node handle
			ros::NodeHandle nh_;

			// node handle in the private namespace
			ros::NodeHandle priv_nh_;

			// phase space related members
			int marker_count_;
			int tracker_;
			int init_flags_;
			std::string server_ip_;
			OWLMarker* markers_;

			// publishers
			ros::Publisher pub_phase_space_markers_;

			// the marker coordinates (visualization is handled by another node)
			phase_space::PhaseSpaceMarkerArray phase_space_markers_;

		public:

			//----Panagiotis Mavridis 03/05/2018-------

			void grabTime(std_msgs::Time&);
			//-------------------------------------------

			// callback functions
			void publishMarkers();

			// constructor
			PhaseSpaceClient(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
		{

			// initialize the phase space client
			priv_nh_.param<std::string>("ip", server_ip_, "192.168.1.230");
			char* ip = new char[server_ip_.size() + 1];
			std::copy(server_ip_.begin(), server_ip_.end(), ip);
			ip[server_ip_.size()] = '\0'; // don't forget the terminating 0

			priv_nh_.param<int>("init_flags", init_flags_, 0);

			priv_nh_.param<int>("marker_count", marker_count_, 72);

			markers_ = new OWLMarker[marker_count_];

			if(owlInit(ip, init_flags_) < 0)
			{
				ROS_ERROR("error in owl initialization");
			}


			// create tracker 0
			tracker_ = 0;
			owlTrackeri(tracker_, OWL_CREATE, OWL_POINT_TRACKER);

			// set markers
			for(int i = 0; i < marker_count_; i++)
				owlMarkeri(MARKER(tracker_, i), OWL_SET_LED, i);

			// activate tracker
			owlTracker(tracker_, OWL_ENABLE);

			//owlSetInteger(OWL_COMMDATA, OWL_ENABLE) ;

			// flush requests and check for errors
			if(!owlGetStatus())
			{
				ROS_ERROR("error in point tracker setup %i", owlGetError());
			}

			// set default frequency
			owlSetFloat(OWL_FREQUENCY, OWL_MAX_FREQUENCY);

			// start streaming
			owlSetInteger(OWL_STREAMING, OWL_ENABLE);

			//----Panagiotis Mavridis 03/05/2018-------

			// Enable Timestamps
			//owlSetInteger(OWL_TIMESTAMP, OWL_ENABLE) ;
			//----------------------------------------


			// advertise topics
			pub_phase_space_markers_ = nh.advertise<phase_space::PhaseSpaceMarkerArray>(nh.resolveName("/phase_space_markers"), 10);

			//ros::Rate loop_rate(10);
		}

			//! Empty stub
			~PhaseSpaceClient() {delete[] markers_;}
			//~PhaseSpaceClient() {}

	};


	//--------Panagiotis Mavridis 03/05/2018-------
	// from https://sourceforge.net/p/personalrobots/code/HEAD/tree/pkg/trunk/drivers/phase_space/phase_s//pace_node.cpp#l83

	void PhaseSpaceClient::grabTime(std_msgs::Time& time)
	{
		int timeVal[3] ;
		timeVal[0] = 1 ;
		timeVal[1] = 2 ;
		timeVal[2] = 3 ;
		int ntime = owlGetIntegerv(OWL_COMMDATA, timeVal) ;

		if (ntime < 0)
			printf("ERROR: ntime=%i\n", ntime) ;

		time.data = ros::Time((unsigned int) timeVal[1], (unsigned int) timeVal[2]) ;
		long latency = ((time.data.toSec() * 1000000000) + time.data.nsec)/NANO_TO_MICRO_DIVISOR;
		fprintf(latency_fp,"%d %d %d\n",timeVal[0],timeVal[1],timeVal[2]);
	}


	//--------------------------------------------




	// this function is called as fast as ROS can from the main loop directly
	void PhaseSpaceClient::publishMarkers()
	{
		int err;
		int pos=0;
		std_msgs::Time time;

		// get some markers
		int n = owlGetMarkers(markers_, marker_count_);

		// check for error
		if((err = owlGetError()) != OWL_NO_ERROR)
		{    ROS_INFO("error %d", err);
			return;
		}

		//---------Panagiotis Mavridis 2/05/2018------------


		//---Calculate the 'read' frequency of the node-----

		/*static bool first_time = true;
		  static struct timespec ts;

		  if(first_time){
		  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
		  first_time = false;
		  }
		  else{
		  struct timespec ts_cycle;
		  clock_gettime(CLOCK_MONOTONIC_RAW, &ts_cycle);
		  long latency = ((ts_cycle.tv_sec - ts.tv_sec) * 1000000000 + (ts_cycle.tv_nsec - ts.tv_nsec))/NANO_TO_MICRO_DIVISOR;
		  fprintf(latency_fp,"%ld\n",latency);
		  ts = ts_cycle;
		  }*/	

		//--------------------------------------------------


		// no data yet
		if(n == 0)
			return;


		if(n > 0)
		{
			//---Panagiotis Mavridis--------
			//grabTime(time);	
			//------------------------------

			for (int id = 0; id<n; id++)
			{
				if(markers_[id].cond > 0)
				{   
					ROS_DEBUG("%i %f %f %f", n, 0.001*markers_[id].x, 0.001*markers_[id].y, 0.001*markers_[id].z);

					// phase space gives the coordinates in mm, but in ROS, everything is MKS, so let it be hard-coded.
					// fill only the id and coordinates, the rviz marker is filled in the viz node.
					phase_space::PhaseSpaceMarker marker;
					marker.id = id;
					marker.point.x = 0.001*markers_[id].x;
					marker.point.y = 0.001*markers_[id].y;
					marker.point.z = 0.001*markers_[id].z;
					phase_space_markers_.markers.push_back(marker);

				} 
			}

			//----Panagiotis Mavridis 2/05/2018---------------
			//-In order to calculate phase space path latency
			/*
			struct timespec ts;
			clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
			phase_space_markers_.header.stamp.sec = ts.tv_sec;
			phase_space_markers_.header.stamp.nsec = ts.tv_nsec;
			*/

			//------------------------------------------------

			//---Original Code---replace with above-------
			phase_space_markers_.header.stamp = ros::Time::now();

			phase_space_markers_.header.frame_id = "/phase_space_world";
			pub_phase_space_markers_.publish(phase_space_markers_);

		}

		// empty the vector
		phase_space_markers_.markers.clear();
	}

} // namespace phase_space

int main(int argc, char **argv)
{
	ros::init(argc, argv, "phase_space_node");
	ros::NodeHandle nh;

	//--------Panagiotis Mavridis 25/04/2018----------------
	//----Create file with Phase Space message latencies in order to plot---
	/*struct sched_param schedParam;
	  schedParam.sched_priority = RT_PRIORITY;
	  int sched_policy = SCHED_RR;
	  sched_setscheduler(0, sched_policy, &schedParam);
	 */

	//latency_fp = fopen("/home/mrrobot/Desktop/phase_space-send_marker_latency.txt","w");

	//------------------------------------------------------ 


	phase_space::PhaseSpaceClient node(nh);

	while(ros::ok())
	{
		node.publishMarkers();
		ros::spinOnce();

	}

	//fclose(latency_fp);	

	// cleanup
	owlDone();
	return 0;
}
