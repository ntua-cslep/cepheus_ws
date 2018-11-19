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

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
// Replacement SIGINT handler
void ctrl_C_Handler(int sig)
{
        g_request_shutdown = 1;
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

}

