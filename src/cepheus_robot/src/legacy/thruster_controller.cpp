#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>
#include <sys/time.h>
#include <signal.h>
#include <sys/resource.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pthread.h>
#include <signal.h>

#include "dm7820_library.h"

#define THRUST 0.87

DM7820_Board_Descriptor *board;
DM7820_Error dm7820_status;

uint16_t output_value=0;
uint8_t help_option = 0x00;
uint8_t input_option = 0x00;
uint8_t minor_option = 0x00;
uint8_t output_option = 0x00;
unsigned int minor_number = 0;

int count=1;
uint16_t input_value = 0;
double duty[8];

int freq; //hz
int res; //second
ros::Publisher pub12, pub34, pub56;
geometry_msgs::WrenchStamped t12, t34, t56;



// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}


void timerCallback(const ros::TimerEvent& event)
{
    // dm7820_status = DM7820_StdIO_Get_Input(board, DM7820_STDIO_PORT_2, &input_value);
    // ROS_ERROR("DM7820_StdIO_Get_Input() status %d",dm7820_status);
    // DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Get_Input()");

    for(char i=0; i<8; i++) //WARNING 8 to 16
    {
        if(count < duty[i]*(double)res)
        {
            output_value |= (1<<(i+8));
            dm7820_status = DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_2, output_value);
            DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Output()");
        }
        else 
        {
            output_value &= (~(1<<(i+8)));
            dm7820_status = DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_2, output_value);
            DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Output()");
        }
    }
    count++;
    if(count == res) count = 1;
    // ROS_INFO("tick-tock");
}

void controllerCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{   

    if (cmd->linear.x >= 0.05)
    {
        duty[0] = cmd->linear.x;
        duty[1] = 0.0;
        duty[4] = cmd->linear.x;
        duty[5] = 0.0;
    }
    else if (cmd->linear.x <=-0.05) 
    {
        duty[0] = 0.0;
        duty[1] = -1*(cmd->linear.x);
        duty[4] = 0.0;
        duty[5] = -1*(cmd->linear.x);
    }
    else
    {
        duty[0] = 0.0;
        duty[1] = 0.0;
        duty[4] = 0.0;
        duty[5] = 0.0;
    }

    if (cmd->angular.z >= 0.05)
    {
        duty[2] = 0.0;
        duty[3] = cmd->angular.z;
    }
    else if (cmd->angular.z <=-0.05)
    {
        duty[2] = -1*(cmd->angular.z);
        duty[3] = 0.0;
    }
    else 
    {
        duty[2] = 0.0;
        duty[3] = 0.0;
    }

    t12.header.stamp = ros::Time::now();
    t34.header.stamp = ros::Time::now();
    t56.header.stamp = ros::Time::now();

    t12.wrench.force.x = (duty[0] - duty[1])*THRUST;
    t56.wrench.force.x = (duty[5] - duty[4])*THRUST;
    t34.wrench.force.x = (duty[3] - duty[2])*THRUST;

    pub12.publish(t12);
    pub34.publish(t34);
    pub56.publish(t56);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "thruster_controller", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);
    ros::NodeHandle n;
    ros::Rate loop_rate(1000);
	

	//Device initialization
	ROS_INFO("Opening device with minor number %u ...\n", minor_number);
	dm7820_status = DM7820_General_Open_Board(minor_number, &board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Open_Board()");


	//Output port initialization
	ROS_INFO("Initializing output port %u ...\n", DM7820_STDIO_PORT_2);

	dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, 0xFFFF, DM7820_STDIO_MODE_OUTPUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");


    ros::param::param<int>("~frequency", freq, 10);
    ros::param::param<int>("~resolution", res, 100);

    t12.header.frame_id = "T1";
    t34.header.frame_id = "T2";
    t56.header.frame_id = "T3";

    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, controllerCallback);
    pub12 = n.advertise<geometry_msgs::WrenchStamped>("thruster12", 1000);
    pub34 = n.advertise<geometry_msgs::WrenchStamped>("thruster34", 1000);
    pub56 = n.advertise<geometry_msgs::WrenchStamped>("thruster56", 1000);

    // Set thread's scheduling to realtime
    struct sched_param param;
    param.sched_priority = 98;
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &param) != 0) {
        ROS_ERROR("Couldn't set thruster control to real-time");
    }

    duty[0] = 0;
    duty[1] = 0;
    duty[2] = 0;
    duty[3] = 0;
    duty[4] = 0;
    duty[5] = 0;
    duty[6] = 0;
    duty[7] = 0;


    ros::Timer timer = n.createTimer(ros::Duration(1/(double)(freq*res)), timerCallback);

    while (!g_request_shutdown)
    {
    // Do non-callback stuff
        ros::spinOnce();
        loop_rate.sleep();
    }


  // Do pre-shutdown tasks
    dm7820_status = DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_2, 0);
    DM7820_Return_Status(dm7820_status, "DM7820_StdIO_StdIO_Set_Output()");
    ROS_WARN("\n\tThrusters OFF");
    
    ros::shutdown();
   
    return 0;
}
