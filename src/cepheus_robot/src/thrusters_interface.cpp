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
unsigned char minor_number;

double duty[8];
uint16_t dir[4];
int freq; //hz
uint32_t period;
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


void controllerCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{   
    if (cmd->linear.x >= 0.05)
    {
        duty[0] = cmd->linear.x;
        dir[0] = 1;

        duty[2] = cmd->linear.x;
        dir[2] = 1;
    }
    else if (cmd->linear.x <=-0.05) 
    {
        duty[0] = 1 + cmd->linear.x;
        dir[0] = 2;

        duty[2] = 1 + cmd->linear.x;
        dir[2] = 2;
    }
    else
    {
        duty[0] = 0.0;
        duty[2] = 0.0;
        dir[0] = 0;
        dir[2] = 0;
    }

    if (cmd->angular.z >= 0.05)
    {
        duty[1] = cmd->angular.z;
        dir[1] = 1;
    }
    else if (cmd->angular.z <=-0.05)
    {
        duty[1] = 1 + cmd->angular.z;
        dir[1] = 2;
    }
    else 
    {
        duty[1] = 0.0;
        dir[1] = 0;
    }

    //Seting (pins for Port 2)
    uint16_t output = 0xFF | dir[0]<<8 | dir[1]<<10 | dir[2]<<12 | dir[3]<<14;

    dm7820_status = DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_2, 0);
    DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Output()");

    //Set port's 2 bits to PWM output pulse width modulator peripheral
    dm7820_status = DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, output, DM7820_STDIO_PERIPH_PWM);
    DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

    //Reset port's 2 bits to STD IO bits in order not to act like an PWM
    dm7820_status = DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, ~output, DM7820_STDIO_PERIPH_FIFO_0);
    DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");


    dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_A, (uint32_t)(duty[0]*period) );
    DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
    dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_B, (uint32_t)(duty[1]*period) );
    DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
    dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_C, (uint32_t)(duty[2]*period) );
    DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
    dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_D, (uint32_t)(duty[3]*period) );
    DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

    t12.header.stamp = ros::Time::now();
    t34.header.stamp = ros::Time::now();
    t56.header.stamp = ros::Time::now();

    t12.wrench.force.x = (duty[0])*THRUST;
    t34.wrench.force.x = (duty[1])*THRUST;
    t56.wrench.force.x = (duty[2])*THRUST;

    pub12.publish(t12);
    pub34.publish(t34);
    pub56.publish(t56);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "thruster_interface", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);
    ros::NodeHandle n;
    ros::Rate loop_rate(1000);

    int freq, minor_number_param;
    ros::param::param<int>("~IO_card_minor_num", minor_number_param, 0);
    ros::param::param<int>("~frequency", freq, 10);
	

	//Device initialization
    minor_number = (unsigned char)minor_number_param;
	ROS_INFO("Opening device with minor number %u ...\n", minor_number);
	dm7820_status = DM7820_General_Open_Board(minor_number, &board);
	DM7820_Return_Status(dm7820_status, "Can't open thruster board");


    /******************************************************************
                Programmable clock 0 initialization
    *******************************************************************/
    //Disable PRG clock 0 
    dm7820_status =
        DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0,
                   DM7820_PRGCLK_MODE_DISABLED);
    DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

    //maybe output init

    //Set master clock to 25 MHz clock
    dm7820_status =
        DM7820_PrgClk_Set_Master(board, DM7820_PRGCLK_CLOCK_0,
                     DM7820_PRGCLK_MASTER_25_MHZ);
    DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Master()");

    //Set clock start trigger to start immediately
    dm7820_status =
        DM7820_PrgClk_Set_Start_Trigger(board, DM7820_PRGCLK_CLOCK_0,
                        DM7820_PRGCLK_START_IMMEDIATE);
    DM7820_Return_Status(dm7820_status,
                 "DM7820_PrgClk_Set_Start_Trigger()");

    //Set clock stop trigger so that clock is never stopped
    dm7820_status =
        DM7820_PrgClk_Set_Stop_Trigger(board, DM7820_PRGCLK_CLOCK_0,
                       DM7820_PRGCLK_STOP_NONE);
    DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Stop_Trigger()");

    //Set clock period to obtain 50000 Hz frequency
    dm7820_status =
        DM7820_PrgClk_Set_Period(board, DM7820_PRGCLK_CLOCK_0, 500);
    DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");

    //Put clock into continuous mode and enable it
    dm7820_status =
        DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0,
                   DM7820_PRGCLK_MODE_CONTINUOUS);
    DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");


    /******************************************************************
                              PWM Init
    *******************************************************************/
    //Disable all pulse width modulators to put them into a known state; any
    //pulse width modulator should be disabled before programming it
    dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_1, 0x00);
    DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");

    // Set port's 2 // bits to peripheral output
    dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, 0xFFFF, DM7820_STDIO_MODE_PER_OUT);
    DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

    //Set port's 2 // bits to PWM output pulse width modulator peripheral
    // dm7820_status = DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, 0xFFFF, DM7820_STDIO_PERIPH_PWM);
    // DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

    //Set period master clock to programmable clock 0 with freq 500 hz
    dm7820_status = DM7820_PWM_Set_Period_Master(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_PERIOD_MASTER_PROG_CLOCK_0);
    DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period_Master()");

    // Set pulse width modulator period to obtain frequency 500/freq Hz
    period = (uint32_t)(50000/freq);
    dm7820_status = DM7820_PWM_Set_Period(board, DM7820_PWM_MODULATOR_1, period);
    DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");

    //Set width master clock to 25 MHz clock
    dm7820_status = DM7820_PWM_Set_Width_Master(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_WIDTH_MASTER_PROG_CLOCK_0);
    DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width_Master()");

    //zero out all pwms
    dm7820_status = DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_2, 0);
    DM7820_Return_Status(dm7820_status, "DM7820_StdIO_StdIO_Set_Output()");

    dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_A, 0);
    DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
    dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_B, 0);
    DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
    dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_C, 0);
    DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
    dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_D, 0);
    DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

    //enable PWM 1
    dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_1, 0xFF);
    DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");


    /******************************************************************
                              ROS Init
    *******************************************************************/
    t12.header.frame_id = "T1";
    t34.header.frame_id = "T2";
    t56.header.frame_id = "T3";

    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, controllerCallback);
    pub12 = n.advertise<geometry_msgs::WrenchStamped>("thruster12", 1000);
    pub34 = n.advertise<geometry_msgs::WrenchStamped>("thruster34", 1000);
    pub56 = n.advertise<geometry_msgs::WrenchStamped>("thruster56", 1000);

    // // Set thread's scheduling to realtime
    // struct sched_param param;
    // param.sched_priority = 98;
    // if (pthread_setschedparam(pthread_self(), SCHED_RR, &param) != 0) {
    //     ROS_ERROR("Couldn't set thruster control to real-time");
    // }

    duty[0] = 0;
    duty[1] = 0;
    duty[2] = 0;
    duty[3] = 0;
    duty[4] = 0;
    duty[5] = 0;
    duty[6] = 0;
    duty[7] = 0;


    while (!g_request_shutdown)
    {
    // Do non-callback stuff
        ros::spinOnce();
        loop_rate.sleep();
    }


    // Do pre-shutdown tasks
    dm7820_status = DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, 0b0011111100000000, DM7820_STDIO_PERIPH_FIFO_0);
    DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

    dm7820_status = DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_2, 0);
    DM7820_Return_Status(dm7820_status, "DM7820_StdIO_StdIO_Set_Output()");
    ROS_WARN("\n\tThrusters OFF");
    
    ros::shutdown();
   
    return 0;
}
