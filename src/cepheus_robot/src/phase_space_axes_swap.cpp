/*
    Simple ROS enabled udp server
    publishing  data 2D pose type
    received from CameraServo.exe
    that localize  space robot by
    using MatrixVision Camera
*/
#include <stdio.h> //printf
#include <string.h> //memset
#include <stdlib.h> //exit(0);

#include <stdint.h>
#include <inttypes.h>

#include <sstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

void die(char *s)
{
    perror(s);
    exit(1);
}
 
int main(int argc, char **argv)
{

    /********************************************************************************************
     * ROS setup
     ********************************************************************************************/

    ros::init(argc, argv, "phase_space_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(500); //dont forget to add looprate sleep in the end

    //transforms
    tf::Transform map_to_base;
    tf::Transform map_to_odom;
    tf::StampedTransform base_to_odom;
    tf::StampedTransform map_to_phase;

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;

    /********************************************************************************************
     * THE LOOP
     ********************************************************************************************/

    //wait for the first odom to base transformation up to 5sec
    try{
        ROS_WARN("phase_nodewait for first odom to base_link Transformation");
        listener.waitForTransform("base_link", "odom", ros::Time::now(), ros::Duration(5.0));
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(5.0).sleep();
    }


    while(ros::ok())
    {
        ros::Time data_stamp = ros::Time::now(); //we have to transmitt a time stamp from camera not the delay
        
        // read phasespace
        try{
          listener.lookupTransform("map", "phase_space_5led", data_stamp, map_to_phase);
        }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
        }

        //read odom transformation at the time the foto is taken (past time)
        try{
          listener.lookupTransform("base_link", "odom", ros::Time::now(), base_to_odom);
        }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
        }

        //set position
        map_to_base = map_to_phase;

        map_to_odom.mult(map_to_base, base_to_odom); 

        broadcaster.sendTransform(tf::StampedTransform(map_to_odom, data_stamp, "map", "odom"));

             

        //print details of the client/peer and the data received
        //ROS_INFO("Received packet from %s:%d", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        //ROS_INFO("Receivied data: Rob_ID: %d x:%d, y:%d, th:%d, u:%d, v:%d, w:%d, t:%d\n" , buf[1], x, y, th, u, v, w, t);

        ros::spinOnce();
        //loop_rate.sleep();//consider removing as we have a blocking call in the loop
    }
 
    return 0;
}