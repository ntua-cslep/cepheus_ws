/*
 * Ros node that subscribe 3 mouse topics of type sensor_msgs/joy
 * and post odometry data in a topic mouse/odom of type 
 * nav_msgs/odometry 
 */

#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <math.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#define FR 0
#define RH 1
#define LF 2

/******************************************************************/

geometry_msgs::Vector3Stamped move[3], speed[3];
double dt[3];
ros::Time prev_stamp[3];
int ready[3];
int ready_cnt;

double x_coef, y_coef;

/********************CALLBACK FUNCTIONS***************************/

void frontMouseDataCallback(const sensor_msgs::Joy::ConstPtr& mouse)
{ 	
	int i = FR;
	//movement vector 
	move[i].header.frame_id = mouse->header.frame_id;
	move[i].header.stamp = mouse->header.stamp;
	move[i].vector.x += mouse->axes[0];
	move[i].vector.y += mouse->axes[1];
	move[i].vector.z = 0.0;
	//velocity vector
	speed[i].header.frame_id = mouse->header.frame_id;
	speed[i].header.stamp = mouse->header.stamp;

	dt[i] = ( move[i].header.stamp - prev_stamp[i] ).toSec();
	speed[i].vector.x = move[i].vector.x / dt[i];
	speed[i].vector.y = move[i].vector.y / dt[i];
	speed[i].vector.z = 0.0;
	
	//ROS_INFO("Front x:%f y:%f t:%d frame:", move[FR].vector.x, move[FR].vector.y, prev_stamp[FR].nsec);
	//ROS_INFO_STREAM(mouse->header.frame_id);
	ready[i] = 1;
	ready_cnt += 1;
}

void rightMouseDataCallback(const sensor_msgs::Joy::ConstPtr& mouse)
{ 		
	int i = RH;
	//movement vector 
	move[i].header.frame_id = mouse->header.frame_id;
	move[i].header.stamp = mouse->header.stamp;
	move[i].vector.x += mouse->axes[0];
	move[i].vector.y += mouse->axes[1];
	move[i].vector.z = 0.0;
	//velocity vector
	speed[i].header.frame_id = mouse->header.frame_id;
	speed[i].header.stamp = mouse->header.stamp;

	dt[i] = ( move[i].header.stamp - prev_stamp[i] ).toSec();
	speed[i].vector.x = move[i].vector.x / dt[i];
	speed[i].vector.y = move[i].vector.y / dt[i];
	speed[i].vector.z = 0.0;
	
	//ROS_INFO("Front x:%f y:%f t:%d frame:", move[FR].vector.x, move[FR].vector.y, prev_stamp[FR].nsec);
	//ROS_INFO_STREAM(mouse->header.frame_id);
	ready[i] = 1;
	ready_cnt += 1;
}

void leftMouseDataCallback(const sensor_msgs::Joy::ConstPtr& mouse)
{ 		
	int i = LF;
	//movement vector 
	move[i].header.frame_id = mouse->header.frame_id;
	move[i].header.stamp = mouse->header.stamp;
	move[i].vector.x += mouse->axes[0];
	move[i].vector.y += mouse->axes[1];
	move[i].vector.z = 0.0;
	//velocity vector
	speed[i].header.frame_id = mouse->header.frame_id;
	speed[i].header.stamp = mouse->header.stamp;

	dt[i] = ( move[i].header.stamp - prev_stamp[i] ).toSec();
	speed[i].vector.x = move[i].vector.x / dt[i];
	speed[i].vector.y = move[i].vector.y / dt[i];
	speed[i].vector.z = 0.0;
	
	//ROS_INFO("Front x:%f y:%f t:%d frame:", move[FR].vector.x, move[FR].vector.y, prev_stamp[FR].nsec);
	//ROS_INFO_STREAM(mouse->header.frame_id);
	ready[i] = 1;
	ready_cnt += 1;
}


/******************************************************************/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_node");
	ros::Time::init();
	ros::Rate loop_rate(1000); //limited by the ready flag check
	ros::NodeHandle n; 

	static tf2_ros::TransformBroadcaster map_broadcaster;
	geometry_msgs::TransformStamped map_trans;
    map_trans.header.frame_id = "map";
    map_trans.child_frame_id = "odom";
    map_trans.header.stamp = ros::Time::now();
	map_trans.transform.translation.x = 0;
	map_trans.transform.translation.y = 0;
	map_trans.transform.translation.z = 0;
	map_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

	static tf2_ros::TransformBroadcaster odom_broadcaster;
	geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.header.stamp = ros::Time::now();
	odom_trans.transform.translation.x = 0;
	odom_trans.transform.translation.y = 0;
	odom_trans.transform.translation.z = 0;
	odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

	ros::Time start_time = ros::Time::now();
	//mouse positioning
	tf::TransformListener trans_listener[3];
	tf::StampedTransform trans[3];
	


	ros::Duration(2.0).sleep();//delay 2sec so the tf hade already published tranformations
	try {
      trans_listener[FR].lookupTransform("base_link", "front_mouse", ros::Time(0), trans[FR]);
      trans_listener[LF].lookupTransform("base_link",  "left_mouse", ros::Time(0), trans[LF]);
      trans_listener[RH].lookupTransform("base_link", "right_mouse", ros::Time(0), trans[RH]);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    //enable callback after transformation has readed
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);;
	ros::Subscriber f_sub = n.subscribe("front", 1000, frontMouseDataCallback);
	ros::Subscriber r_sub = n.subscribe("right", 1000, rightMouseDataCallback);
	ros::Subscriber l_sub = n.subscribe("left" , 1000, leftMouseDataCallback);

	nav_msgs::Odometry odom;

    x_coef = 0.00025;//meters per mouse increment
    y_coef = 0.00025;
/*****************************************************************/
	while(ros::ok())
	{
		

		if (ready_cnt >= 1)
		{
		//calculate movement
			//translation averaging and clicks to mm 
			//geometry_msgs::Vector3Stamped base[3];
			//base[FR].header.frame_id = "base_link";
			//tf::Transformer::transformVector("base_link", move[FR], base[FR]);
			//ROS_INFO("x:%f, y:%f", base[FR].vector.x, base[FR].vector.y);
			//double x = trans[FR].getOrigin().x;
			//ROS_INFO( "x:%f", x );

			float Dx = x_coef* (move[FR].vector.x +  move[RH].vector.x  +  move[LF].vector.x )/ 3;
			float Dy = y_coef* (move[FR].vector.y +  move[RH].vector.y  +  move[LF].vector.y )/ 3;
			float Du = x_coef*(speed[FR].vector.x + speed[RH].vector.x  + speed[LF].vector.x )/ 3;
			float Dv = y_coef*(speed[FR].vector.y + speed[RH].vector.y  + speed[LF].vector.y )/ 3;
			//rotation averaging and clicks to rad
			
			ready_cnt = 0;

			for (int n=0; n<=2; n++)
			{
				if (ready[n] == 1)//interrupt runed we have procces the values above calculating Dx so can clear them
				{
					prev_stamp[n] = move[n].header.stamp;			
					move[n].vector.x = 0.0;
					move[n].vector.y = 0.0;
					move[n].vector.z = 0.0;
					//speed consideration
					ready[n]=0;
					ROS_INFO("mouse:%d proccessed", n);
				}
			}

		//update odometry topic
			odom.header.stamp = ros::Time::now();//time_stamp[0];
  	     	//set the position
			odom.pose.pose.position.x += Dx;
			odom.pose.pose.position.y += Dy;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
			odom.header.frame_id = "odom";
		    //set the velocity
		    odom.child_frame_id = "base_link";
		    odom.twist.twist.linear.x = Du;
		    odom.twist.twist.linear.y = Dy;
		    odom.twist.twist.angular.z = 0.0;
			//odom.pose.pose.orientation
			odom_pub.publish(odom);

		//update transform
	        odom_trans.header.stamp = ros::Time::now();//time_stamp[0];
	        odom_trans.transform.translation.x += Dx;//+=
	        odom_trans.transform.translation.y += Dy;
	        odom_trans.transform.translation.z = 0.0;
	        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
	        odom_broadcaster.sendTransform(odom_trans);   

			
		}
		else
		{
			//drift emulation
			map_trans.header.stamp = ros::Time::now();//time_stamp[0];
			map_trans.transform.translation.x = (ros::Time::now()-start_time).toSec() *0.02;
			map_trans.transform.translation.y = (ros::Time::now()-start_time).toSec() *0.01;
			map_trans.transform.translation.z = 0;
			map_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
			map_broadcaster.sendTransform(map_trans); 

			odom_trans.header.stamp = ros::Time::now();
			odom_broadcaster.sendTransform(odom_trans);  
		}


		ros::spinOnce();//runs periodic spinOnce that proccess the  callbacks
		loop_rate.sleep();
	}
	return 0;
}