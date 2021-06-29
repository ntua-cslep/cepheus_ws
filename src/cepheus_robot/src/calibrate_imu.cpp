#include <signal.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>

using namespace std;

double acc_x = 0.0, acc_y = 0.0;
double acc_x_sum = 0.0, acc_y_sum = 0.0;
double angular_vel_z = 0.0;
double angular_vel_z_sum = 0.0;


void imuAccelerationCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    geometry_msgs::Vector3Stamped temp;
    temp = *msg;
    acc_x = temp.vector.x;
    acc_y = temp.vector.y;
}

void imuAngularVelCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    geometry_msgs::Vector3Stamped temp;
    temp = *msg;
    angular_vel_z = temp.vector.z;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "alex_gripper_control_node", ros::init_options::NoSigintHandler);

    ros::NodeHandle nh;
    ros::CallbackQueue main_queue;
    nh.setCallbackQueue(&main_queue);

    ros::Subscriber imu_acc_sub = nh.subscribe("/imu/acceleration", 1, imuAccelerationCallback);
    ros::Subscriber imu_ang_vel_sub = nh.subscribe("/imu/angular_velocity", 1, imuAngularVelCallback);

    double rate, iterations;
    ros::param::param<double>("~loop_rate", rate, 400);
    ros::param::param<double>("~iterations", iterations, 500);
    ros::Rate loop_rate(rate);


    while ( acc_x == 0 || acc_y == 0 || angular_vel_z == 0) {
        ROS_ERROR("No measurements from imu");
        main_queue.callAvailable(ros::WallDuration(1.0));
    }

    int i = 0;
    while (i < iterations) {
        acc_x_sum += acc_x;
        acc_y_sum += acc_y;
        angular_vel_z_sum += angular_vel_z;
        loop_rate.sleep();
        i++;
    }

    cout << acc_x_sum / iterations << endl;
    cout << acc_y_sum / iterations << endl;
    cout << angular_vel_z_sum / iterations << endl;
    // write average to yaml
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "acc_x_offset";
    out << YAML::Value << acc_x_sum / iterations;
    out << YAML::Key << "acc_y_offset";
    out << YAML::Value << acc_y_sum / iterations;
    out << YAML::Key << "angular_vel_z_offset";
    out << YAML::Value << angular_vel_z_sum / iterations;
    out << YAML::Value;
    out << YAML::EndMap;
    string package_path = ros::package::getPath("cepheus_robot");
    package_path.append("/config/imu_offsets.yaml");
    ofstream fout(package_path);
    fout << out.c_str();
    return 0;
}
