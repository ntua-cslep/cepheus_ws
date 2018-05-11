#include "marker_pose_estimator.h"
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

MarkerPoseEstimator mpe;
cv::Mat img, outImg;
cv::Mat R, t;
std::string results;
std::ofstream os;

bool calculated=false;

std::string camera_frame;
std::string target_frame;
geometry_msgs::PoseStamped pose;

ros::Publisher pose_pub;
image_transport::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        img = cv_bridge::toCvShare(msg, "mono8")->image;
        if (img.empty()){
            ROS_ERROR("image is empty");
        }

        int n = mpe.run(img, R, t);

        // Rotation matrix to angle-axis
        cv::Mat rvec;
        cv::Rodrigues(R, rvec);

        // Quaternion
        double a = cv::norm(rvec);
        rvec /= a;
        double qx = rvec.at<double>(0) * std::sin(a / 2.0);
        double qy = rvec.at<double>(1) * std::sin(a / 2.0);
        double qz = rvec.at<double>(2) * std::sin(a / 2.0);
        double qw = std::cos(a / 2.0);

        //ROS_INFO("tx: %f, ty: %f, tz: %f",   t.at<double>(0),t.at<double>(1),t.at<double>(2));
        //ROS_INFO("qx: %f, qy: %f, qz: %f",   qx,qy,qz);

        //the pose of the camera(robot) in the drogue frame
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = target_frame;
        pose.pose.position.x = t.at<double>(0);
        pose.pose.position.y = t.at<double>(1);
        pose.pose.position.z = t.at<double>(2);
        pose.pose.orientation.x = qx;
        pose.pose.orientation.y = qy;
        pose.pose.orientation.z = qz;
        pose.pose.orientation.w = qw;
        pose_pub.publish(pose);
        calculated=true;

        mpe.drawMarkers(img, outImg);
        // cv::putText(outImg, "probe_view", cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1.0, cvScalar(255, 255, 255));
        // cv::imshow("probe_view", outImg);
        // int key = cv::waitKey(60); 

        std_msgs::Header img_header;
        img_header.frame_id = camera_frame;
        img_header.stamp = ros::Time::now();
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(img_header, "mono8", outImg).toImageMsg();
        pub.publish(msg);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "assist_board");
	ros::NodeHandle node;
    ros::Rate loop_rate(100);
    pose_pub = node.advertise<geometry_msgs::PoseStamped>("pose", 1000);

    static tf::TransformBroadcaster drogue_broadcaster;

    //ros parameters
    std::string board, camera, dictionary, image;
    ros::param::param<std::string>("~board", board, ""); //board .yaml file
    ros::param::param<std::string>("~dictionary", dictionary, ""); //dictionary file
    ros::param::param<std::string>("~camera", camera, "");
    ros::param::param<std::string>("~image", image, "");

    float th0, th1, marker_min_size, marker_max_size;
    ros::param::param<float>("~th0", th0, 21);
    ros::param::param<float>("~th1", th1, 7);
    ros::param::param<float>("~marker_min_size", marker_min_size, 0.002);
    ros::param::param<float>("~marker_max_size", marker_max_size, 0.15);

    ros::param::param<std::string>("~camera_frame", camera_frame, "camera");
    ros::param::param<std::string>("~target_frame", target_frame, "drogue");

    mpe.setThresholdParams(th0, th1);
    mpe.setMinMaxSize(marker_min_size, marker_max_size);

    if (board!="") ROS_INFO("board file: %s", board.c_str());
    else ROS_ERROR("No board file path setted in the lauch file");

    if (camera!="") ROS_INFO("camera file: %s", camera.c_str());
    else ROS_ERROR("No camera file path setted in the lauch file");

    if (dictionary!="") ROS_INFO("dictionary file: %s", dictionary.c_str());
    else ROS_ERROR("No camera file path setted in the lauch file");


    if (!mpe.loadBoardParameters(board))
    {
        ROS_ERROR("Couldn't read board parameters");
        return 1;
    }

    if (!mpe.loadDictionary(dictionary))
    {
        ROS_ERROR("Couldn't read dictionary");
        return 1;
    }
    
    if (!mpe.loadCameraParameters(camera))
    {
        ROS_ERROR("Couldn't read camera parameters");
        return 1;
    }

    ROS_INFO("Load everything successfully");

    // cv::namedWindow("probe_view");
    // cv::startWindowThread();
    
    image_transport::ImageTransport it(node);

    ROS_INFO("Subscribe to Image topic: %s", image.c_str());
    image_transport::Subscriber sub = it.subscribe(image, 1, imageCallback);

    pub = it.advertise("/camera/image_result", 1);

    while(ros::ok())
    {
        if(calculated) {
            calculated=false;


            tf::Transform drogue;
            drogue.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
            drogue.setRotation(tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w));

            drogue_broadcaster.sendTransform(tf::StampedTransform(drogue.inverse(), pose.header.stamp, camera_frame, target_frame));
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    // cv::destroyWindow("probe_view");
    return 0;
}

