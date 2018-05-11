// standard headers
#include <fstream>
#include <iostream>
#include <vector>

// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// eigen main header
#include <Eigen/Eigen>

using namespace std;

class PoseTrackingFilter
{
  bool first_;
  Eigen::VectorXd kalman_x; // The parameters calculated by the Kalman filter - part of the 7x1 state vector
  Eigen::MatrixXd kalman_V;// Process noise variance for the parameters
  // Eigen::MatrixXd kalman_W;// Measurement noise variance - this is actually the variance of the measurement noise
  Eigen::MatrixXd kalman_P; // Error covariance matrix - This is a 7x7 matrix
  // Eigen::MatrixXd kalman_K;// Kalman gain - This is a 7x3n vector
  // Eigen::VectorXd kalman_y;; // parameter difference
  // Eigen::MatrixXd kalman_S; // Estimate error
  Eigen::MatrixXd kalman_H;  //derivate of predicted points
  Eigen::VectorXd kalman_h;  //predicted points

public:

  PoseTrackingFilter();
  //Eigen::VectorXd& inizialize();
  //Eigen::VectorXd& OptimalRigidTransformation(Eigen::MatrixXd, Eigen::MatrixXd);
  void prediction(Eigen::MatrixXd);  //kalman_H kalman_h kalman_P   //start point 
  Eigen::VectorXd update(Eigen::MatrixXd);

  void setKalman_x(Eigen::VectorXd v){kalman_x = v;}
  bool isInitialized(){return first_;}
  void initialize(bool f){first_ = f;}
};