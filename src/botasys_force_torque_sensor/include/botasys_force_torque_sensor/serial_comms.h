/*********************************************************************
* serial_comms.h
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Botasys
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of University of Patras nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Aris Synodinos
*********************************************************************/
#ifndef SERIAL_COMMS_H
#define SERIAL_COMMS_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/Eigen>

// For Serial Communication
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>

namespace botasys {

class SerialComms {
public:
  SerialComms(bool calibration = false);
  ~SerialComms();
  void measure();
  void offset(unsigned int values = 100);
private:
  template <typename T>
  struct Measurements_t {
    T Fx,Fy,Fz,Mx,My,Mz;
    Eigen::Matrix<T,6,1> getVector() {
      Eigen::Matrix<T,6,1> result;
      result << Fx, Fy, Fz, Mx, My, Mz;
      return result;
    }
    friend Measurements_t<T> operator+(const Measurements_t<T> &c1, const Measurements_t<T> &c2) {
      SerialComms::Measurements_t<T> result;
      result.Fx = c1.Fx+c2.Fx;
      result.Fy = c1.Fy+c2.Fy;
      result.Fz = c1.Fz+c2.Fz;
      result.Mx = c1.Mx+c2.Mx;
      result.My = c1.My+c2.My;
      result.Mz = c1.Mz+c2.Mz;
      return result;
    }
    friend Measurements_t<T> operator-(const Measurements_t<T> &c1, const Measurements_t<T> &c2) {
      SerialComms::Measurements_t<T> result;
      result.Fx = c1.Fx-c2.Fx;
      result.Fy = c1.Fy-c2.Fy;
      result.Fz = c1.Fz-c2.Fz;
      result.Mx = c1.Mx-c2.Mx;
      result.My = c1.My-c2.My;
      result.Mz = c1.Mz-c2.Mz;
      return result;
    }
  };
  bool initializeSerial(const std::string& serial_port);
  bool getOffset();
  unsigned char readByte();
  Measurements_t<int> fillMeasurements(std::vector<int> values);
  Measurements_t<int> receive();
  Measurements_t<int> getAverage(const std::vector<Measurements_t<int>> &measurements);
  void writeToYaml(Measurements_t<int> measurements);
  bool loadCalibration();
  geometry_msgs::WrenchStamped getWrench(const Eigen::Matrix<double,6,1> &values);

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  int file_descriptor_;
  std::string port_;
  std::string topic_name_;
  std::string frame_id_;
  Measurements_t<int> offset_;
  Eigen::Matrix<double,6,6> calibration_;
};

}

#endif

