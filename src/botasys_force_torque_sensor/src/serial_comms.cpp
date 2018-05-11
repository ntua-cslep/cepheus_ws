/*********************************************************************
* serial_comms.cpp
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

#include <botasys_force_torque_sensor/serial_comms.h>
#include <ros/package.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <chrono>
#include <thread>
#include <functional>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace botasys {

SerialComms::SerialComms(bool calibration) :
  file_descriptor_(-1),
  nh_("~")
{
  nh_.getParam("frame_id", frame_id_);
  // nh_.getParam("wrench_topic_name", topic_name_);
  pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/botasys", 1000);
  nh_.getParam("port", port_);
  while (!this->initializeSerial(port_) && ros::ok()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  if(!calibration) {
    if(!this->getOffset()) {
      ROS_ERROR("Offset not calibrated, the values you are reading may be wrong");
    }
    if(!this->loadCalibration()) {
      ROS_ERROR("Calibration data not found, contact Botasys");
    }
  }
}

SerialComms::~SerialComms()
{
  if(file_descriptor_ != -1) {
    ROS_INFO("Closing Serial Communication");
    close(file_descriptor_);
  }
}

void SerialComms::measure()
{
  while(ros::ok() && file_descriptor_ != -1) {
    SerialComms::Measurements_t<int> value = this->receive() - offset_;
    Eigen::Matrix<int,6,1> measurements = value.getVector();
    Eigen::Matrix<double,6,1> actual_values = calibration_ * measurements.cast<double>();
    pub_.publish(this->getWrench(actual_values));
  }
}

bool SerialComms::initializeSerial(const std::string &serial_port)
{
  ROS_INFO("Initializing Serial Communication");
  int flags;
  termios newtio;
  ROS_INFO("Openning serial port %s", serial_port.c_str());
  // Open the Serial Port
  file_descriptor_ = open(serial_port.c_str(), O_RDONLY | O_NOCTTY);
  if (file_descriptor_ < 0) {
    ROS_ERROR("Failed to open serial port %s", serial_port.c_str());
    return false;
  }

  // Get the Serial Descriptor Flags
  if (tcgetattr(file_descriptor_, &newtio) < 0) {
    ROS_ERROR("Failed to get connection attributes");
    return false;
  }

  cfmakeraw(&newtio);
  // Set the Serial Speed
  cfsetispeed(&newtio, B115200);
  cfsetospeed(&newtio, B115200);
  // Set the Serial Connection Attributes
  if(tcsetattr(file_descriptor_, TCSAFLUSH, &newtio) < 0) {
    ROS_ERROR("Failed to set connection attributes");
    return false;
  }
  // Flush the input and output streams
  if(tcflush(file_descriptor_, TCIOFLUSH) < 0) {
    ROS_ERROR("Failed to flush the input and output streams");
    return false;
  }
  // Get the Serial Descriptor Flags
  if(fcntl(file_descriptor_, F_GETFL) < 0) {
    ROS_ERROR("Failed to set the descriptor flags");
    return false;
  }
  return true;
}

bool SerialComms::getOffset()
{
  if(!nh_.getParam("offset/Fx", offset_.Fx))
    return false;
  if(!nh_.getParam("offset/Fy", offset_.Fy))
    return false;
  if(!nh_.getParam("offset/Fz", offset_.Fz))
    return false;
  if(!nh_.getParam("offset/Mx", offset_.Mx))
    return false;
  if(!nh_.getParam("offset/My", offset_.My))
    return false;
  if(!nh_.getParam("offset/Mz", offset_.Mz))
    return false;
  return true;
}

unsigned char SerialComms::readByte()
{
  unsigned char byte = 0;
  bool flag = false;
  while(read(file_descriptor_, &byte, 1) < 0) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(10));
  }
  return byte;
}

SerialComms::Measurements_t<int> SerialComms::fillMeasurements(std::vector<int> values)
{
  Measurements_t<int> measure;
  // Order of data might be wrong
  measure.Fx = values[0];
  measure.Fy = values[1];
  measure.Fz = values[2];
  measure.Mx = values[3];
  measure.My = values[4];
  measure.Mz = values[5];
  return measure;
}

SerialComms::Measurements_t<int> SerialComms::receive() {
  while(ros::ok()) {
    if(this->readByte() == 0x41) {
      std::vector<int> values;
      for(size_t i = 0; i < 6; i++) {
        try {
          this->readByte(); // 0x0A
          this->readByte(); // 0x0D
          unsigned char buffer[6];
          memset(buffer, 0, sizeof(buffer));
          for (size_t j = 0; j < 5; j++)
            buffer[j] = this->readByte();
          buffer[6] = '\0';
          std::string str = reinterpret_cast<const char*>(buffer);
          int value = std::stoi(str);
          values.push_back(value);
        }
        catch (std::exception &e) {
          ROS_ERROR("Error while reading a packet, %s", e.what());
          close(file_descriptor_);
          while (!this->initializeSerial(port_)) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
          }
        }
      }
      return (this->fillMeasurements(values));
    }
  }
}

SerialComms::Measurements_t<int> SerialComms::getAverage(const std::vector<Measurements_t<int> >& measurements)
{
  SerialComms::Measurements_t<int> result;
  for(size_t i = 0; i < measurements.size(); i++) {
    result.Fx += measurements[i].Fx;
    result.Fy += measurements[i].Fy;
    result.Fz += measurements[i].Fz;
    result.Mx += measurements[i].Mx;
    result.My += measurements[i].My;
    result.Mz += measurements[i].Mz;
  }
  result.Fx = result.Fx / measurements.size();
  result.Fy = result.Fy / measurements.size();
  result.Fz = result.Fz / measurements.size();
  result.Mx = result.Mx / measurements.size();
  result.My = result.My / measurements.size();
  result.Mz = result.Mz / measurements.size();
  return result;
}

void SerialComms::writeToYaml(Measurements_t<int> measurements)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "port";
  out << YAML::Value << port_;
  out << YAML::Key << "offset";
  out << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "Fx" << YAML::Value << measurements.Fx;
  out << YAML::Key << "Fy" << YAML::Value << measurements.Fy;
  out << YAML::Key << "Fz" << YAML::Value << measurements.Fz;
  out << YAML::Key << "Mx" << YAML::Value << measurements.Mx;
  out << YAML::Key << "My" << YAML::Value << measurements.My;
  out << YAML::Key << "Mz" << YAML::Value << measurements.Mz;
  out << YAML::EndMap;
  out << YAML::EndMap;
  std::string package_path = ros::package::getPath("botasys_force_torque_sensor");
  package_path.append("/config/offset.yaml");
  std::ofstream fout(package_path);
  fout << out.c_str();
}

bool SerialComms::loadCalibration()
{
  std::string package_path = ros::package::getPath("botasys_force_torque_sensor");
  package_path.append("/config/calibration.txt");
  try {
    std::ifstream fin(package_path);
    for(size_t i = 0; i < 6; i++) {
     for(size_t j = 0; j < 6; j++) {
       if(fin.peek() != EOF) {
         fin >> calibration_(i,j);
       }
     }
    }
  } catch (std::exception &e) {
    ROS_ERROR("%s", e.what());
    return false;
  }
  return true;
}

geometry_msgs::WrenchStamped SerialComms::getWrench(const Eigen::Matrix<double,6,1>& values)
{
  geometry_msgs::WrenchStamped msg;
  msg.header.frame_id = frame_id_; //"botasys";
  msg.header.stamp = ros::Time::now();
  msg.wrench.force.x = values(0);
  msg.wrench.force.y = values(1);
  msg.wrench.force.z = values(2);
  msg.wrench.torque.x = values(3);
  msg.wrench.torque.y = values(4);
  msg.wrench.torque.z = values(5);
  return msg;
}

void SerialComms::offset(unsigned int values) {
  std::vector<Measurements_t<int>> measurements;
  for (unsigned int i = 0; i < values; i++) {
    measurements.push_back(this->receive());
  }
  Measurements_t<int> average = this->getAverage(measurements);
  this->writeToYaml(average);
}

}

