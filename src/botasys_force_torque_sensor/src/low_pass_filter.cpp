/*********************************************************************
* low_pass_filter.cpp
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

#include <deque>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

ros::Publisher pub;
std::deque<geometry_msgs::WrenchStamped> measurements;
double threshold = 2.0;

void callback(const geometry_msgs::WrenchStamped &msg) {
  measurements.push_front(msg);
  if(measurements.size() > 25) {
    measurements.pop_back();
  }
  double Fx = 0, Fy = 0, Fz = 0, Mx = 0, My = 0, Mz = 0;
  for (size_t i = 0; i < measurements.size(); i++) {
    Fx += measurements[i].wrench.force.x;
    Fy += measurements[i].wrench.force.y;
    Fz += measurements[i].wrench.force.z;
    Mx += measurements[i].wrench.torque.x;
    My += measurements[i].wrench.torque.y;
    Mz += measurements[i].wrench.torque.z;
  }
  Fx = Fx / measurements.size();
  Fy = Fy / measurements.size();
  Fz = Fz / measurements.size();
  Mx = Mx / measurements.size();
  My = My / measurements.size();
  Mz = Mz / measurements.size();
  if (Fx < threshold && Fx > -threshold)
    Fx = 0.0;
  if (Fy < threshold && Fy > -threshold)
    Fy = 0.0;
  if (Fz < threshold && Fz > -threshold)
    Fz = 0.0;
  if (Mx < threshold && Mx > -threshold)
    Mx = 0.0;
  if (My < threshold && My > -threshold)
    My = 0.0;
  if (Mz < threshold && Mz > -threshold)
    Mz = 0.0;
  geometry_msgs::WrenchStamped new_msg;
  new_msg.header = msg.header;
  new_msg.wrench.force.x = Fx;
  new_msg.wrench.force.y = Fy;
  new_msg.wrench.force.z = Fz;
  new_msg.wrench.torque.x = Mx;
  new_msg.wrench.torque.y = My;
  new_msg.wrench.torque.z = Mz;
  pub.publish(new_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "botasys_low_pass_filter");
  ros::NodeHandle nh;
  pub = nh.advertise<geometry_msgs::WrenchStamped>("/filtered_botasys", 1000);
  ros::Subscriber sub = nh.subscribe("/botasys", 100, &callback);
  ros::spin();
  return 0;
}
