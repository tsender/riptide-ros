/*********************************************************************************
 *  Copyright (c) 2017, The Underwater Robotics Team
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#ifndef DEAD_RECKON_1D_H
#define DEAD_RECKON_1D_H

#include "ros/ros.h"
#include "riptide_msgs/Imu.h"
#include "imu_3dm_gx4/FilterOutput.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"
#include "math.h"
#include "stdio.h"
#include "string"
#include "fstream"
//#include <boost/lexical_cast.h>
using namespace std;

class DeadReckon1D
{
private:
  ros::NodeHandle nh;
  ros::Subscriber imu_sub, imu_filter_sub;
  
  FILE *fid;
  int num;
  char file_name_KF[100], file_name[100];
  double tLastKF, tStart, deltaT;
  bool initKF, init;

  int count, minCount;
  double aXInitKF[5], aYInitKF[5], aXInit[5], aYInit[5];
  double vXInitKF[3], vYInitKF[3], vXInit[3], vYInit[3];
  geometry_msgs::Vector3 lastAccelKF, lastAccel, lastVelKF, lastVel;
  geometry_msgs::Vector3 accel, velKF, vel, posKF, pos;
  sensor_msgs::Imu raw_imu;

public:
  DeadReckon1D();
  ~DeadReckon1D();
  void InitMsgs();
  void IMUCB(const sensor_msgs::Imu::ConstPtr& imu);
  void IMUFilterCB(const imu_3dm_gx4::FilterOutput::ConstPtr& imu);
};

#endif
