#include "riptide_hardware/dead_reckon_1D.h"

#define X 0
#define Y 1

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dead_reckon_1D");
  DeadReckon1D DR;
  ros::spin();
}

//Constructor
DeadReckon1D::DeadReckon1D() : nh("dead_reckon_1D")
{
  imu_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &DeadReckon1D::ImuCB, this);
  imu_filter_sub = nh.subscribe<imu_3dm_gx4::FilterOutput>("/imu/filter", 1, &DeadReckon1D::FilterCallback, this);

  initKF = false;
  initPP = false;
  tLastKF = 0;
  tLastPP = 0;
  deltaT = 0.01;
  count = 0;
  minCount = 5; // Need 3 data points accel to get vel, need 3 calculated vel's to get pos (requires 5 accel data points)

  // Initialize arrays
  for (int i = 0; i < minCount; i++)
  {
    aXInitKF[i] = 0;
    aXInitPP[i] = 0;
    aYInitKF[i] = 0;
    aYInitPP[i] = 0;
  }

  //Create a new file for logging IMU data
  bool found_new_file_name = false;
  int num = 1; //Begin file counting here
  sprintf(file_name_KF, "//home//tsender//osu-uwrt//DR_1DKF_%i.csv", num);
  sprintf(file_name_PP, "//home//tsender//osu-uwrt//DR_1DPP_%i.csv", num);

  //If unable to open for reading, then the file does not exist --> new file_name
  while (!found_new_file_name)
  {
    ROS_INFO("Dead Reckon 1D KF File Name: ");
    ROS_INFO("\t%s", file_name_KF);
    fid = fopen(file_name_KF, "r");
    if (fid)
    { //File already exists
      //Increase num, and create new file name
      fclose(fid);
      num++;
      sprintf(file_name_KF, "//home//tsender//osu-uwrt//DR_1DKF_%i.csv", num);
      sprintf(file_name_PP, "//home//tsender//osu-uwrt//DR_1DPP_%i.csv", num);
    }
    else
    { //File does not exist
      found_new_file_name = true;
    }
  }
}

//Log magnetometer vector components
void DeadReckon1D::ImuCB(const riptide_msgs::Imu::ConstPtr &imu)
{
  //Open file and print values
  fid = fopen(file_name_PP, "a"); //Open file for "appending"
  if (!fid)
  {
    ROS_INFO("DeadReckon_1DPP: file not opened");
  }

  if (!initPP)
  {
    aXInitPP[count] = imu->linear_accel.x;
    aYInitPP[count] = imu->linear_accel.y;
    count++;

    if (count == minCount)
    {
      lastAccelPP.x = imu->linear_accel.x;
      lastAccelPP.y = imu->linear_accel.y;

      // Calculate initial 3 velocities
      vXInitPP[0] = deltaT / 2 * (aXInitPP[0] + 2 * aXInitPP[1] + aXInitPP[2]);
      vXInitPP[1] = vXInitPP[0] + deltaT / 2 * (aXInitPP[2] + aXInitPP[3]);
      vXInitPP[2] = vXInitPP[1] + deltaT / 2 * (aXInitPP[3] + aXInitPP[4]);
      velPP.x = vXInitPP[2];
      lastVelPP.x = velPP.x;

      vYInitPP[0] = deltaT / 2 * (aYInitPP[0] + 2 * aYInitPP[1] + aYInitPP[2]);
      vYInitPP[1] = vYInitPP[0] + deltaT / 2 * (aYInitPP[2] + aYInitPP[3]);
      vYInitPP[2] = vYInitPP[1] + deltaT / 2 * (aYInitPP[3] + aYInitPP[4]);
      velPP.y = vYInitPP[2];
      lastVelPP.y = velPP.y;
      lastVelPP.z = 0;

      // Calculate initial positions
      posPP.x = deltaT / 2 * (vXInitPP[0] + 2 * vXInitPP[1] + vXInitPP[2]);
      posPP.y = deltaT / 2 * (vYInitPP[0] + 2 * vYInitPP[1] + vYInitPP[2]);
      posPP.z = 0;

      // Write first values to post-processed (PP) file
      fprintf(fid, "%f,", 0.0);
      fprintf(fid, "%f,", lastAccelPP.x);
      fprintf(fid, "%f,", velPP.x);
      fprintf(fid, "%f,", posPP.x);
      fprintf(fid, "%f,", lastAccelPP.y);
      fprintf(fid, "%f,", velPP.y);
      fprintf(fid, "%f\n", posPP.y);
      fclose(fid);

      tLastPP = imu->header.stamp.toSec();
      initPP = true;

      ROS_INFO("PP: pos.x = %f", posPP.x);
    }
  }
  else
  {
    // Get newest accel's
    double ax = imu->linear_accel.x;
    double ay = imu->linear_accel.y;

    // Calculate new velocities and positions
    velPP.x += deltaT / 2 * (lastAccelPP.x + ax);
    velPP.y += deltaT / 2 * (lastAccelPP.y + ay);

    posPP.x += deltaT / 2 * (lastVelPP.x + velPP.x);
    posPP.y += deltaT / 2 * (lastVelPP.y + velPP.y);

    // Update last variables
    lastAccelPP.x = ax;
    lastAccelPP.y = ay;
    lastVelPP.x = velPP.x;
    lastVelPP.y = velPP.y;

    // Write values to PP file
    fprintf(fid, "%f,", imu->header.stamp.toSec() - tLastPP);
    fprintf(fid, "%f,", aXInitPP[count]);
    fprintf(fid, "%f,", velPP.x);
    fprintf(fid, "%f,", posPP.x);
    fprintf(fid, "%f,", aYInitPP[count]);
    fprintf(fid, "%f,", velPP.y);
    fprintf(fid, "%f\n", posPP.y);
    fclose(fid);

    tLastPP = imu->header.stamp.toSec();
    ROS_INFO("PP: pos.x = %f", posPP.x);
  }
}

void DeadReckon1D::FilterCallback(const imu_3dm_gx4::FilterOutput::ConstPtr &filter_msg)
{
}
