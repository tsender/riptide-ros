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
  imu_sub = nh.subscribe<imu_3dm_gx4::FilterOutput>("/imu/filter", 1, &DeadReckon1D::ImuCB, this);
  //imu_filter_sub = nh.subscribe<imu_3dm_gx4::FilterOutput>("/imu/filter", 1, &DeadReckon1D::FilterCallback, this);

  initKF = false;
  init = false;
  tLastKF = 0;
  tStart = 0;
  deltaT = 0.01;
  count = 0;
  minCount = 5; // Need 3 data points accel to get vel, need 3 calculated vel's to get pos (requires 5 accel data points)

  // Initialize arrays
  for (int i = 0; i < minCount; i++)
  {
    aXInitKF[i] = 0;
    aXInit[i] = 0;
    aYInitKF[i] = 0;
    aYInit[i] = 0;
  }

  //Create a new file for logging IMU data
  bool found_new_file_name = false;
  num = 1; //Begin file counting here
  //sprintf(file_name_KF, "//home//tsender//osu-uwrt//DR_1DKF_%i.csv", num);
  sprintf(file_name, "//home//tsender//osu-uwrt//DR_1D_%i.csv", num);

  //If unable to open for reading, then the file does not exist --> new file_name
  while (!found_new_file_name)
  {
    ROS_INFO("Dead Reckon 1D File Name: ");
    ROS_INFO("\t%s", file_name);
    fid = fopen(file_name, "r");
    if (fid)
    { //File already exists
      //Increase num, and create new file name
      fclose(fid);
      num++;
      //sprintf(file_name_KF, "//home//tsender//osu-uwrt//DR_1DKF_%i.csv", num);
      sprintf(file_name, "//home//tsender//osu-uwrt//DR_1D_%i.csv", num);
    }
    else
    { //File does not exist
      found_new_file_name = true;
    }
  }
}

//Log magnetometer vector components
void DeadReckon1D::ImuCB(const imu_3dm_gx4::FilterOutput::ConstPtr &imu)
{
  //Open file and print values
  fid = fopen(file_name, "a"); //Open file for "aending"
  if (!fid)
  {
    ROS_INFO("DeadReckon_1D: file not opened");
  }

  if (!init)
  {
    aXInit[count] = imu->linear_acceleration.x;
    aYInit[count] = imu->linear_acceleration.y;
    count++;

    if (count == minCount)
    {
      lastAccel.x = imu->linear_acceleration.x;
      lastAccel.y = imu->linear_acceleration.y;

      // Calculate initial 3 velocities
      vXInit[0] = deltaT / 2 * (aXInit[0] + 2 * aXInit[1] + aXInit[2]);
      vXInit[1] = vXInit[0] + deltaT / 2 * (aXInit[2] + aXInit[3]);
      vXInit[2] = vXInit[1] + deltaT / 2 * (aXInit[3] + aXInit[4]);
      vel.x = vXInit[2];
      lastVel.x = vel.x;

      vYInit[0] = deltaT / 2 * (aYInit[0] + 2 * aYInit[1] + aYInit[2]);
      vYInit[1] = vYInit[0] + deltaT / 2 * (aYInit[2] + aYInit[3]);
      vYInit[2] = vYInit[1] + deltaT / 2 * (aYInit[3] + aYInit[4]);
      vel.y = vYInit[2];
      lastVel.y = vel.y;
      lastVel.z = 0;

      // Calculate initial positions
      pos.x = deltaT / 2 * (vXInit[0] + 2 * vXInit[1] + vXInit[2]);
      pos.y = deltaT / 2 * (vYInit[0] + 2 * vYInit[1] + vYInit[2]);
      pos.z = 0;

      // Write first values to post-processed () file
      fprintf(fid, "%f,", 0.0);
      fprintf(fid, "%f,", lastAccel.x);
      fprintf(fid, "%f,", vel.x);
      fprintf(fid, "%f,", pos.x);
      fprintf(fid, "%f,", lastAccel.y);
      fprintf(fid, "%f,", vel.y);
      fprintf(fid, "%f\n", pos.y);
      fclose(fid);

      tStart = imu->header.stamp.toSec();
      init = true;

      ROS_INFO("pos.x = %f", pos.x);
    }
  }
  else
  {
    // Get newest accel's
    accel.x = imu->linear_acceleration.x;
    accel.y = imu->linear_acceleration.y;

    // Calculate new velocities and positions
    vel.x += deltaT / 2 * (lastAccel.x + accel.x);
    vel.y += deltaT / 2 * (lastAccel.y + accel.y);

    pos.x += deltaT / 2 * (lastVel.x + vel.x);
    pos.y += deltaT / 2 * (lastVel.y + vel.y);

    // Update last variables
    lastAccel.x = accel.x;
    lastAccel.y = accel.y;
    lastVel.x = vel.x;
    lastVel.y = vel.y;

    // Write values to  file
    fprintf(fid, "%f,", imu->header.stamp.toSec() - tStart);
    fprintf(fid, "%f,", accel.x);
    fprintf(fid, "%f,", vel.x);
    fprintf(fid, "%f,", pos.x);
    fprintf(fid, "%f,", accel.y);
    fprintf(fid, "%f,", vel.y);
    fprintf(fid, "%f\n", pos.y);
    fclose(fid);

    ROS_INFO("File %i, t=%.5f: Ax=%.5f, Vx=%.5f, Px=%.5f", num, imu->header.stamp.toSec() - tStart, accel.x, vel.x, pos.x);
  }
}

/*void DeadReckon1D::FilterCallback(const imu_3dm_gx4::FilterOutput::ConstPtr &filter_msg)
{
}*/
