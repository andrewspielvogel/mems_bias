/**
 * @file
 * @date Feb. 2018.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).
 * @brief Implementation of mems_bias.h.
 *
 */

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <mems_bias/mems_bias.h>
#include <helper_funcs/helper_funcs.h>
#include <iostream>


/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

MEMSBias::MEMSBias(config_params params) 
{
  
  params_ = params;

  start_ = 0;
  
  acc_bias = params.acc_bias;
  ang_bias = params.ang_bias;
  mag_bias = params.mag_bias;

}

MEMSBias::~MEMSBias(void)
{
}

void MEMSBias::step(ImuPacket measurement)
{

      
  if (!start_)
  {

    acc_hat = measurement.acc;
    mag_hat = measurement.mag;
      
    prev_time_ = measurement.t;
      
    start_ = 1;
      
  }

  
  /**************************************************************
   * Sensor Bias Estimator
   **************************************************************/

  double dt = measurement.t - prev_time_;
  
  Eigen::Vector3d da           = acc_hat - measurement.acc;
  Eigen::Vector3d dm           = mag_hat - measurement.mag;
  Eigen::Vector3d acc_hat_dot  = -skew(measurement.ang)*(acc_hat - acc_bias) + skew(ang_bias)*acc_hat - params_.K_acc*da;
  Eigen::Vector3d mag_hat_dot  = -skew(measurement.ang - ang_bias)*mag_hat + skew(measurement.ang)*mag_bias - params_.K_mag*dm;
  Eigen::Vector3d ang_bias_dot = -params_.K_ang_bias*(skew(measurement.acc)*da + skew(measurement.mag)*dm);
  Eigen::Vector3d acc_bias_dot = params_.K_acc_bias*skew(measurement.ang)*da;
  Eigen::Vector3d mag_bias_dot = params_.K_mag_bias*skew(measurement.ang)*dm;


  acc_hat  = acc_hat  + dt*acc_hat_dot;
  mag_hat  = mag_hat  + dt*mag_hat_dot;
  ang_bias = ang_bias + dt*ang_bias_dot;
  acc_bias = acc_bias + dt*acc_bias_dot;
  mag_bias = mag_bias + dt*mag_bias_dot;
  
  prev_time_ = measurement.t;

  imu_corrected = measurement;
  imu_corrected.ang = measurement.ang - ang_bias;
  imu_corrected.acc = measurement.acc - acc_bias;
  imu_corrected.mag = measurement.mag - mag_bias;
  
}
