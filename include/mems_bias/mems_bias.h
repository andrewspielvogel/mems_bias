/**
 * @file
 * @date Feb. 2018.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).
 * @brief Class for MEMS IMU bias estimation. 
 */


#ifndef MEMS_BIAS_H
#define MEMS_BIAS_H

#include <Eigen/Core>
#include <helper_funcs/helper_funcs.h>




/**
 * @brief Class for adaptive IMU bias identificaiton.
 */
class MEMSBias
{
public:

  /**
   * @brief Constructor.
   *
   * @param params Estimator parameters.
   *
   */
  MEMSBias(config_params params);

  
  virtual ~MEMSBias(void); /**< Destructor. */
    
  /**
   * @brief Cycle estimation once.
   *
   * @param measurement IMU measurement packet.
   *
   */
  void step(ImuPacket measurement);

  
  Eigen::Vector3d acc_hat;
  Eigen::Vector3d mag_hat;
  Eigen::Vector3d ang_bias;
  Eigen::Vector3d acc_bias;
  Eigen::Vector3d mag_bias;

  ImuPacket imu_corrected;


 private:

  double prev_time_; /**< Previous measurement timestamp. */

  int start_; 

  config_params params_; /**< Bias estimator parameters. */


};

#endif
