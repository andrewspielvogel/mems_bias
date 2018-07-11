/**
 * @file
 * @date Feb. 2018.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).
 * @brief Class for MEMS IMU bias estimation. 
 */


#ifndef MEMS_BIAS_H
#define MEMS_BIAS_H

#include <Eigen/Core>

/**
 *
 * @brief Bias estimator params.
 *
 */
class BiasParams
{

 public:
  Eigen::Matrix3d K_acc; /**< Linear acceleration estimate gain. */
  Eigen::Matrix3d K_mag; /**< Magnetometer estimate gain. */

  Eigen::Matrix3d K_acc_bias; /**< Linear acceleration bias gain. */
  Eigen::Matrix3d K_ang_bias; /**< Angular-rate bias gain. */
  Eigen::Matrix3d K_mag_bias; /**< Magnetometer bias gain. */

};

/**
 *
 * @brief IMU packet
 *
 */
class ImuPacket
{

 public:

  Eigen::Vector3d ang; /**< Angular-rate measurement. */
  Eigen::Vector3d acc; /**< Linear-acceleration measurement. */
  Eigen::Vector3d mag; /**< Magnetometer measurement. */

  int seq_num; /**< Measurement sequence number. */

  double t; /**< Measurement timestamp. */

};


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
  MEMSBias(BiasParams params);

  
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


 private:

  double prev_time_; /**< Previous measurement timestamp. */

  int start_; 

  BiasParams params_; /**< Bias estimator parameters. */


};

#endif
