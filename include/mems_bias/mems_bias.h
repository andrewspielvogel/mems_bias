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
  Eigen::Matrix3d k_acc;
  Eigen::Matrix3d k_mag;

  Eigen::Matrix3d k_acc_bias;
  Eigen::Matrix3d k_ang_bias;
  Eigen::Matrix3d k_mag_bias;

};

/**
 *
 * @brief IMU packet
 *
 */
class ImuPacket
{

 public:

  Eigen::Vector3d ang;
  Eigen::Vector3d acc;
  Eigen::Vector3d mag;

  int seq_num;

  double t;

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

  double prev_time_;

  int start_;

  BiasParams params_;


};

#endif
