/**
 * @file
 * @date July 2018
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Node for MEMS bias estimation.
 */


#include <ros/ros.h>
#include <mems_bias/Imu9DOF.h> 
#include <mems_bias/ImuBias.h>
#include <mems_bias/mems_bias.h>



/**
 *
 * @brief Class for launching MEMS bias estimation rosnode.
 * 
 */
class BiasNode
{

private:
  
  ros::Publisher chatter_; /**< Node publisher. */

  MEMSBias* bias_;
  
public:

  /**
   *
   * @brief Constructor.
   * 
   * @param n ROS NodeHandle
   * 
   */
  BiasNode(ros::NodeHandle n){


    BiasParams params;

    params.k_acc      = Eigen::Matrix3d::Identity();
    params.k_mag      = Eigen::Matrix3d::Identity();
    params.k_acc_bias = Eigen::Matrix3d::Identity()/10;
    params.k_ang_bias = Eigen::Matrix3d::Identity()/100;
    params.k_mag_bias = Eigen::Matrix3d::Identity()/2;
    
    chatter_ = n.advertise<mems_bias::ImuBias>("imu_bias",1);

    bias_ = new MEMSBias(params);

  }

  /**
   *
   * @brief Publishing callback function.
   * 
   */
  void callback(const mems_bias::Imu9DOF::ConstPtr& msg)
  {

    ImuPacket measurement;

    measurement.acc << msg->acc.x,msg->acc.y,msg->acc.z;
    measurement.ang << msg->ang.x,msg->ang.y,msg->ang.z;
    measurement.mag << msg->mag.x,msg->mag.y,msg->mag.z;
    
    measurement.t = msg->header.stamp.toSec();

    bias_->step(measurement);
    
    // initialize mems_bias msg
    mems_bias::ImuBias bias;

    bias.header.stamp    = msg->header.stamp;
    
    bias.ang.x = bias_->ang_bias(0);
    bias.ang.y = bias_->ang_bias(1);
    bias.ang.z = bias_->ang_bias(2);
    bias.acc.x = bias_->acc_bias(0);
    bias.acc.y = bias_->acc_bias(1);
    bias.acc.z = bias_->acc_bias(2);
    bias.mag.x = bias_->mag_bias(0);
    bias.mag.y = bias_->mag_bias(1);
    bias.mag.z = bias_->mag_bias(2);

    // publish packet
    chatter_.publish(bias);
    
  }
 
};

int main(int argc, char **argv)
{

    // initialize node
    ros::init(argc, argv, "mems_bias");

    ros::NodeHandle n;

    ros::Subscriber sub; /**< Node subscriber to imu topic. */

 
    /**********************************************************************
     * Initialize Bias Estimator
     **********************************************************************/
    BiasNode bias_est(n);

    sub = n.subscribe("imu/imu",1,&BiasNode::callback, &bias_est);
    
    ros::spin();
    
    return 0;
}
