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
 * @brief Function for converting string to 3x3 diagonal matrix.
 *
 * @param str Comma seperated list of three diag elements. e.g. "1,1,1"
 *
 */
Eigen::Matrix3d stringToDiag(std::string str)
{
  Eigen::Matrix3d diag;
  Eigen::Vector3d vec;
  sscanf(str.c_str(),"%lf,%lf,%lf",&vec(0),&vec(1),&vec(2));

  diag << vec(0),0,0,0,vec(1),0,0,0,vec(2);

  return diag;

}


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

    ROS_INFO("Loading estimator params.");

    std::string k_acc;
    std::string k_mag;
    std::string k_acc_bias;
    std::string k_ang_bias;
    std::string k_mag_bias;

    n.param<std::string>("k_acc",k_acc, "1,1,1");
    n.param<std::string>("k_mag",k_mag, "1,1,1");
    n.param<std::string>("k_acc_bias",k_acc_bias, "0.1,0.1,0.1");
    n.param<std::string>("k_ang_bias",k_ang_bias, "0.01,0.01,0.01");
    n.param<std::string>("k_mag_bias",k_mag_bias, "0.5,0.5,0.5");

    params.K_acc      = stringToDiag(k_acc);
    params.K_mag      = stringToDiag(k_mag);
    params.K_acc_bias = stringToDiag(k_acc_bias);
    params.K_ang_bias = stringToDiag(k_ang_bias);
    params.K_mag_bias = stringToDiag(k_mag_bias);
    
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
