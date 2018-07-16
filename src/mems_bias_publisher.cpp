/**
 * @file
 * @date July 2018
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Node for MEMS bias estimation.
 */


#include <ros/ros.h>
#include <dscl_msgs/Imu9DOF.h>
#include <dscl_msgs/ImuBias.h>
#include <mems_bias/mems_bias.h>
#include <helper_funcs/helper_funcs.h>
#include <ros/node_handle.h>



/**
 *
 * @brief Class for launching MEMS bias estimation rosnode.
 * 
 */
class BiasNode
{

private:
  
  ros::Publisher chatter_; /**< Node publisher. */
  ros::Publisher chatter_corrected_;

  MEMSBias* bias_;
  
public:

  BiasParams params;

  /**
   *
   * @brief Constructor.
   * 
   * @param n ROS NodeHandle
   * 
   */
  BiasNode(ros::NodeHandle n){



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
    n.param<std::string>("frame_id",params.frameId, "imu");

    params.K_acc      = stringToDiag(k_acc);
    params.K_mag      = stringToDiag(k_mag);
    params.K_acc_bias = stringToDiag(k_acc_bias);
    params.K_ang_bias = stringToDiag(k_ang_bias);
    params.K_mag_bias = stringToDiag(k_mag_bias);


    chatter_ = n.advertise<dscl_msgs::ImuBias>("imu_bias",1);
    chatter_corrected_ = n.advertise<dscl_msgs::Imu9DOF>("imu_corrected",1);

    bias_ = new MEMSBias(params);

  }

  /**
   *
   * @brief Publishing callback function.
   * 
   */
  void callback(const dscl_msgs::Imu9DOF::ConstPtr& msg)
  {

    ImuPacket measurement;

    measurement.acc << msg->acc.x,msg->acc.y,msg->acc.z;
    measurement.ang << msg->ang.x,msg->ang.y,msg->ang.z;
    measurement.mag << msg->mag.x,msg->mag.y,msg->mag.z;
    
    measurement.t = msg->header.stamp.toSec();

    bias_->step(measurement);
    
    // initialize mems_bias msg
    dscl_msgs::ImuBias bias;
    dscl_msgs::Imu9DOF imu_corrected;

    bias.header.stamp    = msg->header.stamp;
    bias.header.frame_id  = params.frameId;
    
    bias.ang.x = bias_->ang_bias(0);
    bias.ang.y = bias_->ang_bias(1);
    bias.ang.z = bias_->ang_bias(2);
    bias.acc.x = bias_->acc_bias(0);
    bias.acc.y = bias_->acc_bias(1);
    bias.acc.z = bias_->acc_bias(2);
    bias.mag.x = bias_->mag_bias(0);
    bias.mag.y = bias_->mag_bias(1);
    bias.mag.z = bias_->mag_bias(2);

    imu_corrected.header.stamp   = msg->header.stamp;
    imu_corrected.header.frame_id = params.frameId;
    
    imu_corrected.ang.x        = msg->ang.x - bias_->ang_bias(0);
    imu_corrected.ang.y        = msg->ang.y - bias_->ang_bias(1);
    imu_corrected.ang.z        = msg->ang.z - bias_->ang_bias(2);
    imu_corrected.acc.x        = msg->acc.x - bias_->acc_bias(0);
    imu_corrected.acc.y        = msg->acc.y - bias_->acc_bias(1);
    imu_corrected.acc.z        = msg->acc.z - bias_->acc_bias(1);
    imu_corrected.mag.x        = msg->mag.x - bias_->mag_bias(0);
    imu_corrected.mag.y        = msg->mag.y - bias_->mag_bias(1);
    imu_corrected.mag.z        = msg->mag.z - bias_->mag_bias(2);


    // publish packet
    chatter_.publish(bias);
    chatter_corrected_.publish(imu_corrected);
    
  }
 
};

int main(int argc, char **argv)
{

    // initialize node
    ros::init(argc, argv, "mems_bias");

    ros::NodeHandle n("~");;

    ros::Subscriber sub; /**< Node subscriber to imu topic. */

 
    /**********************************************************************
     * Initialize Bias Estimator
     **********************************************************************/
    BiasNode bias_est(n);

    char buffer[64];

    sprintf(buffer,"/%s/imu",bias_est.params.frameId.c_str());

    sub = n.subscribe(buffer,1,&BiasNode::callback, &bias_est);
    
    ros::spin();
    
    return 0;
}
