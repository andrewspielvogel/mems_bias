/**
 * @file
 * @date July 2018
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Node for MEMS bias estimation.
 */


#include <ros/ros.h>
#include <mems_bias/Imu9DOF.h> 
#include <mems_bias/ImuBias.h> 



/**
 *
 * @brief Class for launching MEMS bias estimation rosnode.
 * 
 */
class BiasNode
{

private:
  
  ros::Publisher chatter_; /**< Node publisher. */
  
public:

  /**
   *
   * @brief Constructor.
   * 
   * @param n ROS NodeHandle
   * 
   */
  BiasNode(ros::NodeHandle n){
    
    chatter_ = n.advertise<mems_bias::ImuBias>("imu_bias",1);

  }

  /**
   *
   * @brief Publishing callback function.
   * 
   */
  void callback(const mems_bias::Imu9DOF::ConstPtr& msg)
  {


    // initialize mems_bias msg
    mems_bias::ImuBias bias;

    bias.header.stamp    = msg->header.stamp;
    bias.header.frame_id = "bias";
    
    bias.ang.x = msg->ang.x;
    bias.ang.y = msg->ang.y;
    bias.ang.z = msg->ang.z;
    bias.acc.x = msg->acc.x;
    bias.acc.y = msg->acc.y;
    bias.acc.z = msg->acc.z;
    bias.mag.x = msg->mag.x;
    bias.mag.y = msg->mag.y;
    bias.mag.z = msg->mag.z;

    // publish packet
    chatter_.publish(bias);
    
  }
 
};

int main(int argc, char **argv)
{

    // initialize node
    ros::init(argc, argv, "mems_bias");

    ros::NodeHandle n;

    ros::Subscriber sub; /**< Node subscriber to PHINS topic. */

    /**********************************************************************
     * Initialize Bias Estimator
     **********************************************************************/
    BiasNode bias_est(n);

    sub = n.subscribe("imu/imu",1,&BiasNode::callback, &bias_est);
    
    ros::spin();
    
    return 0;
}
