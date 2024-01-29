// using namespace message_filters;
#include "capturedata_mocap.h"


class Node
{
 public:
  Node()
  {
    sub_tact_.subscribe(nh_, "in1", 1);
    sub_webcam_.subscribe(nh_, "in2", 1);
    sub_cmd_.subscribe(nh_, "in3", 1);
    sync_.reset(new Sync(cap_syncPolicy(20), sub_tact_, sub_webcam_, sub_cmd_));
    sync_->registerCallback(boost::bind(&Node::callback, this, _1, _2, _3));
  }


  void callback(const sensor_msgs::ImageConstPtr &img_tact, const sensor_msgs::ImageConstPtr &img_web, const sensor_msgs::TimeReference::ConstPtr &time_ref )
  {
    ROS_INFO("Synchronization successful");
  }

 private:
  ros::NodeHandle nh_;
  // defining filters 
  message_filters::Subscriber<sensor_msgs::Image> sub_tact_;
  message_filters::Subscriber<sensor_msgs::Image> sub_webcam_;
  message_filters::Subscriber<sensor_msgs::TimeReference> sub_cmd_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::TimeReference> cap_syncPolicy;
  typedef message_filters::Synchronizer<cap_syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "synchronizer");

  Node synchronizer;

  ros::spin();
}