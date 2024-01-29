
// header for synchronous message
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/TimeReference.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include "std_msgs/String.h"


#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
// #include "cv_bridge/CvBridge.h"
#include <image_transport/image_transport.h>
#include <boost/format.hpp>

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>


#include <boost/foreach.hpp>
// crophull 3d filter
#include <boost/shared_ptr.hpp>
// segmentation


// approximate time policy
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>


//pcl-related header files
// pcl related


#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/common/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
// crophull 3d filter
#include <pcl/common/common.h>
#include <boost/shared_ptr.hpp>
// segmentation
#include <pcl_conversions/pcl_conversions.h>


//mutex or boost ftn

#include <iostream>

#include "ros/service.h"
#include "ros/service_server.h"
#include <ros/package.h>

#include <stdio.h>
#include <iostream>
#include <string>


#include <tf2_eigen/tf2_eigen.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>


// time
#include <chrono>


// save files
#include <sstream>// std::stringstream
#include <cstdlib>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error


// msgs for pose and force saving

#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include "tf/transform_listener.h"

#include <math.h>


boost::format g_format;
bool save_all_image, save_image_service;
std::string encoding;
bool request_start_end;

using namespace std;
// using namespace sensor_msgs;
// using namespace message_filters;

class Save
{
public:
 Save(ros::NodeHandle nh);
 ~Save();

 void Initialize();
 void Update();
 void KeyboardChecking();


private:
 ros::NodeHandle n;

 static const int MAX_COUNT = 5;
 static const int MAX_WAIT = 1000;
 int count;
 int wait;
 int rwFlag;
 int countingFlag;
 int countingFlag_tf;
 int imgCountingFlag;
 int img2CountingFlag;
 int realSenseFlag;
 int tfFlag;
 int tf_objectFlag;
 int pcFlag0;



/////// async policy

    message_filters::Subscriber<sensor_msgs::Image> sub_tact_;
    message_filters::Subscriber<sensor_msgs::Image> sub_webcam_;
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_;
    message_filters::Subscriber<sensor_msgs::Image> sub_tact_depth_;
    message_filters::Subscriber<sensor_msgs::TimeReference> sub_cmd_;
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
    //                                                         sensor_msgs::Image, 
    //                                                         sensor_msgs::TimeReference> cap_syncPolicy;


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
                                                            sensor_msgs::Image,
                                                            sensor_msgs::Image, 
                                                            sensor_msgs::Image> cap_syncPolicy;

    typedef message_filters::Synchronizer<cap_syncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

///////

 void Callback_assorted(const sensor_msgs::ImageConstPtr &img_tact, 
                        const sensor_msgs::ImageConstPtr &img_web, 
                        const sensor_msgs::ImageConstPtr &img_depth, 
                        const sensor_msgs::ImageConstPtr &img_tact_depth); 
 void cmd_Callback(const sensor_msgs::TimeReference::ConstPtr& time_ref);
 void point0_Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
// void point1_Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
// void point2_Callback(const sensor_msgs::PointCloud2ConstPtr& msg);

 void wrench_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
 void pose_object_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
 void pose_sensor_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

 geometry_msgs::WrenchStamped dataAti;
 geometry_msgs::PoseStamped eePose;
 geometry_msgs::PoseStamped objectPose;
 geometry_msgs::PoseStamped sensorPose;
 geometry_msgs::WrenchStamped dataAti_temp;
 geometry_msgs::PoseStamped eePose_temp;
 geometry_msgs::PoseStamped objectPose_temp;
 geometry_msgs::PoseStamped sensorPose_temp;


 std::string keyboardCom;
 std::string test;
 std::string test_1;

 image_transport::ImageTransport it;
 image_transport::Subscriber sub_image;
 image_transport::Subscriber sub_image2;
 image_transport::Subscriber sub_image_realsense;


 // tf::TransformListener tf_listener_;
 tf2_ros::Buffer tfBuffer;
 std::unique_ptr<tf2_ros::TransformListener> tfListener;


 geometry_msgs::TransformStamped current_tact_transform;
 geometry_msgs::TransformStamped current_realsense_transform;
 geometry_msgs::TransformStamped current_tag0_transform;
 geometry_msgs::TransformStamped current_tag1_transform;
 geometry_msgs::TransformStamped current_tf_btw_tact_tag1;



 ros::Subscriber sub0;
 ros::Subscriber sub_pc0;
 ros::Subscriber sub_pc1;
 ros::Subscriber sub_pc2;
 ros::Subscriber sub_wrench;
 ros::Subscriber sub_pose;
 ros::Subscriber sub_pose_object;
 ros::Subscriber sub_pose_sensor;

 ros::Publisher pub;
 ros::Time stamp_assorted;
 std::stringstream ss_assorted;


 void gettfframe(std::string filename);
 void gettfframe_object(std::string filename);
 void get_tfframe_btw_tact_object(std::string filename);

 void reset_tfframe_object(std::string filename);

 void write_img(const sensor_msgs::ImageConstPtr& image_msg, std::string filename, int depthFlag);
 void write_frames(std::string filename);
 void write_pose(const geometry_msgs::PoseStamped::ConstPtr& sensorPose,std::string filename, std::string ss);

void copyWrench(geometry_msgs::WrenchStamped &in, geometry_msgs::WrenchStamped &out);
void copyPose(geometry_msgs::PoseStamped &in, geometry_msgs::PoseStamped &out);

void callbackTact(const sensor_msgs::ImageConstPtr& image_msg);
void callbackTact2(const sensor_msgs::ImageConstPtr& image_msg);
 void callbackImageRealSense(const sensor_msgs::ImageConstPtr& image_msg);



};
