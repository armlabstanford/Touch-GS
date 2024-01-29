
////////////////////////////////////////
// 		 code for saving tf / image(from realsense and tactile sensor)		//
////////////////////////////////////////


#include "capturedata_imgndepth.h"
std::string  lib_cmd;


using namespace sensor_msgs;
using namespace message_filters;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "capturedata_mocap_sync");
  ros::NodeHandle n;
  Save check(n);

// mutex for in case of image saving process makes slow

  check.Initialize();

  ros::Rate loop_rate(10);
	while(ros::ok())
	{
	  check.Update();
	  loop_rate.sleep();
    //ros::Time curr = ros::Time::now();
    //ROS_INFO_STREAM("Loop time: " <<  curr.toSec()-last.toSec());
    //last = curr;
	}
return 0;

    }

Save::Save(ros::NodeHandle nh) :
 n(nh),
 it(n),
 count(MAX_COUNT),
 wait(MAX_WAIT),
 rwFlag(0),
  countingFlag(0),
  countingFlag_tf(0),
  imgCountingFlag(0),
  img2CountingFlag(0),
 realSenseFlag(0),
 tf_objectFlag(0),
 tfFlag(0),
 pcFlag0(0)
{
}
Save::~Save()
{	// delete listener0;
}



void Save::Initialize(){
using namespace sensor_msgs;
using namespace message_filters;

    tfListener.reset(new tf2_ros::TransformListener(tfBuffer));

  message_filters::Subscriber<sensor_msgs::Image> image_tact_sub(n, "/RunCamera/image_raw_1", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_webcam_sub(n, "/RunCamera/webcam", 1);
//   message_filters::Subscriber<sensor_msgs::Image> image_depth_sub(n, "/camera/depth/image_rect_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub(n, "/camera/aligned_depth_to_color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_tact_depth_sub(n, "/RunCamera/imgDepth", 1);
//   message_filters::Subscriber<sensor_msgs::TimeReference> cmd_sub(n, "/capturedata/lib_cmd", 1);

    sub_tact_.subscribe(n, "/RunCamera/image_raw_1", 1);
    sub_webcam_.subscribe(n, "/RunCamera/webcam", 1);
    sub_depth_.subscribe(n, "/camera/depth/image_rect_raw", 1);
    sub_tact_depth_.subscribe(n, "/RunCamera/imgDepth", 1);
    // sub_cmd_.subscribe(n, "/capturedata/lib_cmd", 1);

    sync_.reset(new Sync(cap_syncPolicy(100), sub_tact_, sub_webcam_, sub_depth_, sub_tact_depth_));
    sync_->registerCallback(boost::bind(&Save::Callback_assorted, this, _1, _2, _3, _4));

    sub0 = n.subscribe<sensor_msgs::TimeReference>("/capturedata/lib_cmd",100, &Save::cmd_Callback, this);

    std::cout << "initializing done. "  << std::endl;

}


void Save::Update(){

  ros::spinOnce();
}

void Save::cmd_Callback(const sensor_msgs::TimeReference::ConstPtr& time_ref){

	ROS_INFO("CTRL: Heard: [%s]", time_ref->source.c_str());
    ros::Time stamp = time_ref->header.stamp;
    std::stringstream ss;
    ss << stamp.sec << "." << stamp.nsec;
    std::cout << "time is: " << ss.str() << std::endl;
    // use below for time stamp 
    // ss.str()
	keyboardCom = time_ref->source.c_str();

	if (keyboardCom.compare("saveall") == 0)
	{
		std::cout << "\tSave image and depth/force data" << std::endl;

		rwFlag = 1;

        tfFlag = 1;

    }


	else if (keyboardCom.compare("tactsave") == 0){
		std::cout << "update parameters from the file" << std::endl;
		rwFlag = 2;

	}
    else if (keyboardCom.compare("calibstart") == 0){
//        tf_objectFlag= 1;
    }
    else if (keyboardCom.compare("calibsave") == 0){
         std::cout << "open and get the average of the object transform and save it" << std::endl;
//         tf_objectFlag = 2;

    }
    else if (keyboardCom.compare("calibend") == 0){
         std::cout << "get the tf matrix from the tactile sensor frame to tag1(attached in upper side of the cube)" << std::endl;
//         tf_objectFlag = 3;

    }

}


void Save::Callback_assorted(const sensor_msgs::ImageConstPtr &img_tact, 
                        const sensor_msgs::ImageConstPtr &img_web, 
                        const sensor_msgs::ImageConstPtr &img_depth, 
                        const sensor_msgs::ImageConstPtr &img_tact_depth){
//// 
//  all the callback should be synchronized
////

    // std::cout << "\tCallback_assorted working.." << std::endl;
	if (rwFlag == 1)
	{

        stamp_assorted = img_tact->header.stamp;
        std::stringstream ss_new;

        ss_new << stamp_assorted.sec << "." << stamp_assorted.nsec;
        std::string timestr = ss_new.str();

        std::cout << "time is: " << timestr << std::endl;
        std::string filename;
        std::string cF = std::to_string(countingFlag);

        // save tactile sensor image data
        filename = ros::package::getPath("capturedata")+"/data/touch_images/touch_image_"+cF+ ".jpg";
        write_img(img_tact,filename, 0);

        // save webcam image data
        filename = ros::package::getPath("capturedata")+"/data/camera_images/image_"+cF+ ".jpg";
        // std::cout << "saving1..." << endl;
        write_img(img_web,filename, 0);

        // save depth image data
        filename = ros::package::getPath("capturedata")+"/data/depth_images/depth_image_"+cF+ ".jpg";
        // std::cout << "saving1..." << endl;
        write_img(img_depth,filename, 1);

        // save tactile sensor depth image data
        filename = ros::package::getPath("capturedata")+"/data/touch_images_depth/touch_image_depth"+cF+ ".jpg";
        // std::cout << "saving1..." << endl;
        write_img(img_tact_depth,filename, 2);

        std::cout << "counting Flag: \t" << countingFlag << std::endl;


        countingFlag++;
        rwFlag = 0;

    }


	


}

// sensor_msgs::image_encodings::MONO16

void Save::write_img(const sensor_msgs::ImageConstPtr& image_msg, std::string filename, int depthFlag){

        // depthFlag = 0: normal img, depthFlag = 1: depth img

		std::string encoding;
        if (depthFlag == 0) encoding =  std::string("bgr8");
        else if (depthFlag == 1) encoding = std::string("16UC1");
        // else if (depthFlag == 1) encoding = std::string("mono8");
        // else if (depthFlag == 1) encoding = sensor_msgs::image_encodings::MONO8;
        // else if (depthFlag == 1) encoding = std::string("passthrough");
        else if (depthFlag == 2) encoding = std::string("32FC1");
	cv::Mat image;
	try
	{
		// std::cout << "saving2..." << endl;
        // if (depthFlag == 1) image = imgMsgToCv(image_msg, "passthrough")
        // if (depthFlag == 1) image = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        if (depthFlag == 1) image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        else image = cv_bridge::toCvShare(image_msg, encoding)->image;


        // image = cv_bridge::toCvShare(image_msg, encoding)->image;


		// std::cout << "saving3..." << endl;

	} catch(cv_bridge::Exception)
	{
		ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
		return;
	}



if (!image.empty()) {

    if (depthFlag == 1) {
        cv::convertScaleAbs(image, image, 0.03);
        // cv::applyColorMap(image, image, cv::COLORMAP_JET);

    	cv::imwrite(filename, image);

        // cv_bridge::CvImagePtr depthImg = cv_bridge::toCvCopy(image_msg , sensor_msgs::image_encodings::TYPE_16UC1);

        // //Normalize the pixel value
        // cv::normalize(depthImg->image, depthImg->image, 1, 0, cv::NORM_MINMAX);
        // cv::imwrite(filename, depthImg->image);
   
    }
    else {
    	cv::imwrite(filename, image);
    }
	
    cv::imwrite(filename, image);
    

	ROS_INFO("Saved image %s", filename.c_str());
}
	else {
		ROS_WARN("Couldn't save image, no data!");
		return;
	}

}


void Save::write_pose(const geometry_msgs::PoseStamped::ConstPtr& sensorPose,std::string filename, std::string ss)
    {

    std::fstream myFile;


    myFile.open(filename, std::fstream::out | std::fstream::app);

    std::string cF = std::to_string(countingFlag);

    std::cout << "Save pose in... "  << filename<< std::endl;


    myFile << cF<< "," << ss << ",";
    // myFile << cF<< "," ;


    myFile << sensorPose->pose.position.x << "," << sensorPose->pose.position.y << "," << sensorPose->pose.position.z << "," ;
    myFile << sensorPose->pose.orientation.x << "," << sensorPose->pose.orientation.y << "," ;
    myFile << sensorPose->pose.orientation.z  << "," << sensorPose->pose.orientation.w  << "\n" ;



    myFile.close();



}

/// No subscribe ftn for realsense camera for now!! we don't have to use ptcloud and icp,
/// but using apriltag we can get the pose of the object
/// !!!
///
void Save::write_frames(std::string filename){

    std::fstream myFile;

    filename = ros::package::getPath("capturedata")+"/data/camera_pose.csv";


    myFile.open(filename, std::fstream::out | std::fstream::app);

    std::string cF = std::to_string(countingFlag);

    std::cout << "Save pose first... "  << std::endl;


    myFile << cF<< ",";

    myFile << sensorPose.pose.position.x << "," << sensorPose.pose.position.y << "," << sensorPose.pose.position.z << "," ;
    myFile << sensorPose.pose.orientation.x << "," << sensorPose.pose.orientation.y << "," ;
    myFile << sensorPose.pose.orientation.z  << "," << sensorPose.pose.orientation.w  << ",";




    std::cout << "Next, save object pose just in case... "  << std::endl;


    myFile << objectPose.pose.position.x << "," << objectPose.pose.position.y << "," << objectPose.pose.position.z << "," ;
    myFile << objectPose.pose.orientation.x << "," << objectPose.pose.orientation.y << "," ;
    myFile << objectPose.pose.orientation.z  << "," << objectPose.pose.orientation.w  << ",";

    std::cout << "Next, save touch sensor pose ... "  << std::endl;



int touch_detected_flag = 0;
 try{
    current_tact_transform = tfBuffer.lookupTransform("map", "tactSensor_1", ros::Time(0),ros::Duration(0.7));

 }
 catch (tf2::TransformException &ex) {
   ROS_WARN("%s",ex.what());
   touch_detected_flag = 1;
 }

    if (touch_detected_flag == 0){
        Eigen::Isometry3d transform_matrix = tf2::transformToEigen(current_tact_transform);
        Eigen::Matrix3d m = transform_matrix.rotation();
        Eigen::Quaterniond q;
        q=m;
        Eigen::Vector3d v = transform_matrix.translation();


        std::cout << "tf from realsense camera to tag_0 Rotation: " << std::endl << m << std::endl;
    //    std::cout << "Rotation: " << std::endl << q.x()<<"dd"<<q.y() << std::endl;
        std::cout << "Translation: " << std::endl << v << std::endl;


        myFile << v.x() << "," << v.y() << "," << v.z() << "," ;
        myFile << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n" ;
        }






    myFile.close();



}

void Save::gettfframe_object(std::string filename){

    std::fstream myFile;

    filename = ros::package::getPath("capturedata")+"/data_tf/data_tf_apriltag.csv";


    myFile.open(filename, std::fstream::out | std::fstream::app);

    std::string cF = std::to_string(countingFlag_tf);

//    geometry_msgs::TransformStamped 	eigenToTransform (const Eigen::Isometry3d &T)

int tag_detected_flag = 0;
int tag1_detected_flag = 0;
 try{
    current_tag0_transform = tfBuffer.lookupTransform("camera_color_optical_frame", "tag_0", ros::Time(0),ros::Duration(0.7));

 }
 catch (tf2::TransformException &ex) {
   ROS_WARN("%s",ex.what());
   tag_detected_flag = 1;
 }

    if (tag_detected_flag == 0){
        Eigen::Isometry3d transform_matrix = tf2::transformToEigen(current_tag0_transform);
        Eigen::Matrix3d m = transform_matrix.rotation();
        Eigen::Quaterniond q;
        q=m;
        Eigen::Vector3d v = transform_matrix.translation();


        std::cout << "tf from realsense camera to tag_0 Rotation: " << std::endl << m << std::endl;
    //    std::cout << "Rotation: " << std::endl << q.x()<<"dd"<<q.y() << std::endl;
        std::cout << "Translation: " << std::endl << v << std::endl;

        myFile << cF<< ",";

        myFile << v.x() << "," << v.y() << "," << v.z() << "," ;
        myFile << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "," ;
        }

    try{
       current_tag1_transform = tfBuffer.lookupTransform("camera_color_optical_frame", "tag_1", ros::Time(0),ros::Duration(0.7));

    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      tag1_detected_flag = 1;
    }

    if (tag1_detected_flag == 0){
        Eigen::Isometry3d transform_matrix = tf2::transformToEigen(current_tag1_transform);
        Eigen::Matrix3d m = transform_matrix.rotation();
        Eigen::Quaterniond q;
        q=m;
        Eigen::Vector3d v = transform_matrix.translation();


        std::cout << "tf from realsense camera to tag_1 Rotation: " << std::endl << m << std::endl;
    //    std::cout << "Rotation: " << std::endl << q.x()<<"dd"<<q.y() << std::endl;
        std::cout << "Translation: " << std::endl << v << std::endl;


        myFile << v.x() << "," << v.y() << "," << v.z() << "," ;
        myFile << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n" ;
        }


    myFile.close();
    
    
    
    countingFlag_tf +=1;

}


void Save::copyPose(geometry_msgs::PoseStamped &in, geometry_msgs::PoseStamped &out){

    out.header.stamp = in.header.stamp;
    out.header.frame_id = in.header.frame_id;
    out.pose.position = in.pose.position ;
    out.pose.orientation = in.pose.orientation;


}




void Save::pose_object_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

    objectPose_temp.header.stamp = msg->header.stamp;
    objectPose_temp.header.frame_id = msg->header.frame_id;
    objectPose_temp.pose.position = msg->pose.position ;
    objectPose_temp.pose.orientation = msg->pose.orientation;

    copyPose(objectPose_temp, objectPose);


}



void Save::pose_sensor_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg){


    sensorPose_temp.header.stamp = msg->header.stamp;
    sensorPose_temp.header.frame_id = msg->header.frame_id;
    sensorPose_temp.pose.position = msg->pose.position ;
    sensorPose_temp.pose.orientation = msg->pose.orientation;

    copyPose(sensorPose_temp, sensorPose);


}

