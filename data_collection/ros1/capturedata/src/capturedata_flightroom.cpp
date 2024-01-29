
////////////////////////////////////////
// 		 code for saving tf / image(from realsense and tactile sensor)		//
////////////////////////////////////////


#include "capturedata_flightroom.h"
std::string  lib_cmd;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "capturedata_flightroom");
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


void Save::cmd_Callback(const std_msgs::String::ConstPtr& msg){

	ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());
	keyboardCom = msg->data.c_str();

	if (keyboardCom.compare("saveall") == 0)
	{
		std::cout << "\tSave image and depth/force data" << std::endl;
		test_1 = "abc";
		std_msgs::String abc;
		std::stringstream ss;
	  ss << "abcd" << count;
	  abc.data = ss.str();


		pub.publish(abc);
        //don't have to pause little bit to sync between arduino and saving code
        //ros::Duration(0.18).sleep();

        // img callback and position dataset follows this flag.
		rwFlag = 1;

        //pause little bit to sync between arduino and saving code
		// ros::Duration(0.5).sleep();
//		pcFlag0 = 1;

		// std::cout<< "frame id: "<<saved_pico0.frame_id_<< <<std::endl;
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


void Save::Initialize(){

  //Listen to the transfomation from the ft sensor to world frame.
	// listener0 = new tf::TransformListener(ros::Duration(300));

	// sub1 = nh.subscribe("/pico/lib_cmd", 1, &Calibration::cmd_Callback);


    //Estabilish the transformer
    tfListener.reset(new tf2_ros::TransformListener(tfBuffer));


    sub0 = n.subscribe<std_msgs::String>("/capturedata/lib_cmd",100, &Save::cmd_Callback, this);


//    std::string topic = n.resolveName("/cv_camera_no_yaml/image_raw");
//    std::string topic = n.resolveName("/RunCamera/image_raw_1");
    std::string topic = n.resolveName("/RunCamera/image_raw_1");
    sub_image = it.subscribe(topic, 1, boost::bind(&Save::callbackTact, this, _1));



    std::string topic2 = n.resolveName("/camera/color/image_raw");
//    std::string topic2 = n.resolveName("/RunCamera/image_raw_2");
    sub_image_realsense = it.subscribe(topic2, 1, boost::bind(&Save::callbackImageRealSense, this, _1));

	// sub_image = it.subscribe(topic, 1, &Save::callbackImage, this);

	// pointcloud subscriber

//    sub_wrench = n.subscribe<geometry_msgs::WrenchStamped>("/atichecking/force_wrench",100, &Save::wrench_Callback, this);
    sub_pose_object = n.subscribe<geometry_msgs::PoseStamped>("/BaseFrame/mavros/vision_pose/pose",100, &Save::pose_object_Callback, this);
    sub_pose_sensor = n.subscribe<geometry_msgs::PoseStamped>("/SensorModule/mavros/vision_pose/pose",100, &Save::pose_sensor_Callback, this);


    //  // pub = n.advertise<sensor_msgs::PointCloud2>("pico/points", 1000);

}


void Save::Update(){

  ros::spinOnce();
}



void Save::write_img(const sensor_msgs::ImageConstPtr& image_msg, std::string filename){
		std::string encoding;
		encoding =  std::string("bgr8");

	cv::Mat image;
	try
	{
		// std::cout << "saving2..." << endl;

		image = cv_bridge::toCvShare(image_msg, encoding)->image;

		// std::cout << "saving3..." << endl;

	} catch(cv_bridge::Exception)
	{
		ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
		return;
	}
if (!image.empty()) {
	cv::imwrite(filename, image);
	ROS_INFO("Saved image %s", filename.c_str());
}
	else {
		ROS_WARN("Couldn't save image, no data!");
		return;
	}

}

/// No subscribe ftn for realsense camera for now!! we don't have to use ptcloud and icp,
/// but using apriltag we can get the pose of the object
/// !!!
///
void Save::callbackTact(const sensor_msgs::ImageConstPtr& image_msg)
{

	// saving flag priority:
	//  1. request by service.
	//  2. request by topic about start and end.
	//  3. flag 'save_all_image'.
	// save the image
if(rwFlag==1){
	// saveImage(image_msg, filename);
    // doesn't matter, we already ot the image when this callbackTact starts.
	// if (imgCountingFlag==0) {ros::Duration(0.01).sleep();}
	std::string filename;
// filename = (g_format % count_ % "jpg").str();
	std::string cF = std::to_string(countingFlag);

// remove img counting flag




std::cout << "saving..." << endl;
    ros::Duration(0.04).sleep();
    imgCountingFlag++;
//	if(imgCountingFlag==25) realSenseFlag = 1;


    // save
    // 0. tactile image
    // 1. pose and force from robot!
    // 2. realsense image
    // 3.
    if (imgCountingFlag==1){
        filename = ros::package::getPath("capturedata")+"/data_img/Img_"+cF+ ".jpg";
        write_img(image_msg,filename);
    }

    if(imgCountingFlag==2) {
        realSenseFlag = 1;
//        img2CountingFlag = 1;

        std::string filename;
	}

    if(imgCountingFlag == 3){

        tf_objectFlag = 1;
         std::cout << "save sensor and object frame from OptiTrack..." << endl;

        write_frames("void");

    }

    if(imgCountingFlag == 4){

        tf_objectFlag = 2;
        std::cout << "save apriltag frame..." << endl;

        gettfframe_object("void");

    }

    if (imgCountingFlag == 6) {
		imgCountingFlag = 0;
		countingFlag++;
		rwFlag =0;
        tfFlag = 0;
	}
}


// collect tf frame of object frame from detected aruco frame

//if(tf_objectFlag == 1){

//    gettfframe_object("void");

//    tf_objectFlag = 0;
//}
//if(tf_objectFlag == 2){

////    reset_tfframe_object("void");

//    tf_objectFlag = 0;
//}
//if(tf_objectFlag == 3){

////    get_tfframe_btw_tact_object("void");

//    tf_objectFlag = 0;
//}


}

void Save::callbackTact2(const sensor_msgs::ImageConstPtr& image_msg)
{

    // saving flag priority:
    //  1. request by service.
    //  2. request by topic about start and end.
    //  3. flag 'save_all_image'.
    // save the image
    // saveImage(image_msg, filename);
    // doesn't matter, we already ot the image when this callbackimage starts.
    // if (imgCountingFlag==0) {ros::Duration(0.01).sleep();}
    if (img2CountingFlag == 1){
        std::string filename;
        std::string cF = std::to_string(countingFlag);

        filename = ros::package::getPath("capturedata")+"/data_img/SecondImg_"+cF+ ".jpg";
        // std::cout << "saving1..." << endl;
        write_img(image_msg,filename);
        img2CountingFlag = 0;
    }


}

void Save::callbackImageRealSense(const sensor_msgs::ImageConstPtr& image_msg)
{

	// saving flag priority:
	//  1. request by service.
	//  2. request by topic about start and end.
    //  3. flag 'save_all_image'.
	// save the image
	// saveImage(image_msg, filename);
	// doesn't matter, we already ot the image when this callbackimage starts.
	// if (imgCountingFlag==0) {ros::Duration(0.01).sleep();}
    if (realSenseFlag == 1){
        std::string filename;
        std::string cF = std::to_string(countingFlag);

        filename = ros::package::getPath("capturedata")+"/data_img/RealsenseImg_"+cF+ ".jpg";
        // std::cout << "saving1..." << endl;
        write_img(image_msg,filename);
        realSenseFlag = 0;
    }


}

void Save::write_frames(std::string filename){

    std::fstream myFile;

    filename = ros::package::getPath("capturedata")+"/data_tf/data_tf.csv";

    myFile.open(filename, std::fstream::out | std::fstream::app);

    std::string cF = std::to_string(countingFlag);

    std::cout << "Save sensor pose first... "  << std::endl;


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

