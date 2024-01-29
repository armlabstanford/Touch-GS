
////////////////////////////////////////
// 		 code for saving tf / image(from realsense and tactile sensor)		//
////////////////////////////////////////


#include "capturedata_allegro.h"
std::string  lib_cmd;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "capturedata_allegro");
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
        tf_objectFlag= 1;
    }
    else if (keyboardCom.compare("calibsave") == 0){
         std::cout << "open and get the average of the object transform and save it" << std::endl;
         tf_objectFlag = 2;

    }
    else if (keyboardCom.compare("calibend") == 0){
         std::cout << "get the tf matrix from the tactile sensor frame to tag1(attached in upper side of the cube)" << std::endl;
         tf_objectFlag = 3;

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

    std::string topic22 = n.resolveName("/RunCamera/image_raw_2");
    sub_image2 = it.subscribe(topic22, 1, boost::bind(&Save::callbackTact2, this, _1));


    std::string topic2 = n.resolveName("/camera/color/image_raw");
//    std::string topic2 = n.resolveName("/RunCamera/image_raw_2");
    sub_image_realsense = it.subscribe(topic2, 1, boost::bind(&Save::callbackImageRealSense, this, _1));

	// sub_image = it.subscribe(topic, 1, &Save::callbackImage, this);

	// pointcloud subscriber

//    sub_wrench = n.subscribe<geometry_msgs::WrenchStamped>("/atichecking/force_wrench",100, &Save::wrench_Callback, this);
//	sub_pose = n.subscribe<geometry_msgs::PoseStamped>("/atichecking/EEpose",100, &Save::pose_Callback, this);
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
void Save::point0_Callback(const sensor_msgs::PointCloud2ConstPtr& cloud){

if(rwFlag==1 && pcFlag0 == 1 ){
    std::string prefix = ros::package::getPath("capturedata")+"/allegro_tf/Pointcloud0_";
    if ((cloud->width * cloud->height) == 0)
        return;

    ROS_INFO ("Received %d data points in frame %s with the following fields: %s",
                        (int)cloud->width * cloud->height,
                        cloud->header.frame_id.c_str (),
                        pcl::getFieldsList (*cloud).c_str ());

    std::stringstream ss;
    ss << prefix << std::to_string(countingFlag) << ".pcd";
    ROS_INFO ("Data saved to %s", ss.str ().c_str ());

    pcl::io::savePCDFile (ss.str (), *cloud, Eigen::Vector4f::Zero (),
                                                Eigen::Quaternionf::Identity (), false);

pcFlag0 = 0;
}

}
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
    ros::Duration(0.1).sleep();
    imgCountingFlag++;
//	if(imgCountingFlag==25) realSenseFlag = 1;


    // save
    // 0. tactile image
    // 1. pose and force from robot!
    // 2. realsense image
    // 3.
    if (imgCountingFlag==1){
        filename = ros::package::getPath("capturedata")+"/allegro_img/Img_"+cF+ ".jpg";
        write_img(image_msg,filename);
    }

    if(imgCountingFlag==2) {
        realSenseFlag = 1;
        img2CountingFlag = 1;

        std::string filename;
	}

    if(imgCountingFlag == 6){

        pcFlag0 = 1;
        gettfframe("void");

    }



    if (imgCountingFlag == 10) {
		imgCountingFlag = 0;
		countingFlag++;
		rwFlag =0;
        tfFlag = 0;
	}
}


// collect tf frame of object frame from detected aruco frame

if(tf_objectFlag == 1){

    gettfframe_object("void");

    tf_objectFlag = 0;
}
if(tf_objectFlag == 2){

//    reset_tfframe_object("void");

    tf_objectFlag = 0;
}
if(tf_objectFlag == 3){

//    get_tfframe_btw_tact_object("void");

    tf_objectFlag = 0;
}


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

        filename = ros::package::getPath("capturedata")+"/allegro_img/SecondImg_"+cF+ ".jpg";
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

        filename = ros::package::getPath("capturedata")+"/allegro_img/RealsenseImg_"+cF+ ".jpg";
        // std::cout << "saving1..." << endl;
        write_img(image_msg,filename);
        realSenseFlag = 0;
    }


}

void Save::gettfframe(std::string filename){

    std::fstream myFile;

    filename = ros::package::getPath("capturedata")+"/allegro_tf/data_tf.csv";

    myFile.open(filename, std::fstream::out | std::fstream::app);

    std::string cF = std::to_string(countingFlag);



    current_tact_transform = tfBuffer.lookupTransform("palm_link", "link_3_tip", ros::Time(0),ros::Duration(0.7));
    current_realsense_transform = tfBuffer.lookupTransform("palm_link", "link_15_tip", ros::Time(0),ros::Duration(0.7));

    //    Eigen::Affine3d transform_matrix = tf2::transformToEigen(transform);
    Eigen::Isometry3d transform_matrix = tf2::transformToEigen(current_tact_transform);
    Eigen::Matrix3d m = transform_matrix.rotation();
    Eigen::Quaterniond q;
    q=m;
    Eigen::Vector3d v = transform_matrix.translation();



    std::cout << "link3(index finger) Rotation: " << std::endl << m << std::endl;
//    std::cout << "Rotation: " << std::endl << q.x()<<"dd"<<q.y() << std::endl;
    std::cout << "Translation: " << std::endl << v << std::endl;

    myFile << cF<< ",";

    myFile << v.x() << "," << v.y() << "," << v.z() << "," ;
    myFile << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "," ;

    Eigen::Isometry3d transform_matrix_cam = tf2::transformToEigen(current_realsense_transform);
    Eigen::Matrix3d m1 = transform_matrix_cam.rotation();
    Eigen::Quaterniond q1;
    q1=m1;
    Eigen::Vector3d v1 = transform_matrix_cam.translation();

    std::cout << "link15(thumb finger) Rotation: " << std::endl << m1 << std::endl;
    std::cout << "Translation: " << std::endl << v1 << std::endl;


    myFile << v1.x() << "," << v1.y() << "," << v1.z() << "," ;
    myFile << q1.x() << "," << q1.y() << "," << q1.z() << "," << q1.w() << "\n" ;


    myFile.close();

}

void Save::gettfframe_object(std::string filename){

    std::fstream myFile;

    filename = ros::package::getPath("capturedata")+"/allegro_tf/data_tf_object.csv";


    myFile.open(filename, std::fstream::out | std::fstream::app);

    std::string cF = std::to_string(countingFlag_tf);

//    geometry_msgs::TransformStamped 	eigenToTransform (const Eigen::Isometry3d &T)

int tag_detected_flag = 0;
int tag1_detected_flag = 0;
 try{
    current_tag0_transform = tfBuffer.lookupTransform("camera_color_optical_frame", "tag_3", ros::Time(0),ros::Duration(0.7));

 }
 catch (tf2::TransformException &ex) {
   ROS_WARN("%s",ex.what());
   tag_detected_flag = 1;
 }

try{
   current_tag1_transform = tfBuffer.lookupTransform("camera_color_optical_frame", "tag_0", ros::Time(0),ros::Duration(0.7));

}
catch (tf2::TransformException &ex) {
  ROS_WARN("%s",ex.what());
  tag1_detected_flag = 1;
}


    // current_aruco_transform = tfBuffer.lookupTransform("panda_link0", "tag_0", ros::Time(0),ros::Duration(0.7));

    if (tag_detected_flag == 0){
        Eigen::Isometry3d transform_matrix = tf2::transformToEigen(current_tag0_transform);
        Eigen::Matrix3d m = transform_matrix.rotation();
        Eigen::Quaterniond q;
        q=m;
        Eigen::Vector3d v = transform_matrix.translation();


        std::cout << "tf from realsense camera to tag_3 Rotation: " << std::endl << m << std::endl;
    //    std::cout << "Rotation: " << std::endl << q.x()<<"dd"<<q.y() << std::endl;
        std::cout << "Translation: " << std::endl << v << std::endl;

        myFile << cF<< " ";

        myFile << v.x() << " " << v.y() << " " << v.z() << " " ;
        myFile << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " ;
        }
    else if(tag_detected_flag ==1 ){
        std::cout << "tag3 not detected, saving default 0 value.. "  << std::endl;

        myFile << cF<< " ";

        myFile << 0 << " " << 0 << " " << 0 << " " ;
        myFile << 0 << " " << 0 << " " << 0 << " " << 1 << " " ;

    }


    if (tag_detected_flag == 0){
        Eigen::Isometry3d transform_matrix = tf2::transformToEigen(current_tag1_transform);
        Eigen::Matrix3d m = transform_matrix.rotation();
        Eigen::Quaterniond q;
        q=m;
        Eigen::Vector3d v = transform_matrix.translation();


        std::cout << "tf from realsense camera to tag_0 Rotation: " << std::endl << m << std::endl;
    //    std::cout << "Rotation: " << std::endl << q.x()<<"dd"<<q.y() << std::endl;
        std::cout << "Translation: " << std::endl << v << std::endl;



        myFile << v.x() << " " << v.y() << " " << v.z() << " " ;
        myFile << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n" ;
        }
    else if(tag_detected_flag ==1 ){
        std::cout << "tag0 not detected, saving default 0 value.. "  << std::endl;


        myFile << 0 << " " << 0 << " " << 0 << " " ;
        myFile << 0 << " " << 0 << " " << 0 << " " << 1 << "\n" ;

    }



    myFile.close();
    
    
    
    countingFlag_tf +=1;

}



