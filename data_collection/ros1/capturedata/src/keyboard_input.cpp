

#include "keyboard_input.h"


int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "capture_keyboard");
  ros::NodeHandle n;
  Calibration check(n);
	signal(SIGINT,quit);
  // AsyncSpinner instead of spin()

  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  check.Initialize();

// keep the loop at 100Hz
  // ros::Rate loop_rate(100);

	check.KeyboardChecking();

	// while(ros::ok())
	// {
	//   check.Update();
	//   loop_rate.sleep();
  //   //ros::Time curr = ros::Time::now();
  //   //ROS_INFO_STREAM("Loop time: " <<  curr.toSec()-last.toSec());
  //   //last = curr;
	// }

//
return 0;

    }

Calibration::Calibration(ros::NodeHandle nh) :
 n(nh),
 count(MAX_COUNT),
 wait(MAX_WAIT)
{
}
Calibration::~Calibration()
{
  delete listener;
}


//void Calibration::point0_Callback(const sensor_msgs::PointCloud2ConstPtr& msg){



//  // Look up transform from ft to world frame
//  tf::StampedTransform tempTransform;
//  try
//  {
//		// listener->waitForTransform("/world_view", "/tf_0_optical_frame", ros::Time(0), ros::Duration(1.0));
//    // listener->lookupTransform("/world_view", "/tf_0_optical_frame", ros::Time(0), tempTransform);
//		// listener->waitForTransform("/r0/camera", "/tag_0", ros::Time(0), ros::Duration(1.0));
//    // listener->lookupTransform("/r0/camera", "/tag_0", ros::Time(0), tempTransform);
//		listener->waitForTransform( "/tag_0","/r0/camera", ros::Time(0), ros::Duration(1.0));
//		listener->lookupTransform( "/tag_0","/r0/camera", ros::Time(0), tempTransform);


//  }
//  catch (tf::TransformException ex)
//  {
//    ROS_ERROR("%s",ex.what());
//  }

//  // Set translation to zero before updating value
//  //tempTransform.setOrigin(tf::Vector3(0.0,0.0,0.0));
//// tempTransform.inverse();
//  pico0_to_world = tempTransform;


//}


void Calibration::Initialize(){

  //Listen to the transfomation from the ft sensor to world frame.
  // listener = new tf::TransformListener(ros::Duration(300));
	// sub0 = n.subscribe<sensor_msgs::PointCloud2>("/r0/pico_flexx/pointstf",100, &Calibration::point0_Callback, this);
  // pub = n.advertise<sensor_msgs::PointCloud2>("pico/points", 1000);

    pub = n.advertise<std_msgs::String>("/capturedata/lib_cmd", 500);

}


void Calibration::Update(){

 //
 // pub.publish(result_pointcloud);

  ros::spinOnce();


}


///////////////////////////////////////////// check the keyboard input
void Calibration::KeyboardChecking(){


	char keyInput;
	bool publishState=false;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

sleep(2);
std::cout << std::endl;
std::cout << " -----------------------------------------------------------------------------" << std::endl;
	std::cout << "  Use the keyboard to send Experiment commands on Tactile sensor" << std::endl;
	std::cout << " -----------------------------------------------------------------------------" << std::endl;

    std::cout << "\tCapture dataset:\t\t\t'G'" << std::endl;
    std::cout << "\tCapture object frame :\t\t\t'Q'" << std::endl;
    std::cout << "\t Get the Depth image and pointcloud data :\t\t'E'" << std::endl;
//	std::cout << "\tSave Data from tactile sensor:\t\t'R'" << std::endl;

//	std::cout << "\tinitialize adjustment of all picos  \t\t 'T'  " << std::endl;
//	std::cout << "\t save adjustment of all picos  \t\t 'Y'  " << std::endl;
//	std::cout << "\tadjust picoflexx 0:\t\t\t\t'U'" << std::endl;
//	std::cout << "\tadjust picoflexx 1:\t\t\t\t'I'" << std::endl;
//	std::cout << "\tadjust picoflexx 2:\t\t\t\t'O'" << std::endl;
//	std::cout << "\tCommand description display:\t\t\t\t'P'" << std::endl;
//	std::cout << "\tSave Image data and Depth data:\t\t\t\t'G'" << std::endl;

//	std::cout << "\t-/+ x: 'z'/'c',\t-/+ y: 'x'/'s',\t-/+ z: 'v'/'f', \t  " << std::endl;


std::cout << " -----------------------------------------------------------------------------" << std::endl;


for(;;)
 {

 std_msgs::String msg;
	 std::stringstream ss;

	 // get the next event from the keyboard
	 if(read(kfd, &keyInput, 1) < 0)
	 {
		 perror("read():");
		 exit(-1);
	 }

	 //linear_=angular_=0;
	 ROS_DEBUG("value: 0x%02X\n", keyInput);
	 printf("value: 0x%02X\n", keyInput);

 switch(keyInput)
	 {
 	 		 case KEYCODE_q:
                 ROS_DEBUG("Q: Capture dataset");
				 //cmd_ = 4;
				 ss << "calibstart";
				 publishState = true;
				 break;
			 case KEYCODE_w:
				 ROS_DEBUG("w: save calibration");
				 //cmd_ = 4;
				 ss << "calibsave";
				 publishState = true;
				 break;
			 case KEYCODE_e:
                 ROS_DEBUG("e: get the Depth image and pointcloud data");
				 //cmd_ = 1;
				 ss << "calibend";
				 publishState = true;
				 break;

			 case KEYCODE_d:
				 ROS_DEBUG("d: reset icp calibration value");
				 //cmd_ = 1;
				 ss << "reseticp";
				 publishState = true;
				 break;

			 case KEYCODE_r:
				 ROS_DEBUG("r: Save Data from tactile sensor");
				 //cmd_ = 1;
				 ss << "tactsave";
				 publishState = true;
				 break;


			 case KEYCODE_u:
				 ROS_DEBUG("u: adjust picoflexx 0");
				 //cmd_ = 1;
				 ss << "pico0calib";
				 publishState = true;
				 break;
			 case KEYCODE_i:
				 ROS_DEBUG("i: adjust picoflexx 1");
				 //cmd_ = 1;
				 ss << "pico1calib";
				 publishState = true;
				 break;
			 case KEYCODE_o:
				 ROS_DEBUG("o: adjust picoflexx 2");
				 //cmd_ = 1;
				 ss << "pico2calib";
				 publishState = true;
				 break;
			 case KEYCODE_g:
				 ROS_DEBUG("g: save camera and depth data");
				 //cmd_ = 1;
				 ss << "saveall";
				 publishState = true;
				 break;


			 case KEYCODE_z:
				 ROS_DEBUG("z: adjust -x");
				 //cmd_ = 1;
				 ss << "-x";
				 publishState = true;
				 break;
			 case KEYCODE_c:
				 ROS_DEBUG("c: adjust +x");
				 //cmd_ = 1;
                 ss << "+x";
				 publishState = true;
				 break;
			 case KEYCODE_x:
				 ROS_DEBUG("x: adjust -y");
				 //cmd_ = 1;
				 ss << "-y";
				 publishState = true;
				 break;

			 case KEYCODE_s:
				 ROS_DEBUG("s: adjust +y");
				 //cmd_ = 1;
				 ss << "+y";
				 publishState = true;
				 break;
			 case KEYCODE_v:
				 ROS_DEBUG("v: adjust -z");
				 //cmd_ = 1;
				 ss << "-z";
				 publishState = true;
				 break;
			 case KEYCODE_f:
				 ROS_DEBUG("f: adjust +z");
				 //cmd_ = 1;
				 ss << "+z";
				 publishState = true;
				 break;

			 case KEYCODE_t:
				 ROS_DEBUG("t: initialize pico i's x/y/z coordinate adjustment");
				 //cmd_ = 1;
				 ss << "init_adjust";
				 publishState = true;
				 break;

			 case KEYCODE_y:
				 ROS_DEBUG("y: save all pico's adjustment");
				 //cmd_ = 1;
				 ss << "save_adjust";
				 publishState = true;
				 break;

			case KEYCODE_1:
				ROS_DEBUG("1: save the first pico's pointcloud for icp");
				ss << "icp_pc0";
				publishState = true;

				break;

			case KEYCODE_2:
				ROS_DEBUG("2: save the second pico's pointcloud for icp");
				ss << "icp_pc1";
				publishState = true;

				break;

			case KEYCODE_3:
				ROS_DEBUG("1: save the third pico's pointcloud for icp");
				ss << "icp_pc2";
				publishState = true;

				break;




			 case KEYCODE_p:
				 ROS_DEBUG("p: Display the command again");
                 std::cout << std::endl;
                 std::cout << " -----------------------------------------------------------------------------" << std::endl;
                     std::cout << "  Use the keyboard to send Experiment commands on Tactile sensor" << std::endl;
                     std::cout << " -----------------------------------------------------------------------------" << std::endl;

                     std::cout << "\tCapture dataset:\t\t\t'G'" << std::endl;
                     std::cout << "\tCapture object frame :\t\t\t'Q'" << std::endl;
                    std::cout << "\t Get the Depth image and pointcloud data :\t\t'E'" << std::endl;
                 //	std::cout << "\tSave Data from tactile sensor:\t\t'R'" << std::endl;

                 //	std::cout << "\tinitialize adjustment of all picos  \t\t 'T'  " << std::endl;
                 //	std::cout << "\t save adjustment of all picos  \t\t 'Y'  " << std::endl;
                 //	std::cout << "\tadjust picoflexx 0:\t\t\t\t'U'" << std::endl;
                 //	std::cout << "\tadjust picoflexx 1:\t\t\t\t'I'" << std::endl;
                 //	std::cout << "\tadjust picoflexx 2:\t\t\t\t'O'" << std::endl;
                 //	std::cout << "\tCommand description display:\t\t\t\t'P'" << std::endl;
                 //	std::cout << "\tSave Image data and Depth data:\t\t\t\t'G'" << std::endl;

                 //	std::cout << "\t-/+ x: 'z'/'c',\t-/+ y: 'x'/'s',\t-/+ z: 'v'/'f', \t  " << std::endl;


                 std::cout << " -----------------------------------------------------------------------------" << std::endl;


				 publishState = false;
				 break;

}


	 if(publishState ==true)
	 {
		 msg.data = ss.str();
		 ROS_INFO("%s", msg.data.c_str());
		 pub.publish(msg);
		 publishState=false;
	 }

 }




 return;
}
