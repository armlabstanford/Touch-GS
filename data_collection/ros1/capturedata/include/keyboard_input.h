
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
//keyboard input
#include <iostream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>



//tf
#include "tf/transform_listener.h"
#include <math.h>

// for marker
#include <visualization_msgs/Marker.h>


using namespace std;


// http://nehe.gamedev.net/article/msdn_virtualkey_codes/15009/
#define VK_SPACE 0x20
#define VK_LSHIFT 0xA0


#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42

#define KEYCODE_0	0x30
#define KEYCODE_1	0x31
#define KEYCODE_2	0x32
#define KEYCODE_3	0x33
#define KEYCODE_4	0x34
#define KEYCODE_5	0x35
#define KEYCODE_6	0x36
#define KEYCODE_7	0x37
#define KEYCODE_8	0x38
#define KEYCODE_9	0x39

#define KEYCODE_NUMPAD0	0x60
#define KEYCODE_NUMPAD1	0x61
#define KEYCODE_NUMPAD2	0x62
#define KEYCODE_NUMPAD3	0x63
#define KEYCODE_NUMPAD4	0x64
#define KEYCODE_NUMPAD5	0x65
#define KEYCODE_NUMPAD6	0x66
#define KEYCODE_NUMPAD7	0x67
#define KEYCODE_NUMPAD8	0x68
#define KEYCODE_NUMPAD9	0x69

#define KEYCODE_A	0x41
#define KEYCODE_B	0x42
#define KEYCODE_C	0x43
#define KEYCODE_D	0x44
#define KEYCODE_E	0x45
#define KEYCODE_F	0x46
#define KEYCODE_G	0x47
#define KEYCODE_H	0x48
#define KEYCODE_I	0x49
#define KEYCODE_J	0x4A
#define KEYCODE_K	0x4B
#define KEYCODE_L	0x4C
#define KEYCODE_M	0x4D
#define KEYCODE_N	0x4E
#define KEYCODE_O	0x4F
#define KEYCODE_P	0x50
#define KEYCODE_Q	0x51
#define KEYCODE_R	0x52
#define KEYCODE_S	0x53
#define KEYCODE_T	0x54
#define KEYCODE_U	0x55
#define KEYCODE_V	0x56
#define KEYCODE_W	0x57
#define KEYCODE_X	0x58
#define KEYCODE_Y	0x59
#define KEYCODE_Z	0x5A

#define KEYCODE_a	0x61
#define KEYCODE_b	0x62
#define KEYCODE_c	0x63
#define KEYCODE_d	0x64
#define KEYCODE_e	0x65
#define KEYCODE_f	0x66
#define KEYCODE_g	0x67
#define KEYCODE_h	0x68
#define KEYCODE_i	0x69
#define KEYCODE_j	0x6A
#define KEYCODE_k	0x6B
#define KEYCODE_l	0x6C
#define KEYCODE_m	0x6D
#define KEYCODE_n	0x6E
#define KEYCODE_o	0x6F
#define KEYCODE_p	0x70
#define KEYCODE_q	0x71
#define KEYCODE_r	0x72
#define KEYCODE_s	0x73
#define KEYCODE_t	0x74
#define KEYCODE_u	0x75
#define KEYCODE_v	0x76
#define KEYCODE_w	0x77
#define KEYCODE_x	0x78
#define KEYCODE_y	0x79
#define KEYCODE_z	0x7A

class Calibration
{
public:
 Calibration(ros::NodeHandle nh);
 ~Calibration();

 void Initialize();
 void Update();
 void KeyboardChecking();

private:
 ros::NodeHandle n;

  // Transform listener
  tf::TransformListener* listener;
  tf::StampedTransform pico0_to_world;                // Transform from pico0 frame to world frame
  tf::StampedTransform pico1_to_world;                // Transform from pico0 frame to world frame
  tf::StampedTransform pico2_to_world;                // Transform from pico0 frame to world frame


 static const int MAX_COUNT = 5;
 static const int MAX_WAIT = 1000;
 int count;
 int wait;
// sensor_msgs::PointCloud2 result_pointcl/*oud;
// sensor_msgs::PointCloud2 msg_temp;

// void point0_Callback(const sensor_msgs::PointCloud2ConstPtr& msg);*/

 ros::Subscriber sub0;
 ros::Subscriber sub1;
 ros::Subscriber sub2;

 ros::Publisher pub;
 // publisher for markers
 ros::Publisher vis_pub;


// Eigen::Matrix4f transformation;

// marker-related
 visualization_msgs::Marker marker;
 uint32_t shape = visualization_msgs::Marker::SPHERE;

};
