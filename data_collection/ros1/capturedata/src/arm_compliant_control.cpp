/*
Copyright (c) <2018>, <Monroe Kennedy III>
* All rights reserved.
*/

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Header.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>

// #include <unsupported/Eigen/MatrixFunctions>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
:
#include <map> //dictionary equivalent
#include <cmath>
// #include <math.h>

#include <sensor_msgs/JointState.h>
#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/controller_manager.h>
#include <robot_controllers_interface/joint_handle.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

// #include <intelligent_coop_carry/DynamicalTerms.h>  //To publish full dynamics
#include <intelligent_coop_carry_msgs/DynamicalTerms.h>
#include <intelligent_coop_carry_msgs/Matrow.h>
#include <intelligent_coop_carry_msgs/GripperWrench.h>
#include <intelligent_coop_carry/SwitchController.h> //For controller switching service
#include <intelligent_coop_carry/math.h> //For controller switching service
#include <intelligent_coop_carry/oper_functions.h> //for angle operations and other math


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class CompliantControl
{
  public:
    CompliantControl(); //constructor
    //Functions
    void Jnt_state_callback(const sensor_msgs::JointStateConstPtr& msg); //Joints state callback
    Eigen::VectorXd Gravity_comp_func(const Eigen::VectorXd& gravity,const Eigen::VectorXd& q,const Eigen::VectorXd& qdot); //const means dont change this
    void Publish_arm_control(const Eigen::VectorXd& torque, const std_msgs::Header& header);
    void Publish_dynamical_terms(const std_msgs::Header& header, const Eigen::MatrixXd& inertia,const Eigen::MatrixXd& jacobian,const Eigen::VectorXd& coriolis,const Eigen::VectorXd& gravity,const std::vector<double>& q, const std::vector<double>& qdot, const std::vector<double>& effort);
    int load_required_controllers();
    //This function takes in a goal configuration, solves dynamics equation with for compliance about desired angle state, and returns torque to maintain that pose
    Eigen::VectorXd Maintain_compliant_pose(const Eigen::VectorXd& q_goal_config, const std_msgs::Header& header, const Eigen::MatrixXd& inertia,const Eigen::MatrixXd& jacobian,const Eigen::VectorXd& coriolis,const Eigen::VectorXd& gravity,const std::vector<double>& q, const std::vector<double>& qdot, const std::vector<double>& effort);
    Eigen::VectorXd  Current_goal_config(const std::map<std::string, double> q_goal_config_dict );

    Eigen::VectorXd Transform_pose_error_function(Eigen::Affine3d goal, geometry_msgs::TransformStamped current);

    void Publish_pose_error(const Eigen::VectorXd pose_error, const geometry_msgs::TransformStamped gripper_transform);

  private:
    Eigen::VectorXd convert_vector_for_TF(const Eigen::Affine3d transform_matrix, const Eigen::VectorXd& input_vect);
    ros::NodeHandle nh;
    ros::Publisher arm_control_pub_;
    ros::Publisher base_diff_pub_;
    ros::Publisher dynamical_pub_;
    ros::Publisher pub_task_space_deflection_pub_;
    //Add any desired variables here
    // double ob_cos_sig = 1.0;
    // tf::TransformListener tf_listener_;
    tf2_ros::Buffer tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;

    tf2_ros::TransformBroadcaster tfbroadcast;

    std::map<std::string,double> friction_gains; //gains for each joint
    // std::vector<KDL::JointHandlePtr> joints_;
    bool initialized_;  /// is KDL structure setup
    ros::Subscriber command_sub_;
    KDL::Chain kdl_chain_;
    KDL::JntArrayVel positions_;
    boost::shared_ptr<KDL::ChainDynParam> kdl_chain_dynamics_;
    boost::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    std::vector<std::string> joint_names_;
    std::vector<bool> continuous_;
    boost::mutex mutex_;
    std::vector<double> limits_lo_;
    std::vector<double> limits_hi_;
    std::vector<double> limits_gain_;
    std::vector<double> command_efforts_;
    std::vector<double> smoothed_efforts_;
    ros::Time last_command_time_;
    double Gravity_comp_funcensation_factor_;
    //Subscribe to the joint state topic
    ros::Subscriber jnt_state_sub;
    // The following must be private variables to be seen by all
    // Load URDF/KDL tree
    urdf::Model model;
    KDL::Tree kdl_tree;
    //Min angle for kinetic friction:
    float min_angle_qdot_speed_ = 0.05; //rad/s
    ros::ServiceClient switch_controller_client_;
    //for numerically calculating qddot
    Eigen::VectorXd qdot_prev_;
    ros::Time qdot_prev_time_;
    //For filtering the measured wrench
    Eigen::VectorXd wrench_ext_prev_;
    double alpha_wrench_lowpass_;
    // Two compliant configurations (A,B)
    std::map<std::string, double> joint_poses_A_comp_config;
    std::map<std::string, double> joint_poses_B_comp_config;
    std::map<std::string, double> joint_poses_comp_config;
    // Eigen::VectorXd task_space_comp_config;
    // geometry_msgs::TransformStamped task_space_comp_config;
    // tf2::Transform task_space_comp_config;
    // Eigen::Affine3d task_space_comp_config;
    Eigen::Isometry3d task_space_comp_config;
    // Stiffness matrix in task space
    Eigen::MatrixXd Stiffness_mat_taskspace;
    Eigen::MatrixXd Dampening_mat_taskspace;
    Eigen::MatrixXd Stiffness_mat_jointspace;
    Eigen::MatrixXd Dampening_mat_jointspace;
    bool jnt_space_oper_bool; //bool to choose whether to use jnt vs task space error

    //For large radian angle error, make a threshold value
    double q_err_max;
    std::map<std::string, double> q_err_max_dict;
    std::vector<std::string> active_joint_names;

    //Testing switching between poses
    ros::Time switch_pose_time;
    bool use_config_A; //true or false to switch from A to B


};


CompliantControl::CompliantControl()
{
  ROS_INFO("Compliant Control");
  nh = ros::NodeHandle(); //takes the node name (global node handle), If you use ~ then its private (under the node handle name) /armcontroller/ *param*
  arm_control_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/torque_control_arm/command",1); //topic name and queue_size
  base_diff_pub_ = nh.advertise<geometry_msgs::Twist>("/base_controller/command",1); //topic name and queue_size
  dynamical_pub_ = nh.advertise<intelligent_coop_carry_msgs::DynamicalTerms>("/dynamical_terms",1);

  pub_task_space_deflection_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/task_space_deflection",1);

  if (!model.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse URDF");
  }
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Could not construct tree from URDF");
  }
  // Populate the Chain
  std::string root, tip;
  nh.param<std::string>("root", root, "torso_lift_link");
  nh.param<std::string>("tip", tip, "gripper_link"); // "wrist_roll_link");
  //arm_control_pub_.publish(msg)
  kdl_tree.getChain(root, tip, kdl_chain_);
  if(!kdl_tree.getChain(root, tip, kdl_chain_))
  {
    ROS_ERROR("Could not construct chain from URDF");
    // return -1;
  }
  // ROS_INFO("made it past kdl_tree");
  kdl_chain_dynamics_.reset(new KDL::ChainDynParam(kdl_chain_, KDL::Vector(0,0,-9.81)));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  // Init positions
  positions_ = KDL::JntArrayVel(kdl_chain_.getNrOfJoints());
  KDL::SetToZero(positions_.q);
  KDL::SetToZero(positions_.qdot);
  // Init Joint Handles
  size_t num_joints;
  for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
  {
    KDL::Joint joint = kdl_chain_.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None)
    {
      joint_names_.push_back(joint.getName());
    }
    else
    {
      ROS_INFO_STREAM_NAMED("TorqueControllerArm", "Missing joint " << i);
    }
  }
  //Set friction gains dictionary(map),
  friction_gains["shoulder_pan_joint"] = 5.3;
  friction_gains["shoulder_lift_joint"] = 13.5;
  friction_gains["upperarm_roll_joint"] = 10.0;
  friction_gains["elbow_flex_joint"] = 10.0;
  friction_gains["forearm_roll_joint"] = 1.0;
  friction_gains["wrist_flex_joint"] = 0.1;
  friction_gains["wrist_roll_joint"] = 1.2;
  //Establish switch controller client
  switch_controller_client_ = nh.serviceClient<intelligent_coop_carry::SwitchController>("switch_robot_controllers");
  //Load the required controllers:
  load_required_controllers();
  //Initialize the start joint angles
  qdot_prev_ = Eigen::VectorXd::Zero(7);
  wrench_ext_prev_ =Eigen::VectorXd::Zero(6);
  alpha_wrench_lowpass_ =1.0; // 0.001; //a in [0,1]; how much to trust new values
  qdot_prev_time_ = ros::Time::now();//.toSec(); //for differentiating time

  //Initalize the two poses for 6 joints to use for compliance
  //Compliant Pose A (shoulder pan joint is not enforced)
  joint_poses_A_comp_config["shoulder_pan_joint"] = 0.5721750259399414;
  joint_poses_A_comp_config["shoulder_lift_joint"] = 0.7424468994140625;
  joint_poses_A_comp_config["upperarm_roll_joint"] = -2.1253304481506348;
  joint_poses_A_comp_config["elbow_flex_joint"] = 1.887946605682373;
  joint_poses_A_comp_config["forearm_roll_joint"] = -2.929136276245117;
  joint_poses_A_comp_config["wrist_flex_joint"] = 0.9644904136657715;
  joint_poses_A_comp_config["wrist_roll_joint"] = 0.7389952540397644;
  //Compliant Pose B (shoulder pan joint is not enforced)
  joint_poses_B_comp_config["shoulder_pan_joint"] = -0.5978689193725586;
  joint_poses_B_comp_config["shoulder_lift_joint"] = 0.7359275817871094;
  joint_poses_B_comp_config["upperarm_roll_joint"] = 2.14373779296875;
  joint_poses_B_comp_config["elbow_flex_joint"] = 1.7487378120422363;
  joint_poses_B_comp_config["forearm_roll_joint"] = 2.880049467086792;
  joint_poses_B_comp_config["wrist_flex_joint"] = 0.8482909202575684;
  joint_poses_B_comp_config["wrist_roll_joint"] = 2.3884081840515137;



  joint_poses_comp_config["shoulder_pan_joint"] = 0.026844501495361328;
  joint_poses_comp_config["shoulder_lift_joint"] = 0.8459906578063965;
  joint_poses_comp_config["upperarm_roll_joint"] = -3.0526223182678223;
  joint_poses_comp_config["elbow_flex_joint"] = 2.0436458587646484;
  joint_poses_comp_config["forearm_roll_joint"] = 0.046786416321992874;
  joint_poses_comp_config["wrist_flex_joint"] = -1.162757396697998;
  joint_poses_comp_config["wrist_roll_joint"] = 1.5332138538360596;


  //transition in nullspace of task space orientation?

  //Task Space goal config
  // TF:  tf_echo torso_lift_link gripper_link  (pts in gripper link frame in torso_lift_link frame)
  // - Translation: [0.908, -0.017, 0.368]
  // - Rotation: in Quaternion [0.695, -0.029, -0.011, 0.718]   th_rad = 1.603777, v =  0.99900568, -0.04168513, -0.0158116 (normed), hence vth = 1.53723482, -0.06414361, -0.02433034
  //           in RPY (radian) [1.539, -0.026, -0.056]
  //           in RPY (degree) [88.189, -1.491, -3.222]
  // vth = [0,-w3, w2;
  //        w3, 0,-w1;
  //       -w2, w1, 0]
  // w = 1.5377081, -0.0641633621,.0243378270
  // task_space_comp_config = Eigen::VectorXd::Zero(7);
  // task_space_comp_config(0) = 0.908; //p_x (position)
  // task_space_comp_config(1) = -0.017; //p_y (position)
  // task_space_comp_config(2) =  0.368; //p_z (position)
  // task_space_comp_config(3) = 0.695; //q_x (orientation)
  // task_space_comp_config(4) = -0.029; //q_y (orientation)
  // task_space_comp_config(5) = -0.011; //q_z (orientation)
  // task_space_comp_config(6) = 0.718; //q_w (orientation)

  //task_space_comp_config.translation() = Eigen::Vector3d(0.908, -0.017, 0.368);
  // tf2::Quaternion q;
  // q.setRotation(tf2::Vector3(1.5377081, -0.0641633621,.0243378270), 1.5387648496103778);
  //task_space_comp_config = task_space_comp_config.rotate(Eigen::AngleAxisd(1.5387648496103778, Eigen::Vector3d(0.99900568, -0.04168513, -0.0158116)));
  task_space_comp_config = Eigen::Translation3d(0.908, -0.017, 0.368) * Eigen::AngleAxisd(1.5387648496103778, Eigen::Vector3d(0.99900568, -0.04168513, -0.0158116));
  //How to construct affine transform: https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html


  //Choose whether to use jnt or task space error
  jnt_space_oper_bool = true;// false;



  //Estabilish the transformer
  tfListener.reset(new tf2_ros::TransformListener(tfBuffer));

  //Define the stiffness mat
  // double Kp_x_iso = 500.0;
  // double Kp_th_iso = 60.0;
  double Kp_x_iso = 70.0;
  double Kp_th_iso = 150.0;
  Stiffness_mat_taskspace = Eigen::MatrixXd::Zero(6,6);
  Stiffness_mat_taskspace(0,0)= Kp_x_iso;
  Stiffness_mat_taskspace(1,1)= Kp_x_iso;
  Stiffness_mat_taskspace(2,2)= Kp_x_iso;

  Stiffness_mat_taskspace(3,3)= Kp_th_iso;
  Stiffness_mat_taskspace(4,4)= Kp_th_iso;
  Stiffness_mat_taskspace(5,5)= Kp_th_iso;


  double Kd_x_iso = 2.0;
  double Kd_th_iso = 1.0;
  Dampening_mat_taskspace = Eigen::MatrixXd::Zero(6,6);
  Dampening_mat_taskspace(0,0) = Kd_x_iso;
  Dampening_mat_taskspace(1,1) = Kd_x_iso;
  Dampening_mat_taskspace(2,2) = Kd_x_iso;

  Dampening_mat_taskspace(3,3) = Kd_th_iso;
  Dampening_mat_taskspace(4,4) = Kd_th_iso;
  Dampening_mat_taskspace(5,5) = Kd_th_iso;


  Stiffness_mat_jointspace = Eigen::MatrixXd::Zero(7,7);
  //Emperically find the correct stiffness for each
  double jnt_space_stiff_const = 200.0; //150 when there is no inertia
  Stiffness_mat_jointspace (0,0) = jnt_space_stiff_const; //50.0;
  Stiffness_mat_jointspace (1,1) =  jnt_space_stiff_const;//120.0;
  Stiffness_mat_jointspace (2,2) =  jnt_space_stiff_const;//100.0;
  Stiffness_mat_jointspace (3,3) =  jnt_space_stiff_const;//120.0;
  Stiffness_mat_jointspace (4,4) =  jnt_space_stiff_const;//40.0;
  Stiffness_mat_jointspace (5,5) =  jnt_space_stiff_const;//30.0;
  Stiffness_mat_jointspace (6,6) =  jnt_space_stiff_const;//30.0;

  Dampening_mat_jointspace = Eigen::MatrixXd::Zero(7,7);
  double jnt_space_damp_const = 10.0; //10 when there is no inertia
  Dampening_mat_jointspace(0,0) = jnt_space_damp_const;
  Dampening_mat_jointspace(1,1) = jnt_space_damp_const;
  Dampening_mat_jointspace(2,2) = jnt_space_damp_const;
  Dampening_mat_jointspace(3,3) = jnt_space_damp_const;
  Dampening_mat_jointspace(4,4) = jnt_space_damp_const;
  Dampening_mat_jointspace(5,5) = jnt_space_damp_const;
  Dampening_mat_jointspace(6,6) = jnt_space_damp_const;



  //For large radian angle error, make a threshold value

  q_err_max = 0.087266; //~5*pi/180  rad

  double max_angle_def = 5;

  q_err_max_dict["shoulder_pan_joint"] = max_angle_def*PI/180;//0.087266;
  q_err_max_dict["shoulder_lift_joint"] =  max_angle_def*PI/180;//  0.1745;
  q_err_max_dict["upperarm_roll_joint"] = max_angle_def*PI/180.0 ;// 0.1745;
  q_err_max_dict["elbow_flex_joint"] =  max_angle_def*PI/180.0 ;///0.087266;
  q_err_max_dict["forearm_roll_joint"] =  max_angle_def*PI/180.0 ;///0.087266;
  q_err_max_dict["wrist_flex_joint"] =  max_angle_def*PI/180.0 ;///0.087266;
  q_err_max_dict["wrist_roll_joint"] =  max_angle_def*PI/180.0 ;///0.087266;

  active_joint_names = std::vector<std::string>{"shoulder_pan_joint",
                                                "shoulder_lift_joint",
                                                "upperarm_roll_joint",
                                                "elbow_flex_joint",
                                                "forearm_roll_joint",
                                                "wrist_flex_joint",
                                                "wrist_roll_joint"};



  //Testing switching between poses
  switch_pose_time = ros::Time::now();
  use_config_A = true; //true or false to switch from A to B

  //Intialize subscriber and spin
 jnt_state_sub = nh.subscribe("/joint_states",1,&CompliantControl::Jnt_state_callback, this); //this is always a pointer to current object
}

int CompliantControl::load_required_controllers()
{
  //Wait for service
  ros::service::waitForService("switch_robot_controllers");
  //Try to call service
  intelligent_coop_carry::SwitchController srv;
  srv.request.arm_base_both = "arm";
  srv.request.arm_control = "torque";
  if(switch_controller_client_.call(srv))
  {
    ROS_INFO("Switch service finished with condition: %d", srv.response.success);
    for(int i=0; i < srv.response.active_controllers.size(); i++ )
    {
      ROS_INFO("Loaded controllers: %s", srv.response.active_controllers[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Failed to call service switch_robot_controllers");
    return 1;
  }
  return 0;
}

void CompliantControl::Jnt_state_callback(const sensor_msgs::JointStateConstPtr& msg)
{

  // std::vector<std::string> msg_jnt_names = msg->name;
  if (std::find(msg->name.begin(), msg->name.end(),"shoulder_pan_joint")== msg->name.end())
  {
    return;
  }

    //1. Using joint data, first obtain all dynamical terms then publish them
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> effort;
    // ROS_INFO("made it into compliant callback");

    //1a. Obtain joint states
    for (size_t j=0; j < joint_names_.size(); j++){
      for (size_t i = 0; i < msg->name.size(); i++){
        if(msg->name[i] == joint_names_[j]){
          q.push_back(msg->position[i]);
          dq.push_back(msg->velocity[i]);
          effort.push_back(msg->effort[i]);
        }
      }
    }
    //Update the positions_ variable for KDL
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      positions_.q.data[i] = q[i];
      positions_.qdot.data[i] = dq[i];
    }
    //1b. Obtain dynamical terms
    size_t num_joints = joint_names_.size();
    //Gravity Vector
    KDL::JntArray gravity_jnt_arry(num_joints);
    kdl_chain_dynamics_->JntToGravity(positions_.q, gravity_jnt_arry);
    //Coriolis Vector
    KDL::JntArray coriolis_jnt_arry(num_joints);
    kdl_chain_dynamics_->JntToCoriolis(positions_.q,positions_.qdot, coriolis_jnt_arry);
    //Inertia Tensor
    KDL::JntSpaceInertiaMatrix inertia_jnt_mat(num_joints);
    kdl_chain_dynamics_->JntToMass(positions_.q, inertia_jnt_mat);
    //Jacobian Matrix
    KDL::Jacobian jacobian_jnt_mat(num_joints);
    jac_solver_->JntToJac(positions_.q, jacobian_jnt_mat);
    //access jnt_array: jnt_array.data[i] (Eigen::VectorXd);   but for matrix I, J:  J.data(i,j) (Eigen::Matrix), Inertia.data (Eigen::MatrixXd)
    // ROS_INFO("populated KDL Dynamical Terms");
    //Now publish the dynamical terms
    Publish_dynamical_terms(msg->header, inertia_jnt_mat.data,jacobian_jnt_mat.data, coriolis_jnt_arry.data, gravity_jnt_arry.data, q, dq, effort);
    // ROS_INFO("Published dynamical terms");
    //2. given dynamical terms call the appropriate control callback for
    //a) gravity/friction compensation
    // Eigen::VectorXd torque = Gravity_comp_func(gravity_jnt_arry.data, positions_.q.data, positions_.qdot.data);
    //b) compliant single pose
    // This function takes in specified goal pose, constructs and returns a q_vector



    //Use this to debug arm response
    // ros::Duration config_dt = ros::Time::now() - switch_pose_time;
    // if(config_dt.toSec() > 10.0 )
    // {
    //   if (use_config_A == true){
    //     use_config_A = false;
    //   }else{
    //     use_config_A = true;
    //   }
    //   switch_pose_time = ros::Time::now(); //reset start time to 0 for 10s count
    //   ROS_ERROR_STREAM("Switching configuration");
    // }

    use_config_A = true;


    Eigen::VectorXd q_goal_config = Eigen::VectorXd::Zero(7);
    for (size_t idx=0; idx < q.size(); idx++){
      q_goal_config(idx) = q[idx]; //start at current pose
    }
    /*
    //These were when switching between two poses was desired
    if(use_config_A == true)
    {
      q_goal_config = Current_goal_config(joint_poses_A_comp_config);
      ROS_ERROR_STREAM("Using config A");
    }else{
      q_goal_config = Current_goal_config(joint_poses_B_comp_config);
      ROS_ERROR_STREAM("Using config B");
    }
    */
    q_goal_config = Current_goal_config(joint_poses_comp_config);


    // Eigen::VectorXd q_goal_config = Current_goal_config(joint_poses_A_comp_config);

    //This commands all 7 torques, with the shoulder pan being set to just gravity compensation
    Eigen::VectorXd torque = Maintain_compliant_pose(q_goal_config, msg->header, inertia_jnt_mat.data,jacobian_jnt_mat.data, coriolis_jnt_arry.data, gravity_jnt_arry.data, q, dq, effort);
    //c) compliant hybrid controller btw two poses
    //Publish the given torque
    Publish_arm_control(torque, msg->header);
}


Eigen::VectorXd  CompliantControl::Current_goal_config(const std::map<std::string, double> q_goal_config_dict )
{
  Eigen::VectorXd q_goal_config(7);
  // q_goal_config(0) = q_goal_config_dict.find("shoulder_lift_joint")->second;
  q_goal_config(0) = q_goal_config_dict.at("shoulder_pan_joint");
  q_goal_config(1) = q_goal_config_dict.at("shoulder_lift_joint"); //Same thing as above, new to c++11
  q_goal_config(2) = q_goal_config_dict.at("upperarm_roll_joint");
  q_goal_config(3) = q_goal_config_dict.at("elbow_flex_joint");
  q_goal_config(4) = q_goal_config_dict.at("forearm_roll_joint");
  q_goal_config(5) = q_goal_config_dict.at("wrist_flex_joint");
  q_goal_config(6) = q_goal_config_dict.at("wrist_roll_joint");
  return q_goal_config;
}

void CompliantControl::Publish_pose_error(const Eigen::VectorXd pose_error, const geometry_msgs::TransformStamped gripper_transform)
{
  //This function takes a 6d pose error and publishes a marker for visualization
  visualization_msgs::Marker position_marker;
  position_marker.header.frame_id = "/torso_lift_link";
  position_marker.header.stamp = ros::Time::now();
  position_marker.id = 0;
  position_marker.type = visualization_msgs::Marker::ARROW;
  position_marker.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point position_start;
  geometry_msgs::Point position_end;
  position_start.x = gripper_transform.transform.translation.x;
  position_start.y = gripper_transform.transform.translation.y;
  position_start.z = gripper_transform.transform.translation.z;
  position_end.x = gripper_transform.transform.translation.x + pose_error(0);
  position_end.y = gripper_transform.transform.translation.y + pose_error(1);
  position_end.z = gripper_transform.transform.translation.z + pose_error(2);
  std::vector<geometry_msgs::Point> position_pts;
  position_pts.push_back(position_start);
  position_pts.push_back(position_end);
  position_marker.points = position_pts;
  position_marker.color.r = 1.0;
  position_marker.color.g = 0.0;
  position_marker.color.b = 0.0;
  position_marker.color.a = 1.0;
  position_marker.scale.x = 0.05;
  position_marker.scale.y = 0.1;
  position_marker.scale.z = 0.1;



  visualization_msgs::Marker orientation_marker;
  orientation_marker.header.frame_id = "torso_lift_link";
  orientation_marker.header.stamp = ros::Time::now();
  orientation_marker.id = 1;
  orientation_marker.type = visualization_msgs::Marker::ARROW;
  orientation_marker.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point orientation_start;
  geometry_msgs::Point orientation_end;
  orientation_start.x = gripper_transform.transform.translation.x;
  orientation_start.y = gripper_transform.transform.translation.y;
  orientation_start.z = gripper_transform.transform.translation.z;
  orientation_end.x = gripper_transform.transform.translation.x + pose_error(3);
  orientation_end.y = gripper_transform.transform.translation.y + pose_error(4);
  orientation_end.z = gripper_transform.transform.translation.z + pose_error(5);
  std::vector<geometry_msgs::Point> orientation_pts;
  orientation_pts.push_back(orientation_start);
  orientation_pts.push_back(orientation_end);
  orientation_marker.points = orientation_pts;
  orientation_marker.color.r = 0.0;
  orientation_marker.color.g = 0.0;
  orientation_marker.color.b = 1.0;
  orientation_marker.color.a = 1.0;
  orientation_marker.scale.x = 0.05;
  orientation_marker.scale.y = 0.1;
  orientation_marker.scale.z = 0.1;

  visualization_msgs::MarkerArray total_array;
  std::vector<visualization_msgs::Marker> marker_list;
  marker_list.push_back(position_marker);
  marker_list.push_back(orientation_marker);
  total_array.markers = marker_list;

  pub_task_space_deflection_pub_.publish(total_array);//Pub the marker array
}

Eigen::VectorXd CompliantControl::Transform_pose_error_function(Eigen::Affine3d goal, geometry_msgs::TransformStamped current){
  //This function takes two transforms (goal, current) and finds the transformed pose difference and returns it in a 6 vector
  Eigen::VectorXd pose_diff = Eigen::VectorXd::Zero(6);
  //Convert both transforms to eigen matricies
  // Eigen::Affine3d goal_transform_matrix = goal;
  // goal_transform_matrix = tf2::transformToEigen(goal);  //4x4 eigen affine matrix
  Eigen::Affine3d current_matrix = tf2::transformToEigen(current);
  //Now with these matricies find the transform difference
  //Get inverse
  // Eigen::Affine3d TF_diff  = current_matrix.inverse()*goal;
  Eigen::Affine3d TF_diff  = goal.inverse() * current_matrix;


  // ROS_ERROR_STREAM("Transform goal Matrix\n" << goal.matrix());
  // ROS_ERROR_STREAM("Transform curr Matrix\n" << current_matrix.matrix());
  // ROS_DEBUG_STREAM("Transform diff Matrix\n" << TF_diff.matrix());

  Eigen::Vector3d position_diff = -goal.rotation()* TF_diff.translation(); //[px,py,pz]
  Eigen::AngleAxisd rot_diff(TF_diff.rotation());
  double diff_angle = rot_diff.angle();
  Eigen::Vector3d diff_rot_axis = -goal.rotation()*diff_angle*rot_diff.axis();

  pose_diff(0) = position_diff(0);
  pose_diff(1) = position_diff(1);
  pose_diff(2) = position_diff(2);
  pose_diff(3) = diff_rot_axis(0);
  pose_diff(4) = diff_rot_axis(1);
  pose_diff(5) = diff_rot_axis(2);
  //Publish the goal transform
  geometry_msgs::TransformStamped goalTransformStamped = tf2::eigenToTransform(goal);
  goalTransformStamped.header.stamp = ros::Time::now();
  goalTransformStamped.header.frame_id = "/torso_lift_link";
  goalTransformStamped.child_frame_id = "goal_transform";
  tfbroadcast.sendTransform(goalTransformStamped);
  return pose_diff;
}

Eigen::VectorXd CompliantControl::Maintain_compliant_pose(const Eigen::VectorXd& q_goal_config, const std_msgs::Header& header, const Eigen::MatrixXd& inertia,const Eigen::MatrixXd& jacobian,const Eigen::VectorXd& coriolis,const Eigen::VectorXd& gravity,const std::vector<double>& q, const std::vector<double>& qdot, const std::vector<double>& effort)
{
  Eigen::VectorXd torque;
  //Add Gravity/coriolis components Component
  torque = gravity + coriolis;

  //Obtain joint state error (q_des - q)
  Eigen::VectorXd q_vect(7);
  Eigen::VectorXd qd_vect(7);
  Eigen::VectorXd q_error(7);
  Eigen::VectorXd qd_error(7);
  // q_error(0) = 0; // No error for shoulder pan joint (set it to zero rather than truncate)
  for(int i=0; i < 7; i++)
  {
    q_vect(i) = q[i];
    qd_vect(i) = qdot[i];
    //Ensure that if the angle error is too great that a smaller error is commanded (thresholding the error)

    //This is incorrect error
    // q_error(i) = q_goal_config(i) - q[i]; //+1 as q starts at shoulder pan joint
    //Obtain angle error
    q_error(i) = angle_diff(q_goal_config(i),q[i]); //Defined in header file


    ROS_ERROR_STREAM("\nqe i:"<< active_joint_names[i] <<"qd:"<<q_goal_config(i) << "q"<< q[i] << " init_err: " << q_error(i));
    //if ( copysign(1.0,q_error(i))*q_error(i) > q_err_max){

    double q_err_max_curr = q_err_max_dict[active_joint_names[i]]; //Look up dict for this joint

    if ( std::abs(q_error(i)) > q_err_max_curr){
      q_error(i) = copysign(1.0,q_error(i))*q_err_max_curr;
   // ROS_ERROR_STREAM("\n qe i:"<< active_joint_names[i] << "sign:" << copysign(1.0,q_error(i)) << " q_err_max" << q_err_max_curr);
    }

    // ROS_ERROR_STREAM("\nqe i:"<< active_joint_names[i] << " final_val: " << q_error(i));
    qd_error(i) = -qdot[i];
  }

  ROS_ERROR_STREAM("\n Total q_error:" << q_error);


  //Obtain the projected K_{q,p}_{n,n} = J^T_{n,6} K{x,p}_{6x6} J_{6xn}
  Eigen::VectorXd q_obj;
  q_obj = Eigen::VectorXd::Zero(7);

  Eigen::MatrixXd K_qp = jacobian.transpose() * Stiffness_mat_taskspace * jacobian;  //This method was having trouble getting middle joints to move, possible jacobian rank/manip_meas related
  Eigen::MatrixXd K_qd = jacobian.transpose() * Dampening_mat_taskspace * jacobian;  //This method was having trouble getting middle joints to move, possible jacobian rank/manip_meas related

  if(jnt_space_oper_bool){
   // q_obj = inertia*(K_qp*q_error +  K_qd*qd_error);// Calculate Objective qddot = K_{q,p}*(q_des-q)
    // Calculate final torque = M(q)*K_{q,p}(q_des-q) + C(dq,q) + G(q) +V(dq) //*ignore friction for now

  //ROS_ERROR_STREAM("q_err jnt space:" << q_error << "Stiffness mat" << Stiffness_mat_jointspace);

  //ROS_ERROR_STREAM("inertia tensor"<<inertia);
  //q_obj = inertia*(Stiffness_mat_jointspace*q_error+ Dampening_mat_jointspace*qd_error);
  q_obj = Stiffness_mat_jointspace*q_error + Dampening_mat_jointspace*qd_error;
  //ROS_ERROR_STREAM("q_obj jnt space:" << q_obj);
  }
  else{
    //Task Position Error
    //Obtain the current transform
    // tf::StampedTransform curr_task_space_transform;
    geometry_msgs::TransformStamped curr_task_space_transform;
    try
    {
      //Get position error
      // tf_listener_.lookupTransform("/torso_lift_link", "/gripper_link", ros::Time(1.0), curr_task_space_transform);
      curr_task_space_transform = tfBuffer.lookupTransform("torso_lift_link", "gripper_link", ros::Time(0),ros::Duration(0.1));
      Eigen::VectorXd task_space_pose_error = Transform_pose_error_function(task_space_comp_config,curr_task_space_transform);

      // //Print transforms
      // ROS_ERROR_STREAM("Transform orig" << task_space_comp_config.translation());
      // ROS_ERROR_STREAM(" Transform current " << curr_task_space_transform.transform.translation);
      // ROS_ERROR_STREAM(" Transform difference " << task_space_pose_error);

      Publish_pose_error(task_space_pose_error, curr_task_space_transform); //Visualize the pose error

      Eigen::MatrixXd Jacobian_V = jacobian.block<3,7>(0,0);
      Eigen::MatrixXd Jacobian_V_inv = pseudoInverse(Jacobian_V);
      Eigen::MatrixXd Jacobian_W = jacobian.block<3,7>(3,0);
      Eigen::MatrixXd Jacobian_W_inv = pseudoInverse(Jacobian_W);

      Eigen::Matrix<double,7,7> identity_7 = Eigen::Matrix<double,7,7>::Identity();



      Eigen::MatrixXd jacobian_inv = pseudoInverse(jacobian);
      //Get Velocity
      Eigen::VectorXd task_space_vel = jacobian*qd_vect;
      // q_obj = inertia*jacobian_inv*(Stiffness_mat_taskspace*task_space_pose_error);// - Dampening_mat_taskspace*task_space_vel); //This is Kp x_err (no differential)
      //Sudo task priority as no Jdot*qdot

      //Task space priority position/angle
      // q_obj = inertia*(pseudoInverse(Jacobian_V)*(Stiffness_mat_taskspace.block<3,3>(0,0)* task_space_pose_error.block<3,1>(0,0) - Dampening_mat_taskspace.block<3,3>(0,0)*task_space_vel.block<3,1>(0,0))
      //       + (identity_7 - Jacobian_V_inv*Jacobian_V)*Jacobian_W_inv
      //       *( Stiffness_mat_taskspace.block<3,3>(3,3)* task_space_pose_error.block<3,1>(3,0) - Jacobian_W*Jacobian_V_inv*(Stiffness_mat_taskspace.block<3,3>(0,0)* task_space_pose_error.block<3,1>(0,0)) ));

      // q_obj = inertia*Jacobian_W_inv*( Stiffness_mat_taskspace.block<3,3>(3,3)* task_space_pose_error.block<3,1>(3,0)); //Tests angular only


      //Position task priority for joint space
      // q_obj = inertia*(pseudoInverse(Jacobian_V)*(Stiffness_mat_taskspace.block<3,3>(0,0)* task_space_pose_error.block<3,1>(0,0) - Dampening_mat_taskspace.block<3,3>(0,0)*task_space_vel.block<3,1>(0,0))
      //       + (identity_7 - Jacobian_V_inv*Jacobian_V)*(Stiffness_mat_jointspace*q_error +  K_qd*qd_error) );

      //Free just position
      q_obj = inertia*(pseudoInverse(Jacobian_V)*(Stiffness_mat_taskspace.block<3,3>(0,0)* task_space_pose_error.block<3,1>(0,0)
              - Dampening_mat_taskspace.block<3,3>(0,0)*task_space_vel.block<3,1>(0,0)));

      //Need to move in nullspace of Jacobian
      Eigen::JacobiSVD<Eigen::MatrixXd> jacobian_svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);


      //Position with full jacobian priority
     /*
     q_obj = inertia*(pseudoInverse(Jacobian_V)*(Stiffness_mat_taskspace.block<3,3>(0,0)* task_space_pose_error.block<3,1>(0,0) - Dampening_mat_taskspace.block<3,3>(0,0)*task_space_vel.block<3,1>(0,0))
            + (identity_7 - jacobian_inv*jacobian)*(Stiffness_mat_jointspace*q_error +  K_qd*qd_error) );
      */

      //View motions between configurations
      //q_obj = inertia*(Stiffness_mat_jointspace*q_error +  K_qd*qd_error);
       //q_obj = inertia*(Stiffness_mat_jointspace*q_error + Dampening_mat_jointspace*qd_error);

      //This is better as the effect of the inertia tensor doesn't remove the effect of certain joints
      q_obj = Stiffness_mat_jointspace*q_error + Dampening_mat_jointspace*qd_error;

      // q_obj =(identity_7 - Jacobian_V_inv*Jacobian_V)*( Stiffness_mat_jointspace*q_error + Dampening_mat_jointspace*qd_error);

      Eigen::MatrixXd testmat = identity_7 - Jacobian_V_inv*Jacobian_V;
      ROS_ERROR_STREAM("Nullspace projection mat:" << testmat);



    }
    catch (tf2::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return torque;
    }
  }

  ROS_ERROR_STREAM("Popuplate curr torque: " << torque << " with q_obj: " << q_obj);

  for(size_t j=0; j<torque.size(); j++)
  {
    torque(j) += q_obj(j);
  }
  for(size_t j=0; j < joint_names_.size(); j++)
  {
    float qdot_sign = 1.0;
    if(qdot[j]<0){
      qdot_sign = -1.0;
    }
    if(qdot[j]*qdot_sign >= min_angle_qdot_speed_){
     //torque(j) += friction_gains[joint_names_[j]]*qdot_sign; //This now adds
      torque(j) = torque(j);
    }
  }


  return torque;
}



Eigen::VectorXd CompliantControl::Gravity_comp_func(const Eigen::VectorXd& gravity,const Eigen::VectorXd& q,const Eigen::VectorXd& qdot){
  Eigen::VectorXd torque;
  //Add Gravity Component
  torque = gravity;
  //Add friction component
  for(size_t j=0; j < joint_names_.size(); j++)
  {
    float qdot_sign = 1.0;
    if(qdot[j]<0){
      qdot_sign = -1.0;
    }
    if(qdot[j]*qdot_sign >= min_angle_qdot_speed_){
     // torque(j) = torque(j) + friction_gains[joint_names_[j]]*qdot_sign; //This now adds
     torque(j) = torque(j);
    }
  }
  //torque(6) = torque(6) + 1.2; //To individually test torques required to move joints 0-6
  return torque;
}



void CompliantControl::Publish_arm_control(const Eigen::VectorXd& torque, const std_msgs::Header& header)
{
  trajectory_msgs::JointTrajectory jnt_traj;
  std::vector<trajectory_msgs::JointTrajectoryPoint> jnt_traj_pt_list;
  trajectory_msgs::JointTrajectoryPoint jnt_traj_pt;
  for(size_t j=0; j < joint_names_.size(); j++)
  {
    jnt_traj_pt.effort.push_back(torque(j));
    jnt_traj.joint_names.push_back(joint_names_[j]);
  }
  jnt_traj_pt_list.push_back(jnt_traj_pt);
  jnt_traj.points = jnt_traj_pt_list;
  jnt_traj.header = header;
  //Now publish
  arm_control_pub_.publish(jnt_traj);
}

Eigen::VectorXd CompliantControl::convert_vector_for_TF(const Eigen::Affine3d transform_matrix, const Eigen::VectorXd& input_vect)
{
  //This operation converts from 3-vector to 4 vector with last element as 1 for mulpicliation to tranformation matrix
  Eigen::Vector4d new_vect;
  new_vect(0) = input_vect(0);
  new_vect(1) = input_vect(1);
  new_vect(2) = input_vect(2);
  new_vect(3) = 1.0;
  Eigen::Vector4d transformed_vect = transform_matrix*new_vect;
  Eigen::Vector3d final_vect;
  final_vect(0) = transformed_vect(0);
  final_vect(1) = transformed_vect(1);
  final_vect(2) = transformed_vect(2);
  return final_vect;
}

void CompliantControl::Publish_dynamical_terms(const std_msgs::Header& header, const Eigen::MatrixXd& inertia,const Eigen::MatrixXd& jacobian,const Eigen::VectorXd& coriolis,const Eigen::VectorXd& gravity,const std::vector<double>& q, const std::vector<double>& qdot, const std::vector<double>& effort)
{
  intelligent_coop_carry_msgs::DynamicalTerms dyn_term_msg;
  dyn_term_msg.header = header;
  //Populate Inertia
  for(int i=0; i<inertia.rows(); i++)
  {
    intelligent_coop_carry_msgs::Matrow curr_row;
    for(int j=0; j<inertia.cols(); j++)
    {
      curr_row.row.push_back(inertia(i,j));
    }
    dyn_term_msg.inertia.push_back(curr_row);
  }
  //Populate Jacobian
  for(int i=0; i<jacobian.rows(); i++)
  {
    intelligent_coop_carry_msgs::Matrow curr_row;
    for(int j=0; j<jacobian.cols(); j++)
    {
      curr_row.row.push_back(jacobian(i,j));
    }
    dyn_term_msg.jacobian.push_back(curr_row);
  }
  //Obtain qddot
  Eigen::VectorXd qddot(qdot.size());
  ros::Duration qdot_prev_dt =  ros::Time::now() - qdot_prev_time_; //Time from last query
  double dt = qdot_prev_dt.toSec();
  for (int i = 0; i < qdot.size(); i++)
  {
    if ((dt) < 0.1 )
    {
      qddot(i) = (qdot[i] - qdot_prev_(i))/(dt);
      qdot_prev_(i) = qdot[i];
    }
  }
  qdot_prev_time_ = ros::Time::now();
  Eigen::VectorXd Mqddot = inertia*qddot; //M(q) qddot
  //Jacobian inverse:
  Eigen::MatrixXd J_t_inv = pseudoInverse(jacobian.transpose());
  //Turn effort into a vector
  Eigen::VectorXd effort_vect(effort.size());
  for(int i = 0; i < effort.size(); i++)
  {
    if (dt< 0.05)
    {
      effort_vect(i) = effort[i] - coriolis(i) - gravity(i);// - Mqddot(i); //Subtract off terms to get external input (this should also include friction component added and M qddot)
    }
    else
    {
      effort_vect(i) = effort[i] - coriolis(i) - gravity(i); //Subtract off terms to get external input (this should also include friction component added and M qddot)
    }
  }
  Eigen::VectorXd wrench_ext = J_t_inv*effort_vect;
  //Get transforms and rotate wrench vector
  // tf::TransformStamped transform;
  geometry_msgs::TransformStamped transform;
  try
  {
    // tf_listener_.lookupTransform("/map", "/torso_lift_link", ros::Time(0), transform);
    // tf::Transform temp =  transform;
    // Eigen::Affine3d transform_matrix;
    // tf::transformTFToEigen(temp, transform_matrix);  //4x4 eigen affine matrix
    transform = tfBuffer.lookupTransform("map", "torso_lift_link", ros::Time(0),ros::Duration(0.1));
    Eigen::Affine3d transform_matrix = tf2::transformToEigen(transform);

    //obtain force/torque vectors
    Eigen::VectorXd force_vect_gripper_frame = wrench_ext.block<3,1>(0,0);
    Eigen::VectorXd torque_vect_gripper_frame = wrench_ext.block<3,1>(2,0);
    Eigen::VectorXd force_vect_map_frame = convert_vector_for_TF(transform_matrix, force_vect_gripper_frame);
    Eigen::VectorXd torque_vect_map_frame = convert_vector_for_TF(transform_matrix, torque_vect_gripper_frame);
    //Now reassign:
    wrench_ext(0) = force_vect_map_frame(0);
    wrench_ext(1) = force_vect_map_frame(1);
    wrench_ext(2) = force_vect_map_frame(2);
    wrench_ext(3) = torque_vect_map_frame(0);
    wrench_ext(4) = torque_vect_map_frame(1);
    wrench_ext(5) = torque_vect_map_frame(2);
  }
  catch (tf2::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return;
  }
  //Populate Dynamic variable
  dyn_term_msg.wrench_ext.header = header;
  dyn_term_msg.wrench_ext.header.frame_id = "shoulder_pan_link";
  dyn_term_msg.wrench_ext.wrench.force.x = wrench_ext(0)*alpha_wrench_lowpass_ + (1.0-alpha_wrench_lowpass_)*wrench_ext_prev_(0);
  dyn_term_msg.wrench_ext.wrench.force.y = wrench_ext(1)*alpha_wrench_lowpass_ + (1.0-alpha_wrench_lowpass_)*wrench_ext_prev_(1);
  dyn_term_msg.wrench_ext.wrench.force.z = wrench_ext(2)*alpha_wrench_lowpass_ + (1.0-alpha_wrench_lowpass_)*wrench_ext_prev_(2);
  dyn_term_msg.wrench_ext.wrench.torque.x = wrench_ext(3)*alpha_wrench_lowpass_ + (1.0-alpha_wrench_lowpass_)*wrench_ext_prev_(3);
  dyn_term_msg.wrench_ext.wrench.torque.y = wrench_ext(4)*alpha_wrench_lowpass_ + (1.0-alpha_wrench_lowpass_)*wrench_ext_prev_(4);
  dyn_term_msg.wrench_ext.wrench.torque.z = wrench_ext(5)*alpha_wrench_lowpass_ + (1.0-alpha_wrench_lowpass_)*wrench_ext_prev_(5);
  //Save this wrench for next time
  wrench_ext_prev_ = wrench_ext;
  //Get Manip measure
  Eigen::MatrixXd JJt = jacobian*jacobian.transpose();
  double JJt_det = JJt.determinant();
  double JJt_det_sqrt;
  if(JJt_det > 0)
  {
    JJt_det_sqrt = std::sqrt(JJt_det);
  }else{
    JJt_det_sqrt = -1; //Should always be positive semi-def, this way we can detect error
  }
  //Save manip measure
  dyn_term_msg.jac_manip = JJt_det_sqrt;
  //Populate joint names, q, qdot, effort, gravity, coriolis
  for(size_t j=0; j < joint_names_.size(); j++)
  {
    dyn_term_msg.joint_names.push_back(joint_names_[j]);
    dyn_term_msg.coriolis.push_back(coriolis(j));
    dyn_term_msg.gravity.push_back(gravity(j));
    dyn_term_msg.q.push_back(q[j]);
    dyn_term_msg.qdot.push_back(qdot[j]);
    dyn_term_msg.effort.push_back(effort[j]);
  }
  //Now publish to dynamics message
  dynamical_pub_.publish(dyn_term_msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_controller_node");
  ROS_INFO("repub_angle_vol_node_running");
  CompliantControl comp_control_obj; //Call the class, service callback is in the constructor, and looped by the ros spin
  ros::spin();
  return 0;
}
