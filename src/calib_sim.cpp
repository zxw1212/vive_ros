#include "ros/ros.h"
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include <termios.h>
#include <stdio.h>
#include <fstream>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>

/*#include <teleop_pilot_station/connect_to_robot.h>
#include <teleop_pilot_station/connect_robot_mic.h>
#include <teleop_pilot_station/connect_robot_speaker.h>
#include <av_manager/open_stream.h>
#include <av_manager/close_stream.h>

#include <robot_fwk_utilities/GetRobotInfo.h>
#include <av_manager/open_audio_stream.h>
#include <av_manager/close_audio_stream.h>*/

using namespace Eigen;
using namespace std;

#define   INIT                    0
#define   CALIB_HEAD              1
#define   CALIB_FRONT             2
#define   GET_SHOULDER_MATRIX     3
#define   SHOW_LIST               4
#define   CONNECT                 5       
#define   CHANGE_ROBOT            6  
#define   EXIT                    7         

// ****** GLOBAL VARIABLES ************************************************************************************************************
string            enter_joy;
string            esc_joy;
string            up_down_menu_joy;
string            right_left_menu_joy;
string            robot_base_vel_lin_joy;
string            robot_base_vel_ang_joy;
string            robot_left_hand_closure_joy;
string            robot_right_hand_closure_joy;

int               enter_button;
int               esc_button;
int               enter_b(0);
int               esc_b(0);
int               enter_b_old(0);
int               esc_b_old(0);
int               robot_base_vel_ang_axis;

vector<int>       robot_base_vel_lin_axis;

double            up_down_a_old(0);
double            up_down_a(0);
double            right_left_a_old(0);
double            right_left_a(0);
double            up_down_menu_axis(0);
double            right_left_menu_axis(0);
double            robot_left_hand_closure_axis(0);
double            robot_right_hand_closure_axis(0);
double            left_hand_cl(0);
double            right_hand_cl(0);

double            teaching_cmd_left_axis_x(0);
double            teaching_cmd_left_axis_y(0);
double            teaching_cmd_right_axis_x(0);
double            teaching_cmd_right_axis_y(0);

Vector3d          joy_l_pos_raw = Vector3d::Zero();
Vector3d          joy_r_pos_raw = Vector3d::Zero();
Vector3d          head_pos_raw = Vector3d::Zero();
Vector3d          robot_base_vel_lin = Vector3d::Zero();
Vector3d          robot_base_vel_ang = Vector3d::Zero();

Quaterniond       joy_l_ori_raw = Quaterniond::Identity();
Quaterniond       joy_r_ori_raw = Quaterniond::Identity();
Quaterniond       head_ori_raw = Quaterniond::Identity();

Matrix4d          sensor2head = Matrix4d::Identity();
Matrix4d          sensor2joyL = Matrix4d::Identity();
Matrix4d          sensor2joyR = Matrix4d::Identity();



sensor_msgs::Joy  teaching_left_cmd;

bool              new_msg = false;



// ****** SERVICES *******************************************************************************************************************


// ****** FUNCTIONS *******************************************************************************************************************
void joy_l__Callback(const sensor_msgs::JoyConstPtr& msg)
{
  new_msg = true;
  if(enter_joy == "L"){   // -- check enter
    enter_b_old = enter_b;
    enter_b = msg->buttons[enter_button];
  }

  if(esc_joy == "L"){   // -- check esc
    esc_b_old = esc_b;
    esc_b = msg->buttons[esc_button];
  }

  /*if(up_down_menu_joy == "L"){   // -- check up down menu
    up_down_a_old = up_down_a;
    up_down_a = msg->axes[up_down_menu_axis];

  }

  if(right_left_menu_joy== "L"){   // -- check right left menu
    right_left_a_old = right_left_a;
    right_left_a = msg->axes[right_left_menu_axis];
  }*/

  if(robot_base_vel_lin_joy== "L"){   // -- check linear base vel
    robot_base_vel_lin.x() = msg->axes[robot_base_vel_lin_axis[0]];
    robot_base_vel_lin.y() = msg->axes[robot_base_vel_lin_axis[1]];
    robot_base_vel_lin.z() = 0;
  }

  if(robot_base_vel_ang_joy== "L"){   // -- check angular  base vel
    robot_base_vel_ang.x() = 0;
    robot_base_vel_ang.y() = 0;
    robot_base_vel_ang.z() = msg->axes[robot_base_vel_ang_axis];
  }

  if(robot_left_hand_closure_joy== "L"){   // -- check left hand closure
    left_hand_cl = msg->axes[robot_left_hand_closure_axis];
  }

  if(robot_right_hand_closure_joy== "L"){   // -- check right hand closure
    right_hand_cl = msg->axes[robot_right_hand_closure_axis];
  }
  /*teaching_left_cmd.buttons = msg->buttons;
  teaching_left_cmd.axes = msg->axes;

  teaching_cmd_left_axis_x = msg->axes[0];
  teaching_cmd_left_axis_y = msg->axes[1];*/

}

void joy_r__Callback(const sensor_msgs::JoyConstPtr& msg)
{
  new_msg = true;
  if(enter_joy == "R"){   // -- check enter
    enter_b_old = enter_b;
    enter_b = msg->buttons[enter_button];
  }

  if(esc_joy == "R"){   // -- check esc
    esc_b_old = esc_b;
    esc_b = msg->buttons[esc_button];
  }

  if(up_down_menu_joy == "R"){   // -- check up down menu
    up_down_a_old = up_down_a;
    up_down_a = msg->axes[up_down_menu_axis];
  }

  if(right_left_menu_joy== "R"){   // -- check right left menu
    right_left_a_old = right_left_a;
    right_left_a = msg->axes[right_left_menu_axis];
  }

  if(robot_base_vel_lin_joy== "R"){   // -- check linear base vel
    robot_base_vel_lin.x() = msg->axes[robot_base_vel_lin_axis[0]];
    robot_base_vel_lin.y() = msg->axes[robot_base_vel_lin_axis[1]];
    robot_base_vel_lin.z() = 0;
  }

  if(robot_base_vel_ang_joy== "R"){   // -- check angular  base vel
    robot_base_vel_ang.x() = 0;
    robot_base_vel_ang.y() = 0;
    robot_base_vel_ang.z() = msg->axes[robot_base_vel_ang_axis];
  }

  if(robot_left_hand_closure_joy== "R"){   // -- check left hand closure
    left_hand_cl = msg->axes[robot_left_hand_closure_axis];
  }

  if(robot_right_hand_closure_joy== "R"){   // -- check right hand closure
    right_hand_cl = msg->axes[robot_right_hand_closure_axis];
  }
  teaching_cmd_right_axis_x = msg->axes[0];
  teaching_cmd_right_axis_y = msg->axes[1];
}

void joy_l_pose__Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  joy_l_pos_raw.x() = msg->pose.position.x;
  joy_l_pos_raw.y() = msg->pose.position.y;
  joy_l_pos_raw.z() = msg->pose.position.z;

  joy_l_ori_raw.x() = msg->pose.orientation.x;
  joy_l_ori_raw.y() = msg->pose.orientation.y;
  joy_l_ori_raw.z() = msg->pose.orientation.z;
  joy_l_ori_raw.w() = msg->pose.orientation.w;

  sensor2joyL.topLeftCorner(3,3) = (Matrix3d) joy_l_ori_raw;
  sensor2joyL.block<3,1>(0,3) = joy_l_pos_raw;
}

void joy_r_pose__Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  joy_r_pos_raw.x() = msg->pose.position.x;
  joy_r_pos_raw.y() = msg->pose.position.y;
  joy_r_pos_raw.z() = msg->pose.position.z;

  joy_r_ori_raw.x() = msg->pose.orientation.x;
  joy_r_ori_raw.y() = msg->pose.orientation.y;
  joy_r_ori_raw.z() = msg->pose.orientation.z;
  joy_r_ori_raw.w() = msg->pose.orientation.w;

  sensor2joyR.topLeftCorner(3,3) = (Matrix3d) joy_r_ori_raw;
  sensor2joyR.block<3,1>(0,3) = joy_r_pos_raw;
}


void head_pose__Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  head_pos_raw.x() = msg->pose.position.x;
  head_pos_raw.y() = msg->pose.position.y;
  head_pos_raw.z() = msg->pose.position.z;

  head_ori_raw.x() = msg->pose.orientation.x;
  head_ori_raw.y() = msg->pose.orientation.y;
  head_ori_raw.z() = msg->pose.orientation.z;
  head_ori_raw.w() = msg->pose.orientation.w;

  sensor2head.topLeftCorner(3,3) = (Matrix3d) head_ori_raw;
  sensor2head.block<3,1>(0,3) = head_pos_raw;
}


Matrix4d inv_H(Matrix4d M){
  Matrix4d M_inv = Matrix4d::Identity();

  M_inv.topLeftCorner(3,3) = ((Matrix3d) M.topLeftCorner(3,3)).transpose();
  M_inv.block<3,1>(0,3) = -((Matrix3d) M.topLeftCorner(3,3)).transpose() * M.block<3,1>(0,3);

  return M_inv;
}

string execute( std::string cmd )
{
    string file_name = "result.txt" ;
    system( ( cmd + " > " + file_name ).c_str() ) ; // redirect output to file

    // open file for input, return string containing characters in the file
    ifstream file(file_name.c_str()) ;
    return { istreambuf_iterator<char>(file), istreambuf_iterator<char>() } ;
}


int cursor_movement(int menu_index, double up_down_shift, int n_elem)
{
  static bool              time_flag(false);
  static ros::Time         time_index;

  if(up_down_shift < 0.6 && up_down_shift > -0.6)
    {
      time_index = ros::Time::now();
      time_flag = true;
    }

  if((ros::Time::now() - time_index > ros::Duration(0.02)) && time_flag)
  {
    time_flag = false;
    if(up_down_shift > 0 && menu_index > 0)
    {
      menu_index--;
    }
    if(up_down_shift < 0 && menu_index < n_elem-1)
    {
      menu_index++;
    }
  }

  return menu_index;
}

Matrix4d inv_Mat (Matrix4d M){
	Matrix4d inv_M = Matrix4d::Identity();

	inv_M.block<3,3>(0,0) = M.block<3,3>(0,0).transpose();
	inv_M.block<3,1>(0,3) = -M.block<3,3>(0,0).transpose()*M.block<3,1>(0,3);

	return inv_M;
}

/*string get_robot_IP(ros::ServiceClient get_robot_info_client, robot_fwk_utilities::GetRobotInfo get_robot_info_srv, string robot_node_name,string get_robot_info_sv)
{
  // call get robot info service
  get_robot_info_srv.request.request_info = true;
  if(get_robot_info_client.call(get_robot_info_srv))
  {
    
    std::cout << get_robot_info_srv.response.robot_ip << std::endl;
    std::cout << get_robot_info_srv.response.robot_name << std::endl;
    std::cout << "------------------------------" << std::endl;
  }
  else{
    ROS_INFO("Error in get_robot_info service!");
    return "error";
  }
  return get_robot_info_srv.response.robot_ip;
}*/


// ****** MAIN F. *********************************************************************************************************************
int main(int argc, char **argv)
{
	int 	            run_freq(30);							          // Node frequecy
  int               state(0);
  int               n_elem(0);
  int               menu_index(0);

  double            arm_l(1);
  double            shoulder_d(0);
  double            neck_l(0);
  double            bias_vive(0.17);
  double head_roll, head_pitch, head_yaw;


  string            joy_l_tn;
  string            joy_r_tn;
  string            up_menu_tn;
  string            central_menu_tn;
  string            down_menu_tn;
  string            joy_l_pose_tn;
  string            joy_r_pose_tn;
  string            head_pose_tn;
  string            calib_head_str;
  string            calib_down_str;
  string            calib_front_str;
  string            list_menu;
  string            separator_str;
  string            show_robot_list_info_str;
  string            change_robot_info_str;
  string            to;
  string            robot_ns;
  string            open_stream_sv;
  string            close_stream_sv;
  string            get_robot_info_sv;
  string            open_audio_sv;
  string            close_audio_sv;
  string            left_camera_tail;
  string            right_camera_tail;
  string            head_pose_tail;
  string            left_joy_pose_tail;
  string            right_joy_pose_tail;
  string            twist_base_tail;
  string            left_joy_closure_tail;
  string            right_joy_closure_tail;
  string            teaching_cmd_left_tail;
  string            teaching_cmd_right_tail;
  string            robot_left_camera_tn;
  string            robot_right_camera_tn;
  string            robot_left_mic_tn;
  string            robot_right_mic_tn;
  string            robot_speaker_tn;
  string            robot_head_pose_tn;
  string            robot_hand_L_pose_tn;
  string            robot_hand_R_pose_tn;
  string            robot_twist_base_tn;
  string            find_robots_cmd;
  string            left_joy_closure_tn;
  string            right_joy_closure_tn;
  string            teaching_cmd_left_tn;
  string            teaching_cmd_right_tn;
  string            world_tf_frame;
  string            ip_address;
  string            left_camera_port;
  string            right_camera_port;
  string            audio_local_port;
  string            audio_remote_port;

  stringstream      ss;

  string            robot_list[50];
  string            robot_ip_list[50];

  bool              exit_prog(false);
  bool              enable_cmd(false);
  bool              enable_robot_microphone(true);
  bool              enable_robot_speaker(true);

  Vector3d          head_calib2head_P;
  Vector3d          shoulder2joyR_P;
  Vector3d          shoulder2joyL_P;
  Vector3d          euler;


  Quaterniond       head_calib2head_Q;
  Quaterniond       shoulder2joyR_Q;
  Quaterniond       shoulder2joyL_Q;

  Matrix4d          sensor2head_calib = Matrix4d::Identity();
  Matrix4d          head_calib2head = Matrix4d::Identity();
  Matrix4d          shoulder2joyL = Matrix4d::Identity();
  Matrix4d          shoulder2joyR = Matrix4d::Identity();
  Matrix4d          head_calib2shoulderL = Matrix4d::Identity();
  Matrix4d          head_calib2shoulderR = Matrix4d::Identity();
  Matrix4d          frankaR2frankaRang = Matrix4d::Identity();
  Matrix4d          frankaL2frankaLang = Matrix4d::Identity();
  Matrix4d          pilotHand2qbHandR = Matrix4d::Identity();
  Matrix4d          pilotHand2qbHandL = Matrix4d::Identity();

  // Define quaternion for head pose in order to trasform it from quaternion to RPY
  tf::Quaternion    head_quat;

  ros::Duration     robot_list_refresh_rate = ros::Duration(60);
  ros::Time         robot_list_refresh_timer(0);
	// --------------------------------------------------------------- Init node
	ros::init(argc, argv, "main_pilot");
	ros::NodeHandle n;


	// --------------------------------------------------------------- Get params
  n.getParam("FREQ_RATE", run_freq);
  n.getParam("JOY_L_TN", joy_l_tn);
  n.getParam("JOY_R_TN", joy_r_tn);
  n.getParam("UP_MENU_TN", up_menu_tn);
  n.getParam("CENTRAL_MENU_TN", central_menu_tn);
  n.getParam("BOTTOM_MENU_TN", down_menu_tn);
  n.getParam("JOY_L_POSE_TN", joy_l_pose_tn);
  n.getParam("JOY_R_POSE_TN", joy_r_pose_tn);
  n.getParam("HEAD_POSE_TN", head_pose_tn);
  n.getParam("OPEN_STREAM_SV", open_stream_sv);
  n.getParam("CLOSE_STREAM_SV", close_stream_sv);
  n.getParam("GET_ROBOT_INFO_SV", get_robot_info_sv);
  n.getParam("OPEN_AUDIO_SV", open_audio_sv);
  n.getParam("CLOSE_AUDIO_SV", close_audio_sv);
  n.getParam("LEFT_CAMERA_TAIL", left_camera_tail);
  n.getParam("RIGHT_CAMERA_TAIL", right_camera_tail);
  //n.getParam("LEFT_MIC_TAIL", left_mic_tail);
  //n.getParam("RIGHT_MIC_TAIL", right_mic_tail);
  //n.getParam("ROBOT_SPEAKER_TAIL", robot_speaker_tail);
  n.getParam("HEAD_POSE_TAIL", head_pose_tail);
  n.getParam("LEFT_JOY_POSE_TAIL", left_joy_pose_tail);
  n.getParam("RIGHT_JOY_POSE_TAIL", right_joy_pose_tail);
  /*n.getParam("HEAD_POSE_TAIL", robot_head_pose_tn); 
  n.getParam("LEFT_JOY_POSE_TAIL", robot_hand_L_pose_tn);
  n.getParam("RIGHT_JOY_POSE_TAIL", robot_hand_R_pose_tn);*/
  n.getParam("TWIST_BASE_TAIL", twist_base_tail);
  n.getParam("LEFT_JOY_CLOSURE_TAIL", left_joy_closure_tail);
  n.getParam("RIGHT_JOY_CLOSURE_TAIL", right_joy_closure_tail);
  n.getParam("TEACHING_CMD_LEFT_TAIL", teaching_cmd_left_tail);
  n.getParam("TEACHING_CMD_RIGHT_TAIL", teaching_cmd_right_tail);
  

  n.getParam("ENTER_JOY", enter_joy);
  n.getParam("ENTER_BUTTON", enter_button);
  n.getParam("ESC_JOY", esc_joy);
  n.getParam("ESC_BUTTON", esc_button);
  n.getParam("UP_DOWN_MENU_JOY", up_down_menu_joy);
  n.getParam("UP_DOWN_MENU_AXIS", up_down_menu_axis);
  n.getParam("RIGHT_LEFT_MENU_JOY", right_left_menu_joy);
  n.getParam("RIGHT_LEFT_MENU_AXIS", right_left_menu_axis);
  n.getParam("ROBOT_BASE_VEL_LIN_JOY", robot_base_vel_lin_joy);
  n.getParam("ROBOT_BASE_VEL_LIN_AXIS", robot_base_vel_lin_axis);
  n.getParam("ROBOT_BASE_VEL_ANG_JOY", robot_base_vel_ang_joy);
  n.getParam("ROBOT_BASE_VEL_ANG_AXIS", robot_base_vel_ang_axis);
  n.getParam("ROBOT_LEFT_HAND_CLOSURE_JOY", robot_left_hand_closure_joy);
  n.getParam("ROBOT_LEFT_HAND_CLOSURE_AXIS", robot_left_hand_closure_axis);
  n.getParam("ROBOT_RIGHT_HAND_CLOSURE_JOY", robot_right_hand_closure_joy);
  n.getParam("ROBOT_RIGHT_HAND_CLOSURE_AXIS", robot_right_hand_closure_axis);


  n.getParam("SEPARATOR", separator_str);
  n.getParam("CALIB_HEAD_STR", calib_head_str);
  n.getParam("CALIB_DOWN_STR", calib_down_str);
  n.getParam("CALIB_FRONT_STR", calib_front_str);
  n.getParam("SHOW_ROBOT_LIST_INFO_STR", show_robot_list_info_str);
  n.getParam("CHANGE_ROBOT_INFO_STR", change_robot_info_str);

  n.getParam("ARM_LENGTH", arm_l);
  n.getParam("SHOULDER_DIST", shoulder_d);
  n.getParam("NECK_LENGTH", neck_l);

  n.getParam("FIND_ROBOT_CMD", find_robots_cmd);

  n.getParam("WORLD_TF_FRAME", world_tf_frame);

  n.getParam("IP_ADDRESS", ip_address);
  n.getParam("LEFT_CAMERA_PORT", left_camera_port);
  n.getParam("RIGHT_CAMERA_PORT", right_camera_port);
  n.getParam("AUDIO_LOCAL_PORT", audio_local_port);
  n.getParam("AUDIO_REMOTE_PORT", audio_remote_port);


	// --------------------------------------------------------------- Subscribe to topics 
  ros::Subscriber      joy_l_sub        = n.subscribe(joy_l_tn, 1, joy_l__Callback); 
  ros::Subscriber      joy_r_sub        = n.subscribe(joy_r_tn, 1, joy_r__Callback); 
  ros::Subscriber      joy_l_pose_sub   = n.subscribe(joy_l_pose_tn, 1, joy_l_pose__Callback); 
  ros::Subscriber      joy_r_pose_sub   = n.subscribe(joy_r_pose_tn, 1, joy_r_pose__Callback); 
  ros::Subscriber      head_pose_sub    = n.subscribe(head_pose_tn, 1, head_pose__Callback); 
  

  // --------------------------------------------------------------- Published topics
  ros::Publisher up_menu_pub = n.advertise<std_msgs::String>(up_menu_tn, 1);
  ros::Publisher central_menu_pub = n.advertise<std_msgs::String>(central_menu_tn, 1);
  ros::Publisher down_menu_pub = n.advertise<std_msgs::String>(down_menu_tn, 1);
  ros::Publisher head_pose_pub;
  ros::Publisher left_joy_pose_pub;
  ros::Publisher right_joy_pose_pub;
  ros::Publisher twist_base_pub;
  ros::Publisher left_joy_closure_pub;
  ros::Publisher right_joy_closure_pub;
  ros::Publisher teaching_cmd_left_pub;
  ros::Publisher teaching_cmd_right_pub;

  // Define publisher for RPY head pose. In fact neck_position_controller needs RPY data
  ros::Publisher neck_RPY_pub = n.advertise<std_msgs::Float64MultiArray>("/neck_position_controller/command", 1); //if publisher write directly on the controller topic, the remapping is unnecessary


  // --------------------------------------------------------------- Services
  // Video connection service
  /*ros::ServiceClient open_stream_client;// = n.serviceClient<av_manager::open_stream>(open_stream_sv);
  av_manager::open_stream open_stream_srv;

  ros::ServiceClient close_stream_client;// = n.serviceClient<av_manager::close_stream>(close_stream_sv);
  av_manager::close_stream close_stream_srv;
  // Get robot info (name and ip) service
  ros::ServiceClient get_robot_info_client;// = n.serviceClient<av_manager::close_stream>(close_stream_sv);
  robot_fwk_utilities::GetRobotInfo get_robot_info_srv;

  // Open audio service
  ros::ServiceClient open_audio_client = n.serviceClient<av_manager::open_audio_stream>(ros::this_node::getNamespace() + open_audio_sv);
  av_manager::open_audio_stream open_audio_srv;

  // Close audio service
  ros::ServiceClient close_audio_client = n.serviceClient<av_manager::close_audio_stream>(ros::this_node::getNamespace() + close_audio_sv);
  av_manager::close_audio_stream close_audio_srv;*/

  // --------------------------------------------------------------- Msgs
  std_msgs::String              up_menu_msg;
  std_msgs::String              central_menu_msg;
  std_msgs::String              down_menu_msg;
  
  geometry_msgs::PoseStamped    head_pose_msg;
  std_msgs::Float64MultiArray   head_joint_msg;
  geometry_msgs::PoseStamped    left_joy_pose_msg;
  geometry_msgs::PoseStamped    right_joy_pose_msg;

  //geometry_msgs::TwistStamped   twist_base_msg;
  geometry_msgs::Twist          twist_base_msg;

  std_msgs::Float64             left_joy_closure_msg;
  std_msgs::Float64             right_joy_closure_msg;

  sensor_msgs::Joy              teaching_cmd_left_msg;
  sensor_msgs::Joy              teaching_cmd_right_msg;
  

  // --------------------------------------------------------------- Init vars
  ros::Rate loop_rate(run_freq);

  static tf::TransformBroadcaster     br;
  tf::Transform                       tf_head;
  tf::Transform                       tf_hand_L;
  tf::Transform                       tf_hand_R;

  frankaR2frankaRang.topLeftCorner(3,3) = (Matrix3d) AngleAxisd(10.0/180*M_PI, Vector3d::UnitZ()) * (Matrix3d) AngleAxisd(80.0/180*M_PI, Vector3d::UnitX());
	frankaL2frankaLang.topLeftCorner(3,3) = (Matrix3d) AngleAxisd(-10.0/180*M_PI, Vector3d::UnitZ()) * (Matrix3d) AngleAxisd(-80.0/180*M_PI, Vector3d::UnitX());

  pilotHand2qbHandR.topLeftCorner(3,3) = (Matrix3d) AngleAxisd(-0.5*M_PI, Vector3d::UnitX());
	pilotHand2qbHandL.topLeftCorner(3,3) = (Matrix3d) AngleAxisd(0.5*M_PI, Vector3d::UnitX());

  while (ros::ok() && !exit_prog){ // --------------------------------------------- MAIN LOOP 

    // --- Get Pose in the right Ref
    head_calib2head = inv_H(sensor2head_calib)*sensor2head;
    // shoulder2joyR = inv_H(head_calib2shoulderR) * inv_H(sensor2head_calib) * sensor2joyR;
    // shoulder2joyL = inv_H(head_calib2shoulderL) * inv_H(sensor2head_calib) * sensor2joyL;

    // shoulder2joyR = inv_Mat(frankaR2frankaRang) * shoulder2joyR * pilotHand2qbHandR;
  	// shoulder2joyL = inv_Mat(frankaL2frankaLang) * shoulder2joyL * pilotHand2qbHandL;

    shoulder2joyR = inv_Mat(frankaR2frankaRang) * inv_H(head_calib2shoulderR) * inv_H(sensor2head_calib) * sensor2joyR * pilotHand2qbHandR;
    shoulder2joyL =  inv_Mat(frankaL2frankaLang) * inv_H(head_calib2shoulderL) * inv_H(sensor2head_calib) * sensor2joyL * pilotHand2qbHandL;


    shoulder2joyR.block<3,1>(0,3) /= arm_l;
    shoulder2joyL.block<3,1>(0,3) /= arm_l;

    head_calib2head_P = head_calib2head.block<3,1>(0,3);
    head_calib2head_Q = head_calib2head.block<3,3>(0,0);

    shoulder2joyR_P = shoulder2joyR.block<3,1>(0,3);
    shoulder2joyR_Q = shoulder2joyR.block<3,3>(0,0);

    shoulder2joyL_P = shoulder2joyL.block<3,1>(0,3);
    shoulder2joyL_Q = shoulder2joyL.block<3,3>(0,0);


    // --- Publish TF 

    tf_head.setOrigin( tf::Vector3(head_calib2head_P.x(), head_calib2head_P.y(), head_calib2head_P.z()) );
    tf_head.setRotation(tf::Quaternion(head_calib2head_Q.x(), head_calib2head_Q.y(),head_calib2head_Q.z(),head_calib2head_Q.w()));
    br.sendTransform(tf::StampedTransform(tf_head, ros::Time::now(), world_tf_frame, "head"));

    tf_hand_R.setOrigin( tf::Vector3(shoulder2joyR_P.x(), shoulder2joyR_P.y(), shoulder2joyR_P.z()) );
    tf_hand_R.setRotation(tf::Quaternion(shoulder2joyR_Q.x(), shoulder2joyR_Q.y(),shoulder2joyR_Q.z(),shoulder2joyR_Q.w()));
    br.sendTransform(tf::StampedTransform(tf_hand_R, ros::Time::now(), "right_link0", "hand_R"));

    tf_hand_L.setOrigin( tf::Vector3(shoulder2joyL_P.x(), shoulder2joyL_P.y(), shoulder2joyL_P.z()) );
    tf_hand_L.setRotation(tf::Quaternion(shoulder2joyL_Q.x(), shoulder2joyL_Q.y(),shoulder2joyL_Q.z(),shoulder2joyL_Q.w()));
    br.sendTransform(tf::StampedTransform(tf_hand_L, ros::Time::now(), "left_link0", "hand_L"));
   


    switch(state){
      case INIT: // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ INIT
        state = CALIB_HEAD;
        
        break;

      case CALIB_HEAD: // +++++++++++++++++++++++++++++++++++++++++++++++++++++++ CALIB_HEAD
        down_menu_msg.data = calib_head_str;
        down_menu_pub.publish(down_menu_msg);
        std::cout << down_menu_msg.data<< "\n";

        if(enter_b_old == 1 && enter_b == 0 && new_msg){
          new_msg = false;
          sensor2head_calib.topLeftCorner(3,3) = sensor2head.topLeftCorner(3,3);
          sensor2head_calib.block<3,1>(0,3) = sensor2head.block<3,1>(0,3);

          if (arm_l <= 0 || shoulder_d <= 0 || neck_l <= 0){
            state = CALIB_FRONT;
          }
          else{
            state = GET_SHOULDER_MATRIX;
          }
        }
        break;

      case CALIB_FRONT: // ++++++++++++++++++++++++++++++++++++++++++++++++++++++ CALIB_FRONT
        down_menu_msg.data = calib_front_str;
        down_menu_pub.publish(down_menu_msg);
       std::cout << down_menu_msg.data<< "\n";

        if(enter_b_old == 1 && enter_b == 0 && new_msg){
          
          new_msg = false;

          // --- create calibration matrix for both arms
          arm_l = ((inv_H(sensor2head_calib)*sensor2joyR)(0,3) + (inv_H(sensor2head_calib)*sensor2joyL)(0,3))/2 +bias_vive;
          shoulder_d = fabs((inv_H(sensor2head_calib)*sensor2joyL)(1,3) - (inv_H(sensor2head_calib)*sensor2joyR)(1,3));
          neck_l = fabs(((inv_H(sensor2head_calib)*sensor2joyR)(2,3) + (inv_H(sensor2head_calib)*sensor2joyL)(2,3))/2);
          

          cout << "CALIBRATION COMPLETE:" << endl;
          cout << "   -Arm length         = " << arm_l << endl;
          cout << "   -Shoulders distance = " << shoulder_d << endl;
          cout << "   -Neck length        = " << neck_l << endl;

          state = GET_SHOULDER_MATRIX;
        }
        break;

      case GET_SHOULDER_MATRIX: // ++++++++++++++++++++++++++++++++++++++++++++++ GET_SHOULDER_MATRIX

          head_calib2shoulderL.topLeftCorner(3,3) = Matrix3d::Identity();
          head_calib2shoulderL.block<3,1>(0,3) << 0, shoulder_d/2, -neck_l;
          
          head_calib2shoulderR.topLeftCorner(3,3) = Matrix3d::Identity();
          head_calib2shoulderR.block<3,1>(0,3) << 0, -shoulder_d/2, -neck_l;

          menu_index = 0;

          state =  SHOW_LIST;
        break;

      case SHOW_LIST: // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++ SHOW_LIST
        // --- execute ROS command
        /*if((ros::Time::now() - robot_list_refresh_timer) > robot_list_refresh_rate){
          ss = stringstream(execute(find_robots_cmd));

          robot_list_refresh_timer = ros::Time::now();
        
          // --- Get elements of the list
          std::cout << "ROBOT LIST: " << std::endl;
          n_elem = 0;
          while(std::getline(ss, to, '\n')){
            robot_list[n_elem] = to;
            get_robot_info_client = n.serviceClient<robot_fwk_utilities::GetRobotInfo>("/" + robot_list[n_elem] + get_robot_info_sv);
            robot_ip_list[n_elem] = get_robot_IP(get_robot_info_client, get_robot_info_srv, robot_list[n_elem], get_robot_info_sv);
            n_elem++;
          }
          robot_list[n_elem] = "REFRESH";
          robot_list[n_elem+1] = "EXIT";
        }

        // --- Move cursor
        menu_index = cursor_movement(menu_index, up_down_a, n_elem+2);
        // --- Print the list in a string adding the cursor and the separator
        list_menu = "";
        for(int i=0; i<=n_elem+1; i++){
          if (menu_index == i){
            list_menu = list_menu + "> " + robot_list[i] + "*" ;
          }
          else{
            list_menu = list_menu + "  " + robot_list[i] + "*" ;
          }

        }

        // --- Publish the list
        central_menu_msg.data = list_menu;
        central_menu_pub.publish(central_menu_msg);

        down_menu_msg.data = show_robot_list_info_str;
        down_menu_pub.publish(down_menu_msg);

        // --- Check if a robot is selected
        if(enter_b_old == 1 && enter_b == 0 && new_msg){

          new_msg = false;
          if(menu_index >= n_elem){
            
            if(menu_index == n_elem){ // REFRESH BUTTON
              ss = stringstream(execute(find_robots_cmd));
              robot_list_refresh_timer = ros::Time::now();

              // --- Get elements of the list
              std::cout << "ROBOT LIST: " << std::endl;
              n_elem = 0;
              while(std::getline(ss, to, '\n')){
                robot_list[n_elem] = to;
                get_robot_info_client = n.serviceClient<robot_fwk_utilities::GetRobotInfo>("/" + robot_list[n_elem] + get_robot_info_sv);
                robot_ip_list[n_elem] = get_robot_IP(get_robot_info_client, get_robot_info_srv, robot_list[n_elem], get_robot_info_sv);
                n_elem++;
              }

              robot_list[n_elem] = "REFRESH";
              robot_list[n_elem+1] = "EXIT";

              menu_index = n_elem; // cast to REFRESH BUTTON
            }
            if(menu_index == n_elem+1){ //EXIT BUTTON
              state = EXIT;
            }
          }
          else{

            robot_ns = robot_list[menu_index];

            // --- Create the topic name for each topic
            robot_left_camera_tn = "/" + robot_ns + left_camera_tail;
            robot_right_camera_tn = "/" + robot_ns + right_camera_tail;

            //robot_left_mic_tn = "/" + robot_ns + left_mic_tail;
            //robot_right_mic_tn = "/" + robot_ns + right_mic_tail;

            //robot_speaker_tn = "/" + robot_ns + robot_speaker_tail;*/

            robot_head_pose_tn = "/" + robot_ns + head_pose_tail;
            robot_hand_L_pose_tn = "/" + robot_ns + left_joy_pose_tail;
            robot_hand_R_pose_tn = "/" + robot_ns + right_joy_pose_tail;
            robot_twist_base_tn = "/" + robot_ns + twist_base_tail;
            /*left_joy_closure_tn = "/" + robot_ns + left_joy_closure_tail;
            right_joy_closure_tn = "/" + robot_ns + right_joy_closure_tail;
            teaching_cmd_left_tn = "/" + robot_ns + teaching_cmd_left_tail;
            teaching_cmd_right_tn = "/" + robot_ns + teaching_cmd_right_tail;


            // --- Call service to connect microphone to the audio manager node
            /*if(enable_robot_microphone)
            {
              std::cout << "CONNECTING ROBOT MICROPHONE 2 PILOT SPEAKER"<< std::endl;
              connect_robot_mic_srv.request.left_mic = robot_left_mic_tn;
              connect_robot_mic_srv.request.right_mic = robot_right_mic_tn;
              connect_robot_mic_client.call(connect_robot_mic_srv);             
            }

            if(enable_robot_speaker)
            {
              std::cout << "CONNECTING PILOT MICROPHONE 2 ROBOT SPEAKER"<< std::endl;
              connect_robot_speaker_srv.request.speaker = robot_speaker_tn;
              connect_robot_speaker_client.call(connect_robot_speaker_srv);                  
            }*/

            // --- Create publishers
            head_pose_pub           = n.advertise<geometry_msgs::PoseStamped>(robot_head_pose_tn, 1);
            left_joy_pose_pub       = n.advertise<geometry_msgs::PoseStamped>(robot_hand_L_pose_tn, 1);
            right_joy_pose_pub      = n.advertise<geometry_msgs::PoseStamped>(robot_hand_R_pose_tn, 1);
            //twist_base_pub          = n.advertise<geometry_msgs::TwistStamped>(robot_twist_base_tn, 1);
            twist_base_pub          = n.advertise<geometry_msgs::Twist>(robot_twist_base_tn, 1);
            //left_joy_closure_pub    = n.advertise<std_msgs::Float64>(left_joy_closure_tn, 1);
            //right_joy_closure_pub   = n.advertise<std_msgs::Float64>(right_joy_closure_tn, 1);
            //teaching_cmd_left_pub   = n.advertise<sensor_msgs::Joy>(teaching_cmd_left_tn, 1);
            //teaching_cmd_right_pub  = n.advertise<sensor_msgs::Joy>(teaching_cmd_right_tn, 1);

            // --- Create service clients
            /*open_stream_client = n.serviceClient<av_manager::open_stream>("/" + robot_ns + open_stream_sv);
            close_stream_client = n.serviceClient<av_manager::close_stream>("/" + robot_ns + close_stream_sv);

            // --- Call service to open the video stream
            open_stream_srv.request.ip_address = ip_address;
            open_stream_srv.request.first_port = left_camera_port;
            open_stream_srv.request.second_port = right_camera_port;
            open_stream_client.call(open_stream_srv);

            // --- Call service to open the audio stream
            open_audio_srv.request.enable_send = true;
            open_audio_srv.request.enable_receive = true;
            open_audio_srv.request.broadcast = true; //true for pilot side
            open_audio_srv.request.ip_local = ip_address;
            open_audio_srv.request.port_local = audio_local_port;
            open_audio_srv.request.ip_remote = robot_ip_list[menu_index];
            open_audio_srv.request.port_remote = audio_remote_port;
            open_audio_srv.request.broadcast_namespace = robot_ns;
            open_audio_client.call(open_audio_srv);*/
            
            state = CONNECT;
         // }
        //}
        break;

      case CONNECT: // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ CONNECT

        // --- Create new messages
        head_pose_msg.header.stamp = ros::Time::now();
        head_pose_msg.header.frame_id = "head";
        head_pose_msg.pose.position.x = head_calib2head_P.x();
        head_pose_msg.pose.position.y = head_calib2head_P.y();
        head_pose_msg.pose.position.z = head_calib2head_P.z();
        head_pose_msg.pose.orientation.x = head_calib2head_Q.x();
        head_pose_msg.pose.orientation.y = head_calib2head_Q.y();
        head_pose_msg.pose.orientation.z = head_calib2head_Q.z();
        head_pose_msg.pose.orientation.w = head_calib2head_Q.w();

        // From head quaternion to RPY
        tf::Matrix3x3(tf::Quaternion(head_calib2head_Q.x(), head_calib2head_Q.y(), head_calib2head_Q.z(), head_calib2head_Q.w())).getRPY(head_roll, head_pitch, head_yaw);

        head_joint_msg.data.clear();
        head_joint_msg.data.push_back(head_roll);
        head_joint_msg.data.push_back(head_pitch); // + 1.0); offset se si tiene il visore sulla fronte
        head_joint_msg.data.push_back(head_yaw);


        left_joy_pose_msg.header.stamp = ros::Time::now();
        left_joy_pose_msg.header.frame_id = "hand_L";
        left_joy_pose_msg.pose.position.x = shoulder2joyL_P.x();
        left_joy_pose_msg.pose.position.y = shoulder2joyL_P.y();
        left_joy_pose_msg.pose.position.z = shoulder2joyL_P.z();
        left_joy_pose_msg.pose.orientation.x = shoulder2joyL_Q.x();
        left_joy_pose_msg.pose.orientation.y = shoulder2joyL_Q.y();
        left_joy_pose_msg.pose.orientation.z = shoulder2joyL_Q.z();
        left_joy_pose_msg.pose.orientation.w = shoulder2joyL_Q.w();

        right_joy_pose_msg.header.stamp = ros::Time::now();
        right_joy_pose_msg.header.frame_id = "hand_R";
        right_joy_pose_msg.pose.position.x = shoulder2joyR_P.x();
        right_joy_pose_msg.pose.position.y = shoulder2joyR_P.y();
        right_joy_pose_msg.pose.position.z = shoulder2joyR_P.z();
        right_joy_pose_msg.pose.orientation.x = shoulder2joyR_Q.x();
        right_joy_pose_msg.pose.orientation.y = shoulder2joyR_Q.y();
        right_joy_pose_msg.pose.orientation.z = shoulder2joyR_Q.z();
        right_joy_pose_msg.pose.orientation.w = shoulder2joyR_Q.w();

        twist_base_msg.linear.x = robot_base_vel_lin.x();
        twist_base_msg.linear.y = -robot_base_vel_lin.y();
        twist_base_msg.linear.z =  robot_base_vel_lin.z();
        twist_base_msg.angular.x = robot_base_vel_ang.x();
        twist_base_msg.angular.y = robot_base_vel_ang.y();
        twist_base_msg.angular.z = -robot_base_vel_ang.z();


        //left_joy_closure_msg.data = left_hand_cl;
        
        //right_joy_closure_msg.data = right_hand_cl;

        /*teaching_cmd_left_msg.axes.clear();
        teaching_cmd_left_msg.axes.resize(2);
        teaching_cmd_left_msg.axes[0] = teaching_cmd_left_axis_x;
        teaching_cmd_left_msg.axes[1] = teaching_cmd_left_axis_y;

        teaching_cmd_right_msg.axes.clear();
        teaching_cmd_right_msg.axes.resize(2);
        teaching_cmd_right_msg.axes[0] = teaching_cmd_right_axis_x;
        teaching_cmd_right_msg.axes[1] = teaching_cmd_right_axis_y;*/

        /*if(enter_b_old == 1 && enter_b == 0 && new_msg)
        {
          new_msg = false;
          enable_cmd = !enable_cmd;
        }*/

        // FOR TEACHING PURPOSE:: TO FIX
        //teaching_cmd_left_pub.publish(teaching_left_cmd);

        head_pose_pub.publish(head_pose_msg);
        left_joy_pose_pub.publish(left_joy_pose_msg);
        right_joy_pose_pub.publish(right_joy_pose_msg);
        twist_base_pub.publish(twist_base_msg);
        neck_RPY_pub.publish(head_joint_msg);
        // --- Publish messages
        /*if(enable_cmd){
          head_pose_pub.publish(head_pose_msg);
          left_joy_pose_pub.publish(left_joy_pose_msg);
          right_joy_pose_pub.publish(right_joy_pose_msg);
          twist_base_pub.publish(twist_base_msg);
          left_joy_closure_pub.publish(left_joy_closure_msg);
          right_joy_closure_pub.publish(right_joy_closure_msg);          
          //teaching_cmd_right_pub.publish(teaching_cmd_right_msg);
        } 
        else 
        {
          // --- Robot activated message on screen
          down_menu_msg.data = "Robot movements disabled,* press enter to enable movements";
          down_menu_pub.publish(down_menu_msg);
          std::cout << "aaaaaaaaaa"<< "\n";
        }
        // --- Check if esc buttom is pressed
        if(esc_b_old == 1 && esc_b == 0 && new_msg){
          new_msg = false;
          menu_index = 0;
          n_elem = 2;
          robot_list[0] = "No";
          robot_list[1] = "Yes";

          //state = CHANGE_ROBOT;
        }*/
        
        break;

      /*case CHANGE_ROBOT: // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ CHANGE ROBOT
        // --- Move cursor
        menu_index = cursor_movement(menu_index, up_down_a, n_elem);

        // --- Print the list in a string adding the cursor and the separator
        list_menu = "";
        for(int i=0; i<n_elem; i++){
          if (menu_index == i){
            list_menu = list_menu + "> " + robot_list[i] + "*" ;
          }
          else{
            list_menu = list_menu + "  " + robot_list[i] + "*" ;
          }

        }

        // --- Publish the list
        central_menu_msg.data = list_menu;
        central_menu_pub.publish(central_menu_msg);

        down_menu_msg.data = change_robot_info_str;
        down_menu_pub.publish(down_menu_msg);

        // --- Check if enter buttom is pressed
        if(enter_b_old == 1 && enter_b == 0 && new_msg){
          new_msg = false;
          if(menu_index == n_elem-1){
            // --- Call service to close the video stream
            close_stream_srv.request.close = true;
            close_stream_client.call(close_stream_srv);

            close_audio_srv.request.broadcast = true;
            close_audio_srv.request.broadcast_namespace = robot_ns;
            close_audio_client.call(close_audio_srv);

            // --- Close all publishers
            head_pose_pub.shutdown();
            left_joy_pose_pub.shutdown();
            right_joy_pose_pub.shutdown();
            twist_base_pub.shutdown();
            left_joy_closure_pub.shutdown();
            right_joy_closure_pub.shutdown();
            teaching_cmd_left_pub.shutdown();
            teaching_cmd_right_pub.shutdown();

            // --- Close all services clients
            open_stream_client.shutdown();
            close_stream_client.shutdown();

            menu_index = 0;

            ss = stringstream(execute(find_robots_cmd));

            robot_list_refresh_timer = ros::Time::now();
          
            // --- Get elements of the list
            std::cout << "ROBOT LIST: " << std::endl;
            n_elem = 0;
            while(std::getline(ss, to, '\n')){
              robot_list[n_elem] = to;
              get_robot_info_client = n.serviceClient<robot_fwk_utilities::GetRobotInfo>("/" + robot_list[n_elem] + get_robot_info_sv);
              robot_ip_list[n_elem] = get_robot_IP(get_robot_info_client, get_robot_info_srv, robot_list[n_elem], get_robot_info_sv);
              n_elem++;
            }
            robot_list[n_elem] = "REFRESH";
            robot_list[n_elem+1] = "EXIT";

            state = SHOW_LIST;
          }
          else{
            state = CONNECT;
          }
        }
        break;*/

      case EXIT: // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ EXIT
        exit_prog = true;
        break;
    }

		ros::spinOnce();
		loop_rate.sleep();
	}
}