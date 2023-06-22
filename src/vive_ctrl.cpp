#include <cmath>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JoyFeedback.h>
#include <tf/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "vive_ros/vr_interface.h"
#include <vive_ros/devices.h>

// void handleDebugMessages(const std::string &msg) { ROS_DEBUG(" [VIVE] %s", msg.c_str()); }
// void handleInfoMessages(const std::string &msg) { ROS_INFO(" [VIVE] %s", msg.c_str()); }
// void handleErrorMessages(const std::string &msg) { ROS_ERROR(" [VIVE] %s", msg.c_str()); }

enum
{
  X,
  Y,
  XY
};
enum
{
  L,
  R,
  LR
};

class VIVEnode
{
public:
  VIVEnode(int rate);
  ~VIVEnode();
  bool Init();
  void Run();
  void Shutdown();
  // bool setOriginCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  // bool listDevicesCB(vive_ros::list_devices::Request &req, vive_ros::list_devices::Response &res);
  // void set_feedback(sensor_msgs::JoyFeedbackConstPtr msg);
  ros::NodeHandle nh_;
  VRInterface vr_;

private:
  ros::Rate loop_rate_;
  std::vector<double> world_offset_;
  double world_yaw_;
  tf::TransformBroadcaster tf_broadcaster_;
  // ros::ServiceServer set_origin_server_;
  ros::ServiceServer list_devices_server_;
  std::map<std::string, ros::Publisher> button_states_pubs_map;
  std::map<std::string, ros::Publisher> hmd_pose_pubs_map;
  std::map<std::string, ros::Publisher> joystick_pose_pubs_map;
  std::map<std::string, ros::Publisher> tracker_pose_pubs_map;
  std::map<std::string, ros::Publisher> lighthouse_pose_pubs_map;
  ros::Publisher hmd_pose;
  ros::Publisher device_list_pub_;

  // ros::Publisher tracker_pose;
  // ros::Publisher lighthouse_pose;
  // ros::Subscriber feedback_sub_;
};

VIVEnode::VIVEnode(int rate)
    : loop_rate_(rate), nh_("~"), tf_broadcaster_(), vr_(), world_offset_({0, 0, 0}), world_yaw_(0)
{
  nh_.getParam("/vive/world_offset", world_offset_);
  nh_.getParam("/vive/world_yaw", world_yaw_);
  // ROS_INFO(" [VIVE] World offset: [%2.3f , %2.3f, %2.3f] %2.3f", world_offset_[0], world_offset_[1], world_offset_[2], world_yaw_);
  // set_origin_server_ = nh_.advertiseService("/vive/set_origin", &VIVEnode::setOriginCB, this);
  // list_devices_server_ = nh_.advertiseService("list_devices", &VIVEnode::listDevicesCB, this);
  // device_list_pub_ = nh_.advertiseService("list_devices", &VIVEnode::listDevicesCB, this);
  device_list_pub_ = nh_.advertise<vive_ros::devices>("devices_list", 1);
  // feedback_sub_ = nh_.subscribe("/vive/set_feedback", 10, &VIVEnode::set_feedback, this);
  return;
}

VIVEnode::~VIVEnode()
{
  return;
}

bool VIVEnode::Init()
{
  // //  Set logging functions
  // vr_.setDebugMsgCallback(handleDebugMessages);
  // vr_.setInfoMsgCallback(handleInfoMessages);
  // vr_.setErrorMsgCallback(handleErrorMessages);

  if (!vr_.Init())
  {
    return false;
  }

  return true;
}

void VIVEnode::Shutdown()
{
  vr_.Shutdown();
}

// bool VIVEnode::listDevicesCB(vive_ros::list_devices::Request &req, vive_ros::list_devices::Response &res)
// {
//   res.hmd.push_back("DAJE");
//   res.controller.push_back("DAJE");
//   res.tracker.push_back("DAJE");
//   res.lighthouse.push_back("DAJE");
//   std::cout << "DAJE" << std::endl;
//   return true;
// }

// bool VIVEnode::setOriginCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
// {
//   double tf_matrix[3][4];
//   int index = 1, dev_type;
//   while (dev_type != 2)
//   {
//     dev_type = vr_.GetDeviceMatrix(index++, tf_matrix);
//   }
//   if (dev_type == 0)
//   {
//     ROS_WARN(" [VIVE] Coulnd't find controller 1.");
//     return false;
//   }

//   tf::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
//                            tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
//                            tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);
//   tf::Vector3 c_z;
//   c_z = rot_matrix * tf::Vector3(0, 0, 1);
//   c_z[1] = 0;
//   c_z.normalize();
//   double new_yaw = acos(tf::Vector3(0, 0, 1).dot(c_z)) + M_PI_2;
//   if (c_z[0] < 0)
//     new_yaw = -new_yaw;
//   world_yaw_ = -new_yaw;

//   tf::Vector3 new_offset;
//   tf::Matrix3x3 new_rot;
//   new_rot.setRPY(0, 0, world_yaw_);
//   new_offset = new_rot * tf::Vector3(-tf_matrix[0][3], tf_matrix[2][3], -tf_matrix[1][3]);

//   world_offset_[0] = new_offset[0];
//   world_offset_[1] = new_offset[1];
//   world_offset_[2] = new_offset[2];

//   nh_.setParam("/vive/world_offset", world_offset_);
//   nh_.setParam("/vive/world_yaw", world_yaw_);

//   return true;
// }

// void VIVEnode::set_feedback(sensor_msgs::JoyFeedbackConstPtr msg)
// {
//   if (msg->type == 1 /* TYPE_RUMBLE */)
//   {
//     vr_.TriggerHapticPulse(msg->id, 0, (int)(msg->intensity));
//     for (int i = 0; i < 16; i++)
//       vr_.TriggerHapticPulse(i, 0, (int)(msg->intensity));
//   }
// }

void VIVEnode::Run()
{
  int run_hz_count = 0;

  // get only namespace string without node name
  std::string fullpath = nh_.getNamespace();
  int beginIdx = fullpath.rfind('/');
  std::string ns = fullpath.substr(0, beginIdx);
  vive_ros::devices devices;
  while (ros::ok())
  {
    // do stuff
    vr_.Update();

    int controller_count = 1;
    int tracker_count = 1;
    int lighthouse_count = 1;

    devices.hmd.clear();
    devices.controller.clear();
    devices.tracker.clear();
    devices.lighthouse.clear();

    for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
    {
      double tf_matrix[3][4];

      int dev_type = vr_.GetDeviceMatrix(i, tf_matrix);

      // No device
      if (dev_type == 0)
        continue;

      tf::Transform tf;
      geometry_msgs::PoseStamped pose_msg;

      tf.setOrigin(tf::Vector3(-tf_matrix[2][3], -tf_matrix[0][3], tf_matrix[1][3]));

      tf::Quaternion quat;
      tf::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
                               tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
                               tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);

      rot_matrix.getRotation(quat);

      quat[3] = sqrt(fmax(0, 1 + rot_matrix[0][0] + rot_matrix[1][1] + rot_matrix[2][2])) / 2;
      quat[0] = sqrt(fmax(0, 1 + rot_matrix[0][0] - rot_matrix[1][1] - rot_matrix[2][2])) / 2;
      quat[1] = sqrt(fmax(0, 1 - rot_matrix[0][0] + rot_matrix[1][1] - rot_matrix[2][2])) / 2;
      quat[2] = sqrt(fmax(0, 1 - rot_matrix[0][0] - rot_matrix[1][1] + rot_matrix[2][2])) / 2;
      quat[0] = copysign(quat[0], rot_matrix[2][1] - rot_matrix[1][2]);
      quat[1] = copysign(quat[1], rot_matrix[0][2] - rot_matrix[2][0]);
      quat[2] = copysign(quat[2], rot_matrix[1][0] - rot_matrix[0][1]);

      float temp = quat[2];
      quat[2] = quat[1];
      quat[1] = -quat[0];
      quat[0] = -temp;

      tf.setRotation(quat);
      pose_msg.header.stamp = ros::Time::now();
      pose_msg.header.frame_id = "world";

      pose_msg.pose.position.x = tf.getOrigin().x();
      pose_msg.pose.position.y = tf.getOrigin().y();
      pose_msg.pose.position.z = tf.getOrigin().z();

      pose_msg.pose.orientation.x = quat[0];
      pose_msg.pose.orientation.y = quat[1];
      pose_msg.pose.orientation.z = quat[2];
      pose_msg.pose.orientation.w = quat[3];

      // get device serial number
      std::string cur_sn = vr_.GetTrackedDeviceString(vr_.pHMD_, i, vr::Prop_SerialNumber_String);
      std::replace(cur_sn.begin(), cur_sn.end(), '-', '_');
      // It's a HMD
      if (dev_type == 1)
      {
        devices.hmd.push_back(ns + "/hmd_" + cur_sn);
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world", "hmd"));
        hmd_pose = nh_.advertise<geometry_msgs::PoseStamped>("/" + ns + "/hmd_as_posestamped", 10); // assegnamento ad ogni ciclo, si può tirare fuori
        hmd_pose.publish(pose_msg);
      }
      // It's a controller
      else if (dev_type == 2)
      {
        devices.controller.push_back(ns + "/controller_" + cur_sn);

        // tracker
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world", "controller_" + cur_sn));

        vr::VRControllerState_t state;
        vr_.HandleInput(i, state);
        sensor_msgs::Joy joy;
        joy.header.stamp = ros::Time::now();
        joy.header.frame_id = "controller_" + cur_sn;
        joy.buttons.assign(BUTTON_NUM, 0);
        joy.axes.assign(AXES_NUM, 0.0); // x-axis, y-axis
        if ((1LL << vr::k_EButton_ApplicationMenu) & state.ulButtonPressed)
          joy.buttons[0] = 1;
        if ((1LL << vr::k_EButton_SteamVR_Trigger) & state.ulButtonPressed)
          joy.buttons[1] = 1;
        if ((1LL << vr::k_EButton_SteamVR_Touchpad) & state.ulButtonPressed)
          joy.buttons[2] = 1;
        if ((1LL << vr::k_EButton_Grip) & state.ulButtonPressed)
          joy.buttons[3] = 1;
        // TrackPad's axis
        joy.axes[0] = state.rAxis[0].x;
        joy.axes[1] = state.rAxis[0].y;
        // Trigger's axis
        joy.axes[2] = state.rAxis[1].x;

        // JOY buttons and axis PUBLISHER
        if (button_states_pubs_map.count(cur_sn) == 0)
        {
          button_states_pubs_map[cur_sn] = nh_.advertise<sensor_msgs::Joy>("/" + ns + "/controller_" + cur_sn + "/joy", 10);
        }
        button_states_pubs_map[cur_sn].publish(joy);

        // JOY pose PUBLISHER
        if (joystick_pose_pubs_map.count(cur_sn) == 0)
        {
          joystick_pose_pubs_map[cur_sn] = nh_.advertise<geometry_msgs::PoseStamped>("/" + ns + "/controller_" + cur_sn + "_as_posestamped", 10);
        }
        pose_msg.header.stamp = ros::Time::now();
        joystick_pose_pubs_map[cur_sn].publish(pose_msg);
      }
      // It's a tracker
      if (dev_type == 3)
      {
        devices.tracker.push_back(ns + "/tracker_" + cur_sn);

        if (tracker_pose_pubs_map.count(cur_sn) == 0)
        {
          tracker_pose_pubs_map[cur_sn] = nh_.advertise<geometry_msgs::PoseStamped>("/" + ns + "/tracker_" + cur_sn + "_as_posestamped", 10);
        }
        pose_msg.header.stamp = ros::Time::now();
        tracker_pose_pubs_map[cur_sn].publish(pose_msg);

        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world", "tracker_" + cur_sn));
      }

      // It's a lighthouse
      if (dev_type == 4)
      {
        devices.lighthouse.push_back(ns + "/lighthouse_" + cur_sn);

        // LIGHTHOUSE pose PUBLISHER
        if (lighthouse_pose_pubs_map.count(cur_sn) == 0)
        {
          lighthouse_pose_pubs_map[cur_sn] = nh_.advertise<geometry_msgs::PoseStamped>("/" + ns + "/lighthouse_" + cur_sn + "_as_posestamped", 10);
        }
        pose_msg.header.stamp = ros::Time::now();
        lighthouse_pose_pubs_map[cur_sn].publish(pose_msg);

        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world", "lighthouse_" + cur_sn));
      }
    }

    device_list_pub_.publish(devices);

    // Publish corrective transform
    tf::Transform tf_world;
    tf_world.setOrigin(tf::Vector3(world_offset_[0], world_offset_[1], world_offset_[2]));
    tf::Quaternion quat_world;
    quat_world.setRPY(0, 0, 0);
    tf_world.setRotation(quat_world);

    // ROS_INFO_THROTTLE(1.0, "\rController frequency @ %d [fps]", [](int &cin)
    //                   {int ans = cin; cin=0; return ans; }(run_hz_count));
    
    run_hz_count++;
    ros::spinOnce();
    loop_rate_.sleep();
  }
}

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "vive_node");

  VIVEnode nodeApp(90); // VIVE display max fps
  if (!nodeApp.Init())
  {
    nodeApp.Shutdown();
    return 1;
  }

  nodeApp.Run();
  nodeApp.Shutdown();

  return 0;
};