// imported libraries
#include "control_msgs/JointControllerState.h"
#include "geometry_msgs/PointStamped.h"

// C++ client library as ROS interface
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include <ctime>
#include <iostream>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ostream>
#include <sstream>

// define specific values for stability checks
// for instance: foot_x_size2 < CoM.x - centroid.x < foot_x_size for x-stability
// same for y direction
#define foot_x_size 0.03
#define foot_x_size2 -0.04
#define foot_y_size 0.018
#define foot_y_size2 -0.018

// point mass is defined to organized the CoM and the feet centroid
// based on the position and stance state (left or right or double)
class point_mass {
public:
  double x = 0, y = 0, z = 0, mass = 0;
  char state;
};

// check if the robot is stable in both directions x and y
// |CoM - Centroid of the stance feet| < some_value
class stability {
public:
  bool x = 0, y = 0;
  bool check() {
    if (x && y) {
      std::cout << "biped stable (balanced)" << std::endl;
    } else {
      std::cout << "WARNING!!! biped unstable. Going to FALL" << std::endl;
    }
    return x && y;
  }
};

class uthai_kd {
  // ros::NodeHandle n;

  // initialize the publishers to key topics necessary for the control of the
  // biped
  ros::Publisher pb_ankle_pitch_L, pb_ankle_roll_L, pb_hip_pitch_L,
      pb_hip_roll_L, pb_hip_yaw_L, pb_knee_pitch_L, pb_ankle_pitch_R,
      pb_ankle_roll_R, pb_hip_pitch_R, pb_hip_roll_R, pb_hip_yaw_R,
      pb_knee_pitch_R, pb_centroid, pb_centroid_x, pb_centroid_y, pb_com,
      pb_com_x, pb_com_y, pb_l_foot, pb_r_foot;

  // pointstamped left and right foot and initialization for the centroid
  geometry_msgs::PointStamped ps, ps_centroid, ps_l_foot, ps_r_foot;
  KDL::Tree uthai_tree; // initialize KDL tree for the biped
  KDL::JntArray r_jntarray;
  KDL::JntArray l_jntarray;
  double home[12] = {0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0}; // initial angular position of the joints
  point_mass humanoid_CoM, centroid; // point mass for CoM and centroid
  double r_roll, r_pitch, r_yaw, l_roll, l_pitch, l_yaw, base_roll, base_pitch,
      base_yaw; // the orientation of the biped as a whole

  ros::Subscriber read_rhip_y, read_rhip_r, read_rhip_p, read_rknee_p,
      read_rankle_p, read_rankle_r, read_lhip_y, read_lhip_r, read_lhip_p,
      read_lknee_p, read_lankle_p, read_lankle_r;

  // callback functions for the corresponding subscriptions
  void fread_rhip_y(const control_msgs::JointControllerState::ConstPtr &msg) {
    r_jntarray.data[5] = msg->process_value;
    l_jntarray.data[6] = msg->process_value;
  }
  void fread_rhip_r(const control_msgs::JointControllerState::ConstPtr &msg) {
    r_jntarray.data[4] = msg->process_value;
    l_jntarray.data[7] = msg->process_value;
  }
  void fread_rhip_p(const control_msgs::JointControllerState::ConstPtr &msg) {
    r_jntarray.data[3] = msg->process_value;
    l_jntarray.data[8] = msg->process_value;
  }
  void fread_rknee_p(const control_msgs::JointControllerState::ConstPtr &msg) {
    r_jntarray.data[2] = msg->process_value;
    l_jntarray.data[9] = msg->process_value;
  }
  void fread_rankle_p(const control_msgs::JointControllerState::ConstPtr &msg) {
    r_jntarray.data[1] = msg->process_value;
    l_jntarray.data[10] = msg->process_value;
  }
  void fread_rankle_r(const control_msgs::JointControllerState::ConstPtr &msg) {
    r_jntarray.data[0] = msg->process_value;
    l_jntarray.data[11] = msg->process_value;
  }
  void fread_lhip_y(const control_msgs::JointControllerState::ConstPtr &msg) {
    r_jntarray.data[6] = msg->process_value;
    l_jntarray.data[5] = msg->process_value;
  }
  void fread_lhip_r(const control_msgs::JointControllerState::ConstPtr &msg) {
    r_jntarray.data[7] = msg->process_value;
    l_jntarray.data[4] = msg->process_value;
  }
  void fread_lhip_p(const control_msgs::JointControllerState::ConstPtr &msg) {
    r_jntarray.data[8] = msg->process_value;
    l_jntarray.data[3] = msg->process_value;
  }
  void fread_lknee_p(const control_msgs::JointControllerState::ConstPtr &msg) {
    r_jntarray.data[9] = msg->process_value;
    l_jntarray.data[2] = msg->process_value;
  }
  void fread_lankle_p(const control_msgs::JointControllerState::ConstPtr &msg) {
    r_jntarray.data[10] = msg->process_value;
    l_jntarray.data[1] = msg->process_value;
  }
  void fread_lankle_r(const control_msgs::JointControllerState::ConstPtr &msg) {
    r_jntarray.data[11] = msg->process_value;
    l_jntarray.data[0] = msg->process_value;
  }

public:
  bool gazebo = 1;
  KDL::Chain r_leg, l_leg; // initialize the kinematic chain for both legs
  KDL::Frame r_foot,
      l_foot; // initialize the end-effector frames (left and right foot)

  double rhip_y, rhip_r, rhip_p, rknee_p, rankle_p,
      rankle_r; // right leg joints position
  double lhip_y, lhip_r, lhip_p, lknee_p, lankle_p,
      lankle_r; // left leg joints position

  // target joints angular position for each leg
  double T_rhip_y, T_rhip_r, T_rhip_p, T_rknee_p, T_rankle_p, T_rankle_r;
  double T_lhip_y, T_lhip_r, T_lhip_p, T_lknee_p, T_lankle_p, T_lankle_r;

  // the humanoid KD constructor initializing a node
  // and parsing the URDF to define the KDL tree
  // as well as the kinematic chains of the left and right leg
  uthai_kd(ros::NodeHandle *n, std::string urdf, std::string base,
           std::string r_endf, std::string l_endf) {
    if (!kdl_parser::treeFromFile(urdf, uthai_tree)) {
      std::cout << "Failed to construct kdl tree\n";
    } else {
      std::cout << "Success to construct kdl tree\n";
    }
    if (!uthai_tree.getChain(r_endf, l_endf, r_leg)) {
      std::cout << "Failed to get r_leg kinematics chain\n";
    }
    if (!uthai_tree.getChain(l_endf, r_endf, l_leg)) {
      std::cout << "Failed to get l_leg kinematics chain\n";
    } else {
      std::cout << "Success to get kinematics chain\n";
      r_jntarray = KDL::JntArray(r_leg.getNrOfJoints());
      l_jntarray = KDL::JntArray(l_leg.getNrOfJoints());
    }

    // initializing key publishers to smoothen both the control of the joints
    // and the check for stability
    pb_ankle_pitch_L = n->advertise<std_msgs::Float64>(
        "uthai/l_ankle_pitch_position/command", 10);
    pb_ankle_roll_L = n->advertise<std_msgs::Float64>(
        "uthai/l_ankle_roll_position/command", 10);
    pb_hip_pitch_L = n->advertise<std_msgs::Float64>(
        "uthai/l_hip_pitch_position/command", 10);
    pb_hip_roll_L = n->advertise<std_msgs::Float64>(
        "uthai/l_hip_roll_position/command", 10);
    pb_hip_yaw_L =
        n->advertise<std_msgs::Float64>("uthai/l_hip_yaw_position/command", 10);
    pb_knee_pitch_L = n->advertise<std_msgs::Float64>(
        "uthai/l_knee_pitch_position/command", 10);
    pb_ankle_pitch_R = n->advertise<std_msgs::Float64>(
        "uthai/r_ankle_pitch_position/command", 10);
    pb_ankle_roll_R = n->advertise<std_msgs::Float64>(
        "uthai/r_ankle_roll_position/command", 10);
    pb_hip_pitch_R = n->advertise<std_msgs::Float64>(
        "uthai/r_hip_pitch_position/command", 10);
    pb_hip_roll_R = n->advertise<std_msgs::Float64>(
        "uthai/r_hip_roll_position/command", 10);
    pb_hip_yaw_R =
        n->advertise<std_msgs::Float64>("uthai/r_hip_yaw_position/command", 10);
    pb_knee_pitch_R = n->advertise<std_msgs::Float64>(
        "uthai/r_knee_pitch_position/command", 10);

    pb_com = n->advertise<geometry_msgs::PointStamped>("uthai/com", 10);
    pb_com_x = n->advertise<std_msgs::Float64>("com/x", 10);
    pb_com_y = n->advertise<std_msgs::Float64>("com/y", 10);

    pb_centroid =
        n->advertise<geometry_msgs::PointStamped>("uthai/com_target", 10);
    pb_centroid_x = n->advertise<std_msgs::Float64>("centroid/x", 10);
    pb_centroid_y = n->advertise<std_msgs::Float64>("centroid/y", 10);

    pb_l_foot = n->advertise<geometry_msgs::PointStamped>("uthai/l_foot", 10);
    pb_r_foot = n->advertise<geometry_msgs::PointStamped>("uthai/r_foot", 10);

    // This part of the code reads the states of each joint and use callbacks to
    // save those positions
    read_rhip_y = n->subscribe("uthai/r_hip_yaw_position/state", 1000,
                               &uthai_kd::fread_rhip_y, this);
    read_rhip_r = n->subscribe("uthai/r_hip_roll_position/state", 1000,
                               &uthai_kd::fread_rhip_r, this);
    read_rhip_p = n->subscribe("uthai/r_hip_pitch_position/state", 1000,
                               &uthai_kd::fread_rhip_p, this);
    read_rknee_p = n->subscribe("uthai/r_knee_pitch_position/state", 1000,
                                &uthai_kd::fread_rknee_p, this);
    read_rankle_p = n->subscribe("uthai/r_ankle_pitch_position/state", 1000,
                                 &uthai_kd::fread_rankle_p, this);
    read_rankle_r = n->subscribe("uthai/r_ankle_roll_position/state", 1000,
                                 &uthai_kd::fread_rankle_r, this);
    read_lhip_y = n->subscribe("uthai/l_hip_yaw_position/state", 1000,
                               &uthai_kd::fread_lhip_y, this);
    read_lhip_r = n->subscribe("uthai/l_hip_roll_position/state", 1000,
                               &uthai_kd::fread_lhip_r, this);
    read_lhip_p = n->subscribe("uthai/l_hip_pitch_position/state", 1000,
                               &uthai_kd::fread_lhip_p, this);
    read_lknee_p = n->subscribe("uthai/l_knee_pitch_position/state", 1000,
                                &uthai_kd::fread_lknee_p, this);
    read_lankle_p = n->subscribe("uthai/l_ankle_pitch_position/state", 1000,
                                 &uthai_kd::fread_lankle_p, this);
    read_lankle_r = n->subscribe("uthai/l_ankle_roll_position/state", 1000,
                                 &uthai_kd::fread_lankle_r, this);
  }

  // function that sets the targeted joint pose based on the joints position
  // array
  void set_T_jointpose(double *jnt) {
    T_rhip_y = jnt[0];
    T_rhip_r = jnt[1];
    T_rhip_p = jnt[2];
    T_rknee_p = jnt[3];
    T_rankle_p = 0 - (jnt[2] + jnt[3]);
    T_rankle_r = -jnt[1];
    T_lhip_y = jnt[6];
    T_lhip_r = jnt[7];
    T_lhip_p = jnt[8];
    T_lknee_p = jnt[9];
    T_lankle_p = 0 - (jnt[8] + jnt[9]);
    T_lankle_r = -jnt[7];
  }

  // set the target joint pose with the actual state of the joint
  void set_T_equal_jointpose() {
    T_rhip_y = rhip_y;
    T_rhip_r = rhip_r;
    T_rhip_p = rhip_p;
    T_rknee_p = rknee_p;
    T_rankle_p = rankle_p;
    T_rankle_r = rankle_r;
    T_lhip_y = lhip_y;
    T_lhip_r = lhip_r;
    T_lhip_p = lhip_p;
    T_lknee_p = lknee_p;
    T_lankle_p = lankle_p;
    T_lankle_r = lankle_r;
  }

  // similar to the set_T_jointpose function,
  // it sets the joints position provided a joints array of position
  void set_jointpose(double *jnt) {
    rhip_y = jnt[0];
    rhip_r = jnt[1];
    rhip_p = jnt[2];
    rknee_p = jnt[3];
    rankle_p = 0 - (jnt[2] + jnt[3]);
    rankle_r = -jnt[1];
    lhip_y = jnt[6];
    lhip_r = jnt[7];
    lhip_p = jnt[8];
    lknee_p = jnt[9];
    lankle_p = 0 - (jnt[8] + jnt[9]);
    lankle_r = -jnt[7];
  }

  // increasing the joints position by the amount provided by an array of joints
  // position
  void add_jointpose(double *jnt) {
    rhip_y += jnt[0];
    rhip_r += jnt[1];
    rhip_p += jnt[2];
    rknee_p += jnt[3];
    rankle_p += 0 - (jnt[2] + jnt[3]);
    rankle_r += -jnt[1];
    lhip_y += jnt[6];
    lhip_r += jnt[7];
    lhip_p += jnt[8];
    lknee_p += jnt[9];
    lankle_p += 0 - (jnt[8] + jnt[9]);
    lankle_r += -jnt[7];
  }

  // this function automatically set the ankle roll and pitch based
  // on the stance state and the angular position of the hips and knees
  void auto_ankle(char state) {
    // as the robot stands on its right foot, we roll the left hip equally
    if (state == 'r')
      lhip_r = rhip_r;
    // as the robot stands on its left foot, we roll the right hip equally
    else if (state == 'l')
      rhip_r = lhip_r;

    // the ankles roll in the opposite direction of the hips.
    rankle_r = -rhip_r;
    lankle_r = -lhip_r;

    // the ankle pitch is deduced so that the sum of the pitch angles together
    // with the knee and hip is zero.
    rankle_p = 0 - (rhip_p + rknee_p);
    lankle_p = 0 - (lhip_p + lknee_p);
  }

  // changes the current kinematic chains joint angles to the wanted joints
  // position
  void set_kdjointpose() {
    r_jntarray.data[0] = rankle_r;  // r_hip yaw
    r_jntarray.data[1] = rankle_p;  // r_hip rol
    r_jntarray.data[2] = rknee_p;   // r_hip pitch
    r_jntarray.data[3] = rhip_p;    // r_knee pitch
    r_jntarray.data[4] = rhip_r;    // r_ankle pitch
    r_jntarray.data[5] = rhip_y;    // r_ankle roll
    r_jntarray.data[6] = lhip_y;    // r_hip yaw
    r_jntarray.data[7] = lhip_r;    // r_hip rol
    r_jntarray.data[8] = lhip_p;    // r_hip pitch
    r_jntarray.data[9] = lknee_p;   // r_knee pitch
    r_jntarray.data[10] = lankle_p; // r_ankle pitch
    r_jntarray.data[11] = lankle_r; // r_ankle roll

    l_jntarray.data[0] = lankle_r;  // r_hip yaw
    l_jntarray.data[1] = lankle_p;  // r_hip rol
    l_jntarray.data[2] = lknee_p;   // r_hip pitch
    l_jntarray.data[3] = lhip_p;    // r_knee pitch
    l_jntarray.data[4] = lhip_r;    // r_ankle pitch
    l_jntarray.data[5] = lhip_y;    // r_ankle roll
    l_jntarray.data[6] = rhip_y;    // r_hip yaw
    l_jntarray.data[7] = rhip_r;    // r_hip rol
    l_jntarray.data[8] = rhip_p;    // r_hip pitch
    l_jntarray.data[9] = rknee_p;   // r_knee pitch
    l_jntarray.data[10] = rankle_p; // r_ankle pitch
    l_jntarray.data[11] = rankle_r; // r_ankle roll
  }

  // publish the joint position to the appropriate topic
  void joint_publish(ros::Rate *r) {
    if (gazebo) {
      std_msgs::Float64 radi;
      radi.data = rhip_y;
      pb_hip_yaw_R.publish(radi);
      radi.data = rhip_r;
      pb_hip_roll_R.publish(radi);
      radi.data = rhip_p;
      pb_hip_pitch_R.publish(radi);
      radi.data = rknee_p;
      pb_knee_pitch_R.publish(radi);
      radi.data = rankle_p;
      pb_ankle_pitch_R.publish(radi);
      radi.data = rankle_r;
      pb_ankle_roll_R.publish(radi);
      radi.data = lhip_y;
      pb_hip_yaw_L.publish(radi);
      radi.data = lhip_r;
      pb_hip_roll_L.publish(radi);
      radi.data = lhip_p;
      pb_hip_pitch_L.publish(radi);
      radi.data = lknee_p;
      pb_knee_pitch_L.publish(radi);
      radi.data = lankle_p;
      pb_ankle_pitch_L.publish(radi);
      radi.data = lankle_r;
      pb_ankle_roll_L.publish(radi);
    }
    r->sleep();
  }

  // publish the centroid and Centor of Mass to the appropriate topics
  void com_publish(char state, ros::Rate *r) {
    ps.point.x = humanoid_CoM.x;
    ps.point.y = humanoid_CoM.y;
    ps.point.z = 0;
    if (state == 'r' || state == 'd') {
      ps.header.frame_id = "r_foot_ft_link";
      ps_centroid.header.frame_id = "r_foot_ft_link";
    } else if (state == 'l') {
      ps.header.frame_id = "l_foot_ft_link";
      ps_centroid.header.frame_id = "l_foot_ft_link";
    }
    ps_centroid.point.x = centroid.x;
    ps_centroid.point.y = centroid.y;
    ps_centroid.point.z = ps.point.z;
    ps_l_foot.header.frame_id = "r_foot_ft_link";
    ps_l_foot.point.x = l_foot.p.data[0];
    ps_l_foot.point.y = l_foot.p.data[1];
    ps_l_foot.point.z = l_foot.p.data[2];
    ps_r_foot.header.frame_id = "l_foot_ft_link";
    ps_r_foot.point.x = r_foot.p.data[0];
    ps_r_foot.point.y = r_foot.p.data[1];
    ps_r_foot.point.z = r_foot.p.data[2];
    pb_com.publish(ps);
    pb_centroid.publish(ps_centroid);
    pb_l_foot.publish(ps_l_foot);
    pb_r_foot.publish(ps_r_foot);

    std_msgs::Float64 centroid_x, com_x, centroid_y, com_y;
    centroid_x.data = centroid.x;
    com_x.data = humanoid_CoM.x;
    centroid_y.data = centroid.y;
    com_y.data = humanoid_CoM.y;
    pb_com_x.publish(com_x);
    pb_centroid_x.publish(centroid_x);
    pb_com_y.publish(com_y);
    pb_centroid_y.publish(centroid_y);
    r->sleep();
  }

  // defines and compute the center of masss based on the stance of the biped
  // with each leg lengths and masses
  point_mass compute_com(char state,
                         KDL::ChainFkSolverPos_recursive *r_leg_fksolver,
                         KDL::ChainFkSolverPos_recursive *l_leg_fksolver,
                         bool verbose = 0) {
    point_mass r_CoM, l_CoM;
    if (state == 'r' || state == 'd') {
      for (int i = 0; i < r_leg.getNrOfSegments(); i++) {
        r_leg_fksolver->JntToCart(r_jntarray, l_foot, i + 1);
        double r_roll, r_pitch, r_yaw;
        l_foot.M.GetRPY(r_roll, r_pitch, r_yaw);

        KDL::Rotation rot_inv = l_foot.M.Inverse();
        KDL::Vector link_cog = r_leg.getSegment(i).getInertia().getCOG();
        KDL::Vector link_cog_refbase;
        KDL::Vector link_temp = rot_inv.operator*(l_foot.p);
        link_cog_refbase.data[0] = link_temp.data[0] + link_cog.data[0];
        link_cog_refbase.data[1] = link_temp.data[1] + link_cog.data[1];
        link_cog_refbase.data[2] = link_temp.data[2] + link_cog.data[2];
        link_cog_refbase.operator=(l_foot.M.operator*(link_cog_refbase));
        r_CoM.x += link_cog_refbase.data[0] *
                   r_leg.getSegment(i).getInertia().getMass();
        r_CoM.y += link_cog_refbase.data[1] *
                   r_leg.getSegment(i).getInertia().getMass();
        r_CoM.z += link_cog_refbase.data[2] *
                   r_leg.getSegment(i).getInertia().getMass();

        if (i == 6) {
          base_roll = r_roll;
          base_pitch = r_pitch;
          base_yaw = r_yaw;
        }

        r_CoM.mass += r_leg.getSegment(i).getInertia().getMass();
        if (verbose) {
          std::cout << std::setprecision(5) << i << ".) state = "
                    << "  " << r_leg.getSegment(i).getName()
                    << " mass = " << r_leg.getSegment(i).getInertia().getMass()
                    << "        Rotation : " << r_roll << "  " << r_pitch
                    << "  " << r_yaw << "           Mass Trans : " << r_CoM.x
                    << "  " << r_CoM.y << "  " << r_CoM.z << ""
                    << "       Translation main : " << l_foot.p[0] << "  "
                    << l_foot.p[1] << "  " << l_foot.p[2] << "\n";
        }
        humanoid_CoM.x = r_CoM.x / r_CoM.mass;
        humanoid_CoM.y = r_CoM.y / r_CoM.mass;
      }
    }
    if (state == 'l') {
      for (int i = 0; i < l_leg.getNrOfSegments(); i++) {
        l_leg_fksolver->JntToCart(l_jntarray, r_foot, i + 1);
        double l_roll, l_pitch, l_yaw;
        r_foot.M.GetRPY(l_roll, l_pitch, l_yaw);

        KDL::Rotation rot_inv = r_foot.M.Inverse();
        KDL::Vector link_cog = l_leg.getSegment(i).getInertia().getCOG();
        KDL::Vector link_cog_refbase;
        KDL::Vector link_temp = rot_inv.operator*(r_foot.p);
        link_cog_refbase.data[0] = link_temp.data[0] + link_cog.data[0];
        link_cog_refbase.data[1] = link_temp.data[1] + link_cog.data[1];
        link_cog_refbase.data[2] = link_temp.data[2] + link_cog.data[2];
        link_cog_refbase.operator=(r_foot.M.operator*(link_cog_refbase));
        l_CoM.x += link_cog_refbase.data[0] *
                   l_leg.getSegment(i).getInertia().getMass();
        l_CoM.y += link_cog_refbase.data[1] *
                   l_leg.getSegment(i).getInertia().getMass();
        l_CoM.z += link_cog_refbase.data[2] *
                   l_leg.getSegment(i).getInertia().getMass();

        l_CoM.mass += l_leg.getSegment(i).getInertia().getMass();
        if (verbose) {
          std::cout << std::setprecision(5) << i << ".) state = "
                    << "  " << l_leg.getSegment(i).getName()
                    << " mass = " << l_leg.getSegment(i).getInertia().getMass()
                    << "        Rotation : " << l_roll << "  " << l_pitch
                    << "  " << l_yaw << "           Mass Trans : " << l_CoM.x
                    << "  " << l_CoM.y << "  " << l_CoM.z << ""
                    << "       Translation main : " << r_foot.p[0] << "  "
                    << r_foot.p[1] << "  " << r_foot.p[2] << "\n";
        }
        humanoid_CoM.x = l_CoM.x / l_CoM.mass;
        humanoid_CoM.y = l_CoM.y / l_CoM.mass;
      }
    }
    return humanoid_CoM;
  }

  // computes the centroid based on the stance and the feet position on ground
  point_mass compute_centroid(char state, bool verbose = 0) {
    if (state == 'r' || state == 'l') // R leg is stance leg
    {
      centroid.x = 0;
      centroid.y = 0;
      centroid.z = 0;
    } else if (state == 'd') // R is main leg for double support
    {
      centroid.x = l_foot.p.data[0] / 2.0;
      centroid.y = l_foot.p.data[1] / 2.0;
      centroid.z = l_foot.p.data[2] / 2.0;
    }
    centroid.state = state;
    if (verbose)
      std::cout << centroid.state << " centroid : (" << centroid.x << ", "
                << centroid.y << ", " << centroid.z << ")\n";
    return centroid;
  }

  // checks for stability based on the foot stance
  // and the computed centroid and Center of Mass
  stability quick_is_stable(char state, bool verbose = 0) {

    stability out;
    if (verbose)
      std::cout << "Humanoid_CoM : " << humanoid_CoM.x << ", " << humanoid_CoM.y
                << "\n";
    if (centroid.x + foot_x_size2 < humanoid_CoM.x &&
        humanoid_CoM.x < centroid.x + foot_x_size) {
      out.x = true;
      if (verbose)
        std::cout << " x is stable\n";
      if (centroid.y + foot_y_size2 < humanoid_CoM.y &&
          humanoid_CoM.y < centroid.y + foot_y_size) {
        out.y = true;
        if (verbose)
          std::cout << " y is stable\n";
      }
    }
  }

  // given the foot stance, the forward kinematic chain of the left and right
  // leg it computes the difference between the target joint pose and the actual
  // state and d
  bool uthai_will_go_on(char state, KDL::ChainFkSolverPos_recursive *r_fksolver,
                        KDL::ChainFkSolverPos_recursive *l_fksolver,
                        ros::Rate *rate, int freq = 200) {
    // divide the difference between the target joint pose with the current pose
    // by a frequency
    double jntstate[] = {
        (T_rhip_y - rhip_y) / freq,     (T_rhip_r - rhip_r) / freq,
        (T_rhip_p - rhip_p) / freq,     (T_rknee_p - rknee_p) / freq,
        (T_rankle_p - rankle_p) / freq, (T_rankle_r - rankle_r) / freq,
        (T_lhip_y - lhip_y) / freq,     (T_lhip_r - lhip_r) / freq,
        (T_lhip_p - lhip_p) / freq,     (T_lknee_p - lknee_p) / freq,
        (T_lankle_p - lankle_p) / freq, (T_lankle_r - lankle_r) / freq};

    stability humanoid_stability;
    bool stable = false;

    // define PID coefficients per frequency
    double Kp = 20.0 / freq, Ki = 1.5 / freq, Kd = 0.0 / freq, e_x = 0.0,
           e_y = 0.0, de_x = 0.0, de_y = 0.0, ie_x = 0.0, ie_y = 0.0,
           P_limit = 0.2 / freq, I_limit = 4.0 / freq,
           D_limit = 200000.0 / freq;

    // for each frequency, add to the current joint pose the defined joint state
    // per frequency so that to reach the target angle at the end.
    for (int k = 0; k < freq; k++) {
      add_jointpose(jntstate); // add to the joint current pose
      auto_ankle(state);       // set the ankle position and orientation
      set_kdjointpose(); // set the kinematic chain joints pose to the resulting
                         // current pose

      // compute the centor of mass and centroid
      humanoid_CoM = compute_com(state, r_fksolver, l_fksolver, 0);
      centroid = compute_centroid(state, 0);

      // check for stability given the current state and joints rotation
      humanoid_stability = quick_is_stable(state, 0);
      stable = humanoid_stability.check();

      // compute the error and its derivative
      // add the individual errors to constitute the integral for the PID
      // control
      de_x = (humanoid_CoM.x - centroid.x) - e_x;
      de_y = (humanoid_CoM.y - centroid.y) - e_y;
      e_x = humanoid_CoM.x - centroid.x;
      e_y = humanoid_CoM.y - centroid.y;
      ie_x += e_x;
      ie_y += e_y;
      double P_x = (Kp * e_x), P_y = (Kp * e_y), I_x = (Ki * ie_x),
             I_y = (Ki * ie_y), D_x = (Kd * de_x), D_y = (Kd * de_y);
      if (P_x > P_limit)
        P_x = P_limit;
      else if (P_x < -P_limit)
        P_x = -P_limit;
      if (P_y > P_limit)
        P_y = P_limit;
      else if (P_y < -P_limit)
        P_y = -P_limit;
      if (ie_x > I_limit)
        ie_x = I_limit;
      else if (ie_x < -I_limit)
        ie_x = -I_limit;
      if (ie_y > I_limit)
        ie_y = I_limit;
      else if (ie_y < -I_limit)
        ie_y = -I_limit;

      if (D_x > D_limit)
        D_x = D_limit;
      else if (D_x < -D_limit)
        D_x = -D_limit;
      if (D_y > D_limit)
        D_y = D_limit;
      else if (D_y < -D_limit)
        D_y = -D_limit;
      // the joint poses are derived with PID based on the foot position and
      // stance
      if (state == 'r') {
        rhip_r += P_y + I_y + D_y;
        rhip_p -= P_x + I_x + D_x;
        if (l_foot.p.data[1] < 0.16) {
          lhip_r += Kp / 3 * (0.16 - l_foot.p.data[1]);
        }
      } else if (state == 'l') {
        lhip_r += P_y + I_y + D_y;
        lhip_p -= P_x + I_x + D_x;

        if (r_foot.p.data[1] > -0.16) {

          rhip_r -= Kp / 3 * (-r_foot.p.data[1] + 0.16);
        }
      } else if (state == 'd') {
        rhip_r += (P_y + I_y + D_y);
        rhip_p -= (P_x + I_x + D_x);
        lhip_r += (P_y + I_y + D_y);
        lhip_p -= (P_x + I_x + D_x);
      }

      joint_publish(rate); // publish the joints pose to the topics for motion
      com_publish(state, rate); // publish the center of mass and centroid
    }
    set_T_equal_jointpose(); // save the resulting state of the joints to the
                             // target joint pose
  }

  // similarly to the previous function, it computes the range of remaining
  // angular displacement to reach the target and uses PID setup to move the leg
  // accordingly
  bool moveleg(double *target, char state,
               KDL::ChainFkSolverPos_recursive *r_fksolver,
               KDL::ChainFkSolverPos_recursive *l_fksolver, ros::Rate *rate,
               int freq = 200) {

    double jntstate[] = {
        (T_rhip_y - rhip_y) / freq,     (T_rhip_r - rhip_r) / freq,
        (T_rhip_p - rhip_p) / freq,     (T_rknee_p - rknee_p) / freq,
        (T_rankle_p - rankle_p) / freq, (T_rankle_r - rankle_r) / freq,
        (T_lhip_y - lhip_y) / freq,     (T_lhip_r - lhip_r) / freq,
        (T_lhip_p - lhip_p) / freq,     (T_lknee_p - lknee_p) / freq,
        (T_lankle_p - lankle_p) / freq, (T_lankle_r - lankle_r) / freq};

    stability humanoid_stability;
    bool stable = false;
    double Kp = 30.0 / freq, Ki = 1.5 / freq, Kd = 0.0 / freq, e_x = 0.0,
           e_y = 0.0, de_x = 0.0, de_y = 0.0, ie_x = 0.0, ie_y = 0.0,
           te_x = 0.0, te_y = 0.0, te_z = 0.0, P_limit = 0.2 / freq,
           I_limit = 4.0 / freq, D_limit = 200000.0 / freq;

    for (int k = 0; k < freq; k++) {
      add_jointpose(jntstate); // add to the current joint pose, the range of
                               // motion per frequency

      // just like the auto_ankle function, set the ankle roll and pitch for
      // motion
      rankle_r = -rhip_r;
      lankle_r = -lhip_r;
      rankle_p = 0 - (rhip_p + rknee_p);
      lankle_p = 0 - (lhip_p + lknee_p);

      set_kdjointpose(); // set the kinematic chains joint pose

      // compute the resulting center of mass and centroid
      humanoid_CoM = compute_com(state, r_fksolver, l_fksolver, 0);
      centroid = compute_centroid(state, 0);

      if (state == 'l')
        centroid.y -= 0.02;
      if (state == 'r')
        centroid.y += 0.02;
      // check if its stable
      humanoid_stability = quick_is_stable(state, 0);
      stable = humanoid_stability.check();

      de_x = (humanoid_CoM.x - centroid.x) - e_x;
      de_y = (humanoid_CoM.y - centroid.y) - e_y;
      e_x = humanoid_CoM.x - centroid.x;
      e_y = humanoid_CoM.y - centroid.y;
      ie_x += e_x;
      ie_y += e_y;
      double P_x = (Kp * e_x), P_y = (Kp * e_y), I_x = (Ki * ie_x),
             I_y = (Ki * ie_y), D_x = (Kd * de_x), D_y = (Kd * de_y);
      if (P_x > P_limit)
        P_x = P_limit;
      else if (P_x < -P_limit)
        P_x = -P_limit;
      if (P_y > P_limit)
        P_y = P_limit;
      else if (P_y < -P_limit)
        P_y = -P_limit;
      if (ie_x > I_limit)
        ie_x = I_limit;
      else if (ie_x < -I_limit)
        ie_x = -I_limit;
      if (ie_y > I_limit)
        ie_y = I_limit;
      else if (ie_y < -I_limit)
        ie_y = -I_limit;

      if (D_x > D_limit)
        D_x = D_limit;
      else if (D_x < -D_limit)
        D_x = -D_limit;
      if (D_y > D_limit)
        D_y = D_limit;
      else if (D_y < -D_limit)
        D_y = -D_limit;

      // based on the feet position and target feet position, as well as the
      // stance computes the error with respect to the realtime position
      // and defines the joints position to correct it (PID)
      if (state == 'r') {
        te_x = l_foot.p.data[0] - target[0];
        te_y = l_foot.p.data[1] - target[1];
        te_z = l_foot.p.data[2] - target[2];
        rhip_r += P_y + I_y + D_y;
        rhip_p -= P_x + I_x + D_x;
        if (l_foot.p.data[1] < 0.16) {

          lhip_r += Kp / 3 * (0.16 - l_foot.p.data[1]);
        }

        lhip_r -= Kp * te_y;
        lhip_p += Kp * te_x;
        lknee_p -= Kp * te_z;
      } else if (state == 'l') {
        te_x = r_foot.p.data[0] - target[0];
        te_y = r_foot.p.data[1] - target[1];
        te_z = r_foot.p.data[2] - target[2];
        lhip_r += P_y + I_y + D_y;
        lhip_p -= P_x + I_x + D_x;
        if (r_foot.p.data[1] > -0.16) {

          rhip_r -= Kp / 3 * (-r_foot.p.data[1] + 0.16);
        }

        rhip_r -= Kp * te_y;
        rhip_p += Kp * te_x;
        rknee_p -= Kp * te_z;
      } else if (state == 'd') {
        rhip_r += (P_y + I_y + D_y);
        rhip_p -= (P_x + I_x + D_x);
        lhip_r += (P_y + I_y + D_y);
        lhip_p -= (P_x + I_x + D_x);
      }

      joint_publish(rate); // publish the joints pose to the topics for motion
      com_publish(state, rate); // publish the center of mass and centroid
    }
    set_T_equal_jointpose();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "kd_passion_challenge");
  ros::NodeHandle nh;
  ros::Rate rate(100); // you can change the value
  std::string urdf_file;
  nh.getParam("urdf_file", urdf_file);
  uthai_kd uthai(&nh, urdf_file, "base_link", "r_foot_ft_link",
                 "l_foot_ft_link");
  KDL::ChainFkSolverPos_recursive r_fksolver(uthai.r_leg);
  KDL::ChainFkSolverPos_recursive l_fksolver(uthai.l_leg);

  int sampling = 200; // you can change the value

  /* ----------------------------------------------------- */

  // solution
  uthai.uthai_will_go_on('d', &r_fksolver, &l_fksolver, &rate, 150);
  for (int i = 0; i < 2; i++) {

    uthai.T_rhip_p = -0.3;
    uthai.T_rknee_p = 0.55;
    uthai.T_lhip_p = -0.2;
    uthai.T_lknee_p = 0.5;

    uthai.uthai_will_go_on('l', &r_fksolver, &l_fksolver, &rate, sampling);
    double tfoot[] = {0.1, -0.15, 0};
    uthai.moveleg(tfoot, 'l', &r_fksolver, &l_fksolver, &rate, sampling * 2);
    uthai.uthai_will_go_on('d', &r_fksolver, &l_fksolver, &rate,
                           sampling * 1.5);

    uthai.T_lhip_p = -0.3;
    uthai.T_lknee_p = 0.55;
    uthai.T_rhip_p = -0.2;
    uthai.T_rknee_p = 0.5;

    uthai.uthai_will_go_on('r', &r_fksolver, &l_fksolver, &rate, sampling);
    tfoot[1] = 0.15;
    uthai.moveleg(tfoot, 'r', &r_fksolver, &l_fksolver, &rate, sampling * 2);
    uthai.uthai_will_go_on('d', &r_fksolver, &l_fksolver, &rate,
                           sampling * 1.5);
  }
  // end solution

  /* ----------------------------------------------------- */
  std::cout << "DONE!" << std::endl;
  double home[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uthai.set_jointpose(home);
  uthai.set_T_jointpose(home);
  uthai.uthai_will_go_on('d', &r_fksolver, &l_fksolver, &rate, 70);

  uthai.T_rhip_p = -0.3;
  uthai.T_rknee_p = 0.55;
  uthai.T_lhip_p = -0.3;
  uthai.T_lknee_p = 0.55;
  uthai.uthai_will_go_on('d', &r_fksolver, &l_fksolver, &rate, 100);
  std::cout << "DONE!" << std::endl;
  std::clock_t begin = clock();
  ros::spin();
  std::clock_t end = clock();
  std::cout << "elapsed time is " << double(end - begin) / CLOCKS_PER_SEC
            << "\n";
  return 1;
}
