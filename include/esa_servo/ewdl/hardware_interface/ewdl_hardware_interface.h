#ifndef ESA_EWDL_HARDWARE_INTERFACE_H
#define ESA_EWDL_HARDWARE_INTERFACE_H
// STL
#include <string>
#include <vector>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// controller_manager
#include <controller_manager/controller_manager.h>
// hardware_interface
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
// SOEM
#include "ethercat.h"
// esa_servo
#include "esa_servo/ewdl/ethercat/master.h"


namespace esa { namespace ewdl {

static const double POSITION_STEP_FACTOR = 10000.0;
static const double VELOCITY_STEP_FACTOR = 10000.0;


class EWDL_HardwareInterface : public hardware_interface::RobotHW {
protected:
  ros::NodeHandle node;

  double loop_hz;
  std::vector<std::string> joint_names;

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface pos_joint_interface;
  hardware_interface::VelocityJointInterface vel_joint_interface;

  std::vector<double> joint_pos, joint_pos_cmd;
  std::vector<double> joint_vel, joint_vel_cmd;
  std::vector<double> joint_eff, joint_eff_cmd;

  std::vector<double> joint_lower_limits;
  std::vector<double> joint_upper_limits;


  esa::ewdl::ethercat::Master ec_master;

public:


  EWDL_HardwareInterface(ros::NodeHandle &node) : node(node) { }


  bool init_ethercat()
  {
    std::string ifname;
    std::vector<std::string> slaves;

    if (!node.getParam("ethercat/ifname", ifname))
    {
      ROS_FATAL("EtherCAT master network interface not defined.");
      return false;
    }

    if (!node.getParam("ethercat/slaves", slaves))
    {
      ROS_FATAL("EtherCAT slaves not defined.");
      return false;
    }

    ec_master = esa::ewdl::ethercat::Master(ifname, slaves);
    if (!ec_master.init())
    {
      ROS_FATAL("Failed to initialize EtherCAT master.");
      return false;
    }

    // EtherCAT Master
    ROS_INFO("EtherCAT Master network interface: %s", ifname.c_str());

    for (int i=0; i < slaves.size(); i++)
    {
      ROS_INFO("EtherCAT Slave[%d]: %s", i+1, slaves[i].c_str());
    }

    return true;
  }


  bool init()
  {
    if (!init_ethercat())
    {
      ROS_FATAL("Failed to initialize EtherCAT communication!");
      return false;
    }

    if (!node.getParam("/EWDL/hardware_interface/loop_hz", loop_hz))
    {
      ROS_FATAL("Hardware Interface: 'loop_hz' parameter not defined");
      return false;
    }

    if (!node.getParam("/EWDL/hardware_interface/joints", joint_names))
    {
      ROS_FATAL("Hardware Interface: 'joints' parameter not defined");
      return false;
    }

    int n_joints = joint_names.size();

    joint_pos.resize(n_joints, 0.0);
    joint_vel.resize(n_joints, 0.0);
    joint_eff.resize(n_joints, 0.0);

    joint_pos_cmd.resize(n_joints, 0.0);
    joint_vel_cmd.resize(n_joints, 0.0);
    joint_eff_cmd.resize(n_joints, 0.0);

    for (int i=0; i < n_joints; i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &joint_pos[i], &joint_vel[i], &joint_eff[i]);
      joint_state_interface.registerHandle(joint_state_handle);

      hardware_interface::JointHandle pos_joint_handle(joint_state_handle, &joint_pos_cmd[i]);
      pos_joint_interface.registerHandle(pos_joint_handle);

      hardware_interface::JointHandle vel_joint_handle(joint_state_handle, &joint_vel_cmd[i]);
      vel_joint_interface.registerHandle(vel_joint_handle);
    }

    registerInterface(&joint_state_interface);
    registerInterface(&pos_joint_interface);
    registerInterface(&vel_joint_interface);

    ROS_INFO("Hardware Interface initialized correctly.");
    return true;
  }


  bool start()
  {
    if (!ec_master.start())
    {
      ROS_FATAL("EtherCAT master failed to enter in OPERATIONAL STATE!");
      return false;
    }

    ROS_INFO("EtherCAT master entered in OPERATIONAL STATE");
    return true;
  }


  void read()
  {
    joint_pos[0] = ec_master.tx_pdo.position_actual_value / POSITION_STEP_FACTOR;
    joint_vel[0] = ec_master.tx_pdo.position_actual_value / VELOCITY_STEP_FACTOR;
  }


  void write()
  {
    ec_master.rx_pdo.control_word = 0x000F;
    ec_master.rx_pdo.mode_of_operation = 9;
    ec_master.rx_pdo.target_velocity = joint_vel_cmd[0] * VELOCITY_STEP_FACTOR;
    ec_master.rx_pdo.touch_probe_function = 0;
    ec_master.rx_pdo.physical_outputs = 0x0000;

    ec_master.update();
  }


  void close()
  {
    ec_master.close();
  }

};

} }  // namespace
#endif
