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

class EWDL_HardwareInterface : public hardware_interface::RobotHW {
protected:
  ros::NodeHandle node;

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface pos_joint_interface;
  hardware_interface::VelocityJointInterface vel_joint_interface;

  std::vector<double> joint_pos, joint_pos_cmd;
  std::vector<double> joint_vel, joint_vel_cmd;
  std::vector<double> joint_eff, joint_eff_cmd;

  std::vector<double> joint_lower_limits;
  std::vector<double> joint_upper_limits;

public:
  double loop_hz;
  std::vector<std::string> joint_names;
  esa::ewdl::ethercat::Master *ec_master;


  EWDL_HardwareInterface(ros::NodeHandle &node) : node(node) {

    if (!node.getParam("/EWDL/hardware_interface/loop_hz", loop_hz))
      ROS_WARN("Hardware Interface: 'loop_hz' parameter not defined");

    if (!node.getParam("/EWDL/hardware_interface/joints", joint_names))
      ROS_WARN("Hardware Interface: 'joints' parameter not defined");

    int n_joints = joint_names.size();

    joint_pos.resize(n_joints, 0);
    joint_vel.resize(n_joints, 0);
    joint_eff.resize(n_joints, 0);

    joint_pos_cmd.resize(n_joints, 0);
    joint_vel_cmd.resize(n_joints, 0);
    joint_eff_cmd.resize(n_joints, 0);

    for (int i=0; i < n_joints; i++) {
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
  }


  void read(uint16 control_word) {

    ec_master->rx_pdo.control_word = control_word;
    ec_master->rx_pdo.mode_of_operation = 1;
    ec_master->rx_pdo.target_position = joint_pos_cmd[0] * 10000.0;
    ec_master->rx_pdo.touch_probe_function = 0;
    ec_master->rx_pdo.physical_outputs = 0x0000;

    ec_master->update();
    //ROS_DEBUG_THROTTLE(1, "TxPDO 0x%.4x", ec_master->tx_pdo.status_word);
    //printf("error_code: 0x%.4x\n", ec_master->tx_pdo.error_code);
    //printf("status_word: 0x%.4x\n", ec_master->tx_pdo.status_word);
    //printf("mode_of_operation_display: %d\n", ec_master->tx_pdo.mode_of_operation_display);

    joint_pos[0] = ec_master->tx_pdo.position_actual_value / 10000.0;
  }


  void close() {
    ec_master->close();
    ROS_INFO("EtherCAT socket %s closed.", "");
  }

};

} } // namespace
#endif
