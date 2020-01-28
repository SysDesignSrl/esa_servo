#ifndef ESA_EWDL_HARDWARE_INTERFACE_H
#define ESA_EWDL_HARDWARE_INTERFACE_H
// STL
#include <string>
#include <vector>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// std_srvs
#include <std_srvs/Trigger.h>
// controller_manager
#include <controller_manager/controller_manager.h>
// hardware_interface
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
// transmission interface
#include <transmission_interface/transmission_interface_loader.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission.h>
#include <transmission_interface/simple_transmission.h>
// esa_servo
#include "esa_servo/ewdl/ethercat/master.h"


namespace esa { namespace ewdl {

static const double POSITION_STEP_FACTOR = 10000.0;
static const double VELOCITY_STEP_FACTOR = 10000.0;


class EWDL_HardwareInterface : public hardware_interface::RobotHW {
private:
  esa::ewdl::ethercat::Master ec_master;

  uint16 control_word;

  enum mode_of_operation_t : int8
  {
    Q_PROGRAM = -1,                    // Q Program Mode (ESA specific)
    PROFILE_POSITION = 1,              // Profile Position Mode
    PROFILE_VELOCITY = 3,              // Profile Velocity Mode
    TORQUE_PROFILE = 4,                // Torque Profile Mode
    HOMING = 6,                        // Homing Mode
    CYCLIC_SYNCHRONOUS_POSITION = 8,   // Cyclic Synchronous Position Mode
    CYCLIC_SYNCHRONOUS_VELOCITY = 9,   // Cyclic Synchronous Velocity Mode
  } mode_of_operation;


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

protected:

  ros::NodeHandle node;

  double loop_hz;
  std::vector<std::string> joint_names;

  transmission_interface::SimpleTransmission rail_trans;


  transmission_interface::ActuatorToJointStateInterface act_to_jnt_state_interface;
  transmission_interface::JointToActuatorPositionInterface jnt_to_act_pos_interface;
  transmission_interface::JointToActuatorVelocityInterface jnt_to_act_vel_interface;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  //
  std::vector<double> a_pos, a_pos_cmd;
  std::vector<double> a_vel, a_vel_cmd;
  std::vector<double> a_eff, a_eff_cmd;

  std::vector<transmission_interface::ActuatorData> a_data, a_cmd_data;

  //
  std::vector<double> j_pos, j_pos_cmd;
  std::vector<double> j_vel, j_vel_cmd;
  std::vector<double> j_eff, j_eff_cmd;

  std::vector<transmission_interface::JointData> j_data, j_cmd_data;


  std::vector<double> joint_lower_limits;
  std::vector<double> joint_upper_limits;

public:

  EWDL_HardwareInterface(ros::NodeHandle &node) :
    node(node),
    mode_of_operation(mode_of_operation_t::CYCLIC_SYNCHRONOUS_VELOCITY),
    control_word(0x000F),
    rail_trans(3.0/0.200) { }


  bool init()
  {
    // initialize ethercat interface
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

    a_pos.resize(n_joints, 0.0); a_pos_cmd.resize(n_joints, 0.0);
    a_vel.resize(n_joints, 0.0); a_vel_cmd.resize(n_joints, 0.0);
    a_eff.resize(n_joints, 0.0); a_eff_cmd.resize(n_joints, 0.0);

    a_data.resize(n_joints);   a_cmd_data.resize(n_joints);

    j_pos.resize(n_joints, 0.0); j_pos_cmd.resize(n_joints, 0.0);
    j_vel.resize(n_joints, 0.0); j_vel_cmd.resize(n_joints, 0.0);
    j_eff.resize(n_joints, 0.0); j_eff_cmd.resize(n_joints, 0.0);

    j_data.resize(n_joints);   j_cmd_data.resize(n_joints);


    for (int i = 0; i < n_joints; i++)
    {
      a_data[i].position.push_back(&a_pos[i]);
      a_data[i].velocity.push_back(&a_vel[i]);
      a_data[i].effort.push_back(&a_eff[i]);

      a_cmd_data[i].position.push_back(&a_pos_cmd[i]);
      a_cmd_data[i].velocity.push_back(&a_vel_cmd[i]);
      a_cmd_data[i].effort.push_back(&a_eff_cmd[i]);

      j_data[i].position.push_back(&j_pos[i]);
      j_data[i].velocity.push_back(&j_vel[i]);
      j_data[i].effort.push_back(&j_eff[i]);

      j_cmd_data[i].position.push_back(&j_pos_cmd[i]);
      j_cmd_data[i].velocity.push_back(&j_vel_cmd[i]);
      j_cmd_data[i].effort.push_back(&j_eff_cmd[i]);

      // Transmission Interfaces
      transmission_interface::ActuatorToJointStateHandle act_to_jnt_state_handle("rail_trans", &rail_trans, a_data[i], j_data[i]);
      act_to_jnt_state_interface.registerHandle(act_to_jnt_state_handle);

      transmission_interface::JointToActuatorPositionHandle jnt_to_act_pos_handle("rail_trans", &rail_trans, a_cmd_data[i], j_cmd_data[i]);
      jnt_to_act_pos_interface.registerHandle(jnt_to_act_pos_handle);

      transmission_interface::JointToActuatorVelocityHandle jnt_to_act_vel_handle("rail_trans", &rail_trans, a_cmd_data[i], j_cmd_data[i]);
      jnt_to_act_vel_interface.registerHandle(jnt_to_act_vel_handle);

      // Hardware Interfaces
      hardware_interface::JointStateHandle jnt_state_handle(joint_names[i], &j_pos[i], &j_vel[i], &j_eff[i]);
      jnt_state_interface.registerHandle(jnt_state_handle);

      hardware_interface::JointHandle jnt_pos_handle(jnt_state_handle, &j_pos_cmd[i]);
      jnt_pos_interface.registerHandle(jnt_pos_handle);

      hardware_interface::JointHandle jnt_vel_handle(jnt_state_handle, &j_vel_cmd[i]);
      jnt_vel_interface.registerHandle(jnt_vel_handle);
    }

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
    registerInterface(&jnt_vel_interface);

    ROS_INFO("Hardware Interface initialized correctly.");
    return true;
  }


  bool run()
  {
    if (!ec_master.run())
    {
      ROS_FATAL("EtherCAT master failed to enter in OPERATIONAL STATE!");
      return false;
    }

    ROS_INFO("EtherCAT master entered in OPERATIONAL STATE");
    return true;
  }


  bool run(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    if (run())
    {
      res.success = true;
      res.message = "EtherCAT master entered in OPERATIONAL STATE";
    }
    else
    {
      res.success = false;
      res.message = "EtherCAT master failed to enter in OPERATIONAL STATE!";
    }

    return true;
  }


  bool set_zero_position()
  {
    if (ec_master.set_zero_position() == 0)
    {
      ROS_ERROR("Failed to set Zero Position.");
      return false;
    }

    ROS_INFO("Setted Zero Position.");
    return true;
  }


  bool set_zero_position(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    if (set_zero_position())
    {
      res.success = true;
      res.message = "Setted Zero Position.";
    }
    else
    {
      res.success = false;
      res.message = "Failed to set zero position.";
    }

    return true;
  }


  bool start_homing()
  {
    mode_of_operation = mode_of_operation_t::HOMING;
    control_word = 0x001F;

    return true;
  }


  bool start_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    if (start_homing())
    {
      res.success = true;
      res.message = "Homing preocedure started successfully.";
    }
    else
    {
      res.success = false;
      res.message = "Starting Homing preocedure failed.";
    }

    return true;
  }


  bool stop_homing()
  {
    mode_of_operation = mode_of_operation_t::HOMING;
    control_word = 0x000F;

    return true;
  }


  bool stop_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    if (stop_homing())
    {
      res.success = true;
      res.message = "Homing preocedure stopped successfully.";
    }
    else
    {
      res.success = false;
      res.message = "Sopping Homing preocedure failed.";
    }

    return true;
  }


  bool start_motion()
  {
    mode_of_operation = mode_of_operation_t::CYCLIC_SYNCHRONOUS_VELOCITY;
    control_word = 0x000F;

    return true;
  }


  bool start_motion(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    if (start_motion())
    {
      res.success = true;
      res.message = "Motion started successfully.";
    }
    else
    {
      res.success = false;
      res.message = "Starting motion mode failed.";
    }

    return true;
  }


  bool stop_motion()
  {
    mode_of_operation = mode_of_operation_t::CYCLIC_SYNCHRONOUS_VELOCITY;
    control_word = 0x010F;

    return true;
  }


  bool stop_motion(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    if (stop_motion())
    {
      res.success = true;
      res.message = "Motion stopped successfully.";
    }
    else
    {
      res.success = false;
      res.message = "Sopping motion failed.";
    }

    return true;
  }


  void read()
  {
    a_pos[0] = ec_master.tx_pdo.position_actual_value / POSITION_STEP_FACTOR;
    a_vel[0] = ec_master.tx_pdo.position_actual_value / VELOCITY_STEP_FACTOR;

    act_to_jnt_state_interface.propagate();
  }


  void write()
  {
    jnt_to_act_pos_interface.propagate();
    jnt_to_act_vel_interface.propagate();

    ec_master.rx_pdo.control_word = control_word;
    ec_master.rx_pdo.mode_of_operation = mode_of_operation;
    ec_master.rx_pdo.target_velocity = a_vel_cmd[0] * VELOCITY_STEP_FACTOR;
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
