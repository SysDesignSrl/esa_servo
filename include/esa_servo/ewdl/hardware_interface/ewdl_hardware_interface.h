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
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/robot_hw.h>
// esa_servo
#include "esa_servo/ewdl/ethercat/master.h"


namespace esa { namespace ewdl {

static const double POSITION_STEP_FACTOR = 10000.0;
static const double VELOCITY_STEP_FACTOR = 10000.0;


class ServoHW : public hardware_interface::RobotHW {
private:

  esa::ewdl::ethercat::Master ec_master;


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

    ROS_INFO("EtherCAT Master network interface: %s", ifname.c_str());

    for (int i = 0; i < slaves.size(); i++)
    {
      ROS_INFO("EtherCAT Slave[%d]: %s", i+1, slaves[i].c_str());
    }

    return true;
  }

protected:

  ros::NodeHandle node;

  double loop_hz;
  std::vector<std::string> actuator_names;
  std::vector<std::string> joint_names;

  //
  hardware_interface::ActuatorStateInterface act_state_interface;
  hardware_interface::PositionActuatorInterface pos_act_interface;
  hardware_interface::VelocityActuatorInterface vel_act_interface;

  //
  std::vector<double> a_pos, a_pos_cmd;
  std::vector<double> a_vel, a_vel_cmd;
  std::vector<double> a_eff, a_eff_cmd;

  //
  std::vector<double> joint_lower_limits;
  std::vector<double> joint_upper_limits;

public:

  bool reset_controllers = false;


  ServoHW(ros::NodeHandle &node) : node(node)
  {

  }


  bool init()
  {
    // initialize ethercat interface
    if (!init_ethercat())
    {
      ROS_ERROR("Failed to initialize EtherCAT communication!");
      return false;
    }


    if (!node.getParam("/rail/hardware_interface/loop_hz", loop_hz))
    {
      ROS_ERROR("Parameter 'loop_hz' not defined!");
      return false;
    }

    if (!node.getParam("/rail/hardware_interface/actuators", actuator_names))
    {
      ROS_ERROR("Parameter 'actuators' not defined!");
      return false;
    }

    if (!node.getParam("/rail/hardware_interface/joints", joint_names))
    {
      ROS_ERROR("Parameter 'joints' not defined!");
      return false;
    }


    int n_actuators = actuator_names.size();

    a_pos.resize(n_actuators, 0.0); a_pos_cmd.resize(n_actuators, 0.0);
    a_vel.resize(n_actuators, 0.0); a_vel_cmd.resize(n_actuators, 0.0);
    a_eff.resize(n_actuators, 0.0); a_eff_cmd.resize(n_actuators, 0.0);

    for (int i = 0; i < n_actuators; i++)
    {
      hardware_interface::ActuatorStateHandle act_state_handle(actuator_names[i], &a_pos[i], &a_vel[i], &a_eff[i]);
      act_state_interface.registerHandle(act_state_handle);

      hardware_interface::ActuatorHandle pos_act_handle(act_state_handle, &a_pos_cmd[i]);
      pos_act_interface.registerHandle(pos_act_handle);

      hardware_interface::ActuatorHandle vel_act_handle(act_state_handle, &a_vel_cmd[i]);
      vel_act_interface.registerHandle(vel_act_handle);
    }

    registerInterface(&act_state_interface);
    // registerInterface(&pos_act_interface);
    registerInterface(&vel_act_interface);

    ROS_INFO("Hardware Interface initialized correctly.");
    return true;
  }


  /* */
  bool start();
  bool start(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  // bool stop();
  // bool stop(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool set_zero_position();
  bool set_zero_position(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool start_homing();
  bool start_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool stop_homing();
  bool stop_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /**/
  bool start_motion();
  bool start_motion(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool halt();
  bool halt(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool stop();
  bool stop(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);


  void read()
  {
    int n_actuators = actuator_names.size();

    //
    for (int i = 0; i < n_actuators; i++)
    {
      switch (ec_master.tx_pdo[1+i].mode_of_operation_display)
      {
        case 6:   // HOMING
          if ((ec_master.tx_pdo[1+i].status_word >> 10) & 0x01)   // Target Reached
          {
            ROS_INFO_THROTTLE(1.0, "Homing: Target Reached.");
          }
          if ((ec_master.tx_pdo[1+i].status_word >> 12) & 0x01)   // Homing Attained
          {
            ROS_INFO_THROTTLE(1.0, "Homing: Homing Attained.");
          }
          if ((ec_master.tx_pdo[1+i].status_word >> 13) & 0x01)   // Homing Error
          {
            ROS_WARN_THROTTLE(1.0, "Homing: Homing Error.");
          }
          break;

        case 8:   // CYCLIC SYNCHRONOUS POSITION
          if ((ec_master.tx_pdo[1+i].status_word >> 10) & 0x01)   // Target Reached
          {
            ROS_INFO_THROTTLE(1.0, "Cyclic Synchronous Position: Target Reached.");
          }
          if ((ec_master.tx_pdo[1+i].status_word >> 13) & 0x01)   // Following Error
          {
            ROS_WARN_THROTTLE(1.0, "Cyclic Synchronous Position: Following Error.");
          }
          break;

        case 9:   // CYCLIC SYNCHRONOUS VELOCITY
          if ((ec_master.tx_pdo[1+i].status_word >> 10) & 0x01)   // Target Reached
          {
            ROS_INFO_THROTTLE(1.0, "Cyclic Synchronous Velocity: Target Reached.");
          }
          if ((ec_master.tx_pdo[1+i].status_word >> 13) & 0x01)   // Following Error
          {
            ROS_WARN_THROTTLE(1.0, "Cyclic Synchronous Velocity: Following Error.");
          }
          break;

        default:

          break;
      }

      a_pos[i] = ec_master.tx_pdo[1+i].position_actual_value / POSITION_STEP_FACTOR;
      a_vel[i] = ec_master.tx_pdo[1+i].velocity_actual_value / VELOCITY_STEP_FACTOR;

      ROS_DEBUG_THROTTLE(1.0, "Slave[%d], status_word: 0x%.4x", 1+i, ec_master.tx_pdo[1+i].status_word);
      ROS_DEBUG_THROTTLE(1.0, "Slave[%d], mode_of_operation display: %d", 1+i, ec_master.tx_pdo[1+i].mode_of_operation_display);
      ROS_DEBUG_THROTTLE(1.0, "Slave[%d], position_actual_value: %d", 1+i, ec_master.tx_pdo[1+i].position_actual_value);
      ROS_DEBUG_THROTTLE(1.0, "Slave[%d], digital_inputs: 0x%.8x", 1+i, ec_master.tx_pdo[1+i].digital_inputs);
    }
  }


  void write()
  {
    int n_actuators = actuator_names.size();

    //
    for (int i = 0; i < n_actuators; i++)
    {
      if ((ec_master.tx_pdo[1+i].status_word >> 0) & 0x01)   // Ready to Switch On
      {
        ROS_INFO_ONCE("Slave[%d]: Ready to Switch On", i+1);
      }
      if ((ec_master.tx_pdo[1+i].status_word >> 1) & 0x01)   // Switched On
      {
        ROS_INFO_ONCE("Slave[%d]: Switched On", i+1);
      }
      if ((ec_master.tx_pdo[1+i].status_word >> 2) & 0x01)   // Operation Enabled
      {
        ROS_INFO_ONCE("Slave[%d]: Operation Enabled", i+1);
      }
      if ((ec_master.tx_pdo[1+i].status_word >> 3) & 0x01)   // Fault
      {
        ROS_ERROR_ONCE("Slave[%d]: Fault!!", i+1);
      }
      if ((ec_master.tx_pdo[1+i].status_word >> 4) & 0x01)   // Voltage Enabled
      {
        ROS_INFO_ONCE("Slave[%d]: Voltage Enabled", i+1);
      }
      if ((ec_master.tx_pdo[1+i].status_word >> 5) & 0x01)   // Quick Stop
      {
        ROS_INFO_ONCE("Slave[%d]: Quick Stop", i+1);
      }
      if ((ec_master.tx_pdo[1+i].status_word >> 6) & 0x01)   // Switch On Disabled
      {
        ROS_INFO_ONCE("Slave[%d]: Switch On Disabled", i+1);
      }
      if ((ec_master.tx_pdo[1+i].status_word >> 7) & 0x01)   // Warning
      {
        ROS_WARN_ONCE("Slave[%d]: Warning", i+1);
      }

      switch (ec_master.tx_pdo[1+i].mode_of_operation_display)
      {
        case 6:   // HOMING

          ec_master.rx_pdo[1+i].control_word = 0x001F;

          if ((ec_master.tx_pdo[1+i].status_word >> 10) & 0x01)   // Target Reached
          {

          }
          if ((ec_master.tx_pdo[1+i].status_word >> 11) & 0x01)   // Internal Limit Active
          {
            ROS_WARN_ONCE("Slave[%d]: Internal Limit Active", i+1);
          }
          if ((ec_master.tx_pdo[1+i].status_word >> 12) & 0x01)   // Homing Attained
          {

          }
          if ((ec_master.tx_pdo[1+i].status_word >> 13) & 0x01)   // Homing Error
          {

          }
          break;

        case 8:   // CYCLIC SYNCHRONOUS POSITION
          if ((ec_master.tx_pdo[1+i].status_word >> 10) & 0x01)   // Target Reached
          {

          }
          if ((ec_master.tx_pdo[1+i].status_word >> 11) & 0x01)   // Internal Limit Active
          {
            ROS_WARN_ONCE("Slave[%d]: Internal Limit Active", i+1);
          }
          if ((ec_master.tx_pdo[1+i].status_word >> 13) & 0x01)   // Following Error
          {

          }
          break;

        case 9:   // CYCLIC SYNCHRONOUS VELOCITY
          if ((ec_master.tx_pdo[1+i].status_word >> 10) & 0x01)   // Target Reached
          {

          }
          if ((ec_master.tx_pdo[1+i].status_word >> 11) & 0x01)   // Internal Limit Active
          {
            ROS_WARN_ONCE("Slave[%d]: Internal Limit Active", i+1);
          }
          if ((ec_master.tx_pdo[1+i].status_word >> 13) & 0x01)   // Following Error
          {

          }
          break;

        default:

          break;
      }

      ec_master.rx_pdo[1+i].target_velocity = a_vel_cmd[i] * VELOCITY_STEP_FACTOR;
      ec_master.rx_pdo[1+i].touch_probe_function = 0;
      ec_master.rx_pdo[1+i].physical_outputs = 0x0000;

      ROS_DEBUG_THROTTLE(1.0, "Slave[%d], control_word: 0x%.4x", 1+i, ec_master.rx_pdo[1+i].control_word);
    }

    ec_master.update();
  }


  void close()
  {
    ec_master.close();
    ROS_INFO("EtherCAT socket closed.");
  }

};

} }  // namespace
#endif
