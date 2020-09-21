#ifndef ESA_EWDL_HARDWARE_INTERFACE_H
#define ESA_EWDL_HARDWARE_INTERFACE_H
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

// STL
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// roscpp
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/steady_timer.h>
// std_srvs
#include <std_srvs/Trigger.h>
// controller_manager
#include <controller_manager/controller_manager.h>
// transmission_interface
#include <transmission_interface/transmission_interface.h>
// hardware_interface
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/robot_hw.h>
// xmlrpcpp
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

// esa_servo
#include "esa_servo/ewdl/ethercat/master.h"


namespace esa { namespace ewdl {

static const double POSITION_STEP_FACTOR = 10000.0;
static const double VELOCITY_STEP_FACTOR = 10000.0;


class ServoHW : public hardware_interface::RobotHW {
private:

  esa::ewdl::ethercat::Master ec_master;

  // std::ofstream logfile;

protected:

  ros::NodeHandle node;
  ros::SteadyTimer control_loop;

  // Controller Manager
  std::shared_ptr<controller_manager::ControllerManager> controller_manager;

  // Hardware Interface
  hardware_interface::ActuatorStateInterface act_state_interface;
  hardware_interface::PositionActuatorInterface pos_act_interface;
  hardware_interface::VelocityActuatorInterface vel_act_interface;

  std::vector<std::string> joints;
  std::vector<std::string> actuators;

  //
  std::vector<double> a_pos, a_pos_cmd;
  std::vector<double> a_vel, a_vel_cmd;
  std::vector<double> a_eff, a_eff_cmd;

  //
  std::vector<double> joint_lower_limits;
  std::vector<double> joint_upper_limits;


  void control_loop_cb(const ros::SteadyTimerEvent &ev)
  {
    pthread_t pthread = pthread_self();
    sched_param param;
    int policy;

    errno = pthread_getschedparam(pthread, &policy, &param);
    if (errno != 0)
    {
      perror("pthread_getschedparam");
      exit(EXIT_FAILURE);
    }

    if (policy != SCHED_FIFO || param.sched_priority < 90)
    {
      policy = SCHED_FIFO;
      param.sched_priority = 90;
      errno = pthread_setschedparam(pthread, policy, &param);
      if (errno != 0)
      {
        perror("pthread_setschedparam");
        exit(EXIT_FAILURE);
      }
    }

    const ros::Time now = ros::Time::now();
    const ros::Duration period((ev.current_real - ev.last_real).toSec());
    // ROS_DEBUG("period: %lu us", period.toNSec() / 1000U);

    read(now, period);
    act_to_jnt_state_interface->propagate();

    controller_manager->update(now, period, reset_controllers);
    reset_controllers = false;

    jnt_to_act_pos_interface->propagate();
    write(now, period);
  }

public:

  struct {
    bool ready_to_switch_on;
    bool switched_on;
    bool operation_enabled;
    bool fault;
    bool voltage_enabled;
    bool quick_stop;
    bool switch_on_disabled;
    bool warning;
    bool remote;
    bool target_reached;
    bool internal_limit_active;
    bool homing_attained;
    bool homing_error;
    bool following_error;
  } status;

  bool reset_controllers = true;

  // Transmission Interface
  transmission_interface::ActuatorToJointStateInterface* act_to_jnt_state_interface;
  transmission_interface::JointToActuatorPositionInterface* jnt_to_act_pos_interface;

  ServoHW(ros::NodeHandle &node) : node(node),
  controller_manager(new controller_manager::ControllerManager(this, node))
  {

  }


  bool init_ethercat(const std::string &ifname, const std::vector<std::string> &slaves)
  {
    ec_master = esa::ewdl::ethercat::Master(ifname, slaves);

    if (ec_master.init())
    {
      ROS_INFO("EtherCAT Master interface: %s", ifname.c_str());
    }
    else
    {
      ROS_FATAL("Failed to initialize EtherCAT master.");
      return false;
    }

    for (int i = 0; i < slaves.size(); i++)
    {
      ROS_INFO("EtherCAT Slave[%d]: %s", 1+i, slaves[i].c_str());
    }

    return true;
  }


  bool config_slaves(XmlRpc::XmlRpcValue &slaves_param)
  {
    for (int i = 0; i < slaves_param.size(); i++)
    {
      try
      {
        int homing_method = slaves_param[i]["homing_method"];
        int homing_speed_to_switch = slaves_param[i]["homing_speed"][0];
        int homing_speed_to_zero = slaves_param[i]["homing_speed"][1];
        int homing_acceleration = slaves_param[i]["homing_acceleration"];

        int home_offset = slaves_param[i]["home_offset"];
        int home_switch = slaves_param[i]["home_switch"];

        int following_error_window = slaves_param[i]["following_error_window"];

        int in_position_counts = slaves_param[i]["in_position_counts"];
        int in_position_error_range = slaves_param[i]["in_position_error_range"];
        int in_position_timing = slaves_param[i]["in_position_timing"];

        int quickstop_deceleration = slaves_param[i]["quickstop_deceleration"];

        const uint16 slave_idx = 1 + i;
        ROS_DEBUG("EtherCAT Slave[%d] Homing Method: %d", slave_idx, homing_method);
        ROS_DEBUG("EtherCAT Slave[%d] Homing Speed: %d %d", slave_idx, homing_speed_to_switch, homing_speed_to_zero);
        ROS_DEBUG("EtherCAT Slave[%d] Homing Acceleration: %d", slave_idx, homing_acceleration);
        ROS_DEBUG("EtherCAT Slave[%d] Home Offset: %d", slave_idx, home_offset);
        ROS_DEBUG("EtherCAT Slave[%d] Home Switch: 0x%.2x", slave_idx, home_switch);
        ROS_DEBUG("EtherCAT Slave[%d] Following Error Window: %u", slave_idx, following_error_window);
        ROS_DEBUG("EtherCAT Slave[%d] In Position Counts: %u", slave_idx, in_position_counts);
        ROS_DEBUG("EtherCAT Slave[%d] In Position Error Range: %u", slave_idx, in_position_error_range);
        ROS_DEBUG("EtherCAT Slave[%d] In Position Timing: %u", slave_idx, in_position_timing);
        ROS_DEBUG("EtherCAT Slave[%d] QuickStop Deceleration: %u", slave_idx, quickstop_deceleration);

        ec_master.config_homing(slave_idx, homing_method, homing_speed_to_switch, homing_speed_to_zero, homing_acceleration, home_offset, home_switch);
        ec_master.config_following_error_window(slave_idx, following_error_window);
        ec_master.config_in_position(slave_idx, in_position_counts, in_position_error_range, in_position_timing);
        ec_master.config_quickstop(slave_idx, quickstop_deceleration);
      }
      catch (const XmlRpc::XmlRpcException &ex)
      {
        auto code = ex.getCode();
        auto message = ex.getMessage();
        ROS_ERROR("Error Code: %d, %s", code, message.c_str());
        return false;
      }
    }

    return true;
  }


  bool init(double loop_hz, const std::vector<std::string> &joints, const std::vector<std::string> &actuators)
  {
    this->joints = joints;
    this->actuators = actuators;

    // EtherCAT
    std::string ifname;
    if (!node.getParam("ethercat/ifname", ifname))
    {
      std::string param_name = node.resolveName("ethercat/ifname");
      ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
      return false;
    }

    XmlRpc::XmlRpcValue slaves_param;
    if (!node.getParam("ethercat/slaves", slaves_param))
    {
      std::string param_name = node.resolveName("ethercat/slaves");
      ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
      return false;
    }

    std::vector<std::string> slaves;
    for (int i = 0; i < slaves_param.size(); i++)
    {
      try
      {
        std::string device_name = slaves_param[i]["device_name"];
        slaves.push_back(device_name);
      }
      catch (const XmlRpc::XmlRpcException &ex)
      {
        auto code = ex.getCode();
        auto message = ex.getMessage();
        ROS_ERROR("Error Code: %d, %s", code, message.c_str());
      }
    }

    if (!init_ethercat(ifname, slaves))
    {
      ROS_ERROR("Failed to initialize EtherCAT master");
      close();
      return false;
    }

    if (!config_slaves(slaves_param))
    {
      ROS_ERROR("Failed to configure EtherCAT slaves");
      close();
      return false;
    }

    //
    ros::WallDuration period(1.0/loop_hz);
    control_loop = node.createSteadyTimer(period, &esa::ewdl::ServoHW::control_loop_cb, this, false, false);

    // Hardware Interface
    const int n_actuators = actuators.size();

    a_pos.resize(n_actuators, 0.0); a_pos_cmd.resize(n_actuators, 0.0);
    a_vel.resize(n_actuators, 0.0); a_vel_cmd.resize(n_actuators, 0.0);
    a_eff.resize(n_actuators, 0.0); a_eff_cmd.resize(n_actuators, 0.0);

    for (int i = 0; i < n_actuators; i++)
    {
      hardware_interface::ActuatorStateHandle act_state_handle(actuators[i], &a_pos[i], &a_vel[i], &a_eff[i]);
      act_state_interface.registerHandle(act_state_handle);

      hardware_interface::ActuatorHandle pos_act_handle(act_state_handle, &a_pos_cmd[i]);
      pos_act_interface.registerHandle(pos_act_handle);

      hardware_interface::ActuatorHandle vel_act_handle(act_state_handle, &a_vel_cmd[i]);
      vel_act_interface.registerHandle(vel_act_handle);
    }

    registerInterface(&act_state_interface);
    registerInterface(&pos_act_interface);

    return true;
  }


  /* */
  bool start();
  bool start(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool fault_reset();
  bool fault_reset(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool ready_to_switch_on();
  bool ready_to_switch_on(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool switch_on();
  bool switch_on(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool switch_off();
  bool switch_off(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool start_homing();
  bool start_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /**/
  bool start_motion();
  bool start_motion(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool halt();
  bool halt(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool quick_stop();
  bool quick_stop(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool set_zero_position();
  bool set_zero_position(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);


  void read(const ros::Time &time, const ros::Duration &period)
  {
    const int n_actuators = actuators.size();

    for (int i = 0; i < n_actuators; i++)
    {
      const uint16 slave_idx = 1 + i;

      a_pos[i] = ec_master.tx_pdo[slave_idx].position_actual_value / POSITION_STEP_FACTOR;

      //ROS_DEBUG_THROTTLE(1.0, "Slave[%d], status_word: 0x%.4x", slave_idx, ec_master.tx_pdo[slave_idx].status_word);
      //ROS_DEBUG_THROTTLE(1.0, "Slave[%d], mode_of_operation display: %d", slave_idx, ec_master.tx_pdo[slave_idx].mode_of_operation_display);
      //ROS_DEBUG_THROTTLE(1.0, "Slave[%d], position_actual_value: %d", slave_idx, ec_master.tx_pdo[slave_idx].position_actual_value);
      //ROS_DEBUG_THROTTLE(1.0, "Slave[%d], digital_inputs: 0x%.8x", slave_idx, ec_master.tx_pdo[slave_idx].digital_inputs);
    }

    for (int i = 0; i < n_actuators; i++)
    {
      const uint16 slave_idx = 1 + i;

      uint16 status_word = ec_master.tx_pdo[slave_idx].status_word;
      int8 mode_of_operation_display = ec_master.tx_pdo[slave_idx].mode_of_operation_display;
      // ROS_DEBUG("Slave[%d], status_word: 0x%.4x", slave_idx, status_word);
      // ROS_DEBUG("Slave[%d], mode_of_operation display: %d", slave_idx, mode_of_operation_display);

      status.ready_to_switch_on = (status_word >> 0) & 0x01;
      status.switched_on = (status_word >> 1) & 0x01;
      status.operation_enabled = (status_word >> 2) & 0x01;
      status.fault = (status_word >> 3) & 0x01;
      status.voltage_enabled = (status_word >> 4) & 0x01;
      status.quick_stop = (status_word >> 5) & 0x01;
      status.switch_on_disabled = (status_word >> 6) & 0x01;
      status.warning = (status_word >> 7) & 0x01;

      status.remote = (status_word >> 9) & 0x01;
      status.target_reached = (status_word >> 10) & 0x01;
      status.internal_limit_active = (status_word >> 11) & 0x01;

      if (!status.operation_enabled)
      {
        reset_controllers = true;
      }

      switch (mode_of_operation_display)
      {
        case esa::ewdl::ethercat::mode_of_operation_t::HOMING:
          status.homing_attained = (status_word >> 12) & 0x01;
          status.homing_error = (status_word >> 13) & 0x01;
          break;
        case esa::ewdl::ethercat::mode_of_operation_t::CYCLIC_SYNCHRONOUS_POSITION:
          status.following_error = (status_word >> 13) & 0x01;
          break;
        case esa::ewdl::ethercat::mode_of_operation_t::CYCLIC_SYNCHRONOUS_VELOCITY:
          status.following_error = (status_word >> 13) & 0x01;
          break;
      }
    }
  }


  void write(const ros::Time &time, const ros::Duration &period)
  {
    const int n_actuators = actuators.size();

    for (int i = 0; i < n_actuators; i++)
    {
      const uint16 slave_idx = 1 + i;

      ec_master.rx_pdo[slave_idx].target_position = a_pos_cmd[i] * POSITION_STEP_FACTOR;
      ec_master.rx_pdo[slave_idx].touch_probe_function = 0;
      ec_master.rx_pdo[slave_idx].physical_outputs = 0x0000;

      // ROS_DEBUG_THROTTLE(1.0, "Slave[%d], control_word: 0x%.4x", slave_idx, ec_master.rx_pdo[slave_idx].control_word);
      // ROS_DEBUG("Slave[%d], control_word: 0x%.4x", slave_idx, ec_master.rx_pdo[slave_idx].control_word);
      // ROS_DEBUG("Slave[%d], mode_of_operation: %d", slave_idx, ec_master.rx_pdo[slave_idx].mode_of_operation);
      ROS_DEBUG("%d %d", ec_master.tx_pdo[slave_idx].position_actual_value, ec_master.rx_pdo[slave_idx].target_position);
    }

    ec_master.update();
  }


  void close()
  {
    control_loop.stop();

    ec_master.close();
    ROS_INFO("EtherCAT socket closed.");
  }

};

} }  // namespace
#endif
