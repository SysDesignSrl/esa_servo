#include <string>
#include <vector>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
// std_mags
#include <std_msgs/Bool.h>
// std_srvs
#include <std_srvs/Trigger.h>
// industrial_msgs
#include <industrial_msgs/RobotMode.h>
#include <industrial_msgs/RobotStatus.h>
#include <industrial_msgs/TriState.h>
// transmission_interface
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
// xmlrpcpp
#include <XmlRpcValue.h>
// esa_servo
#include "esa_servo/ewdl/hardware_interface/ewdl_hardware_interface.h"


int main (int argc, char* argv[])
{
  ros::init(argc, argv, "ewdl_servo");

  // Node
  ros::NodeHandle node("~");

  ros::CallbackQueue callback_queue;
  node.setCallbackQueue(&callback_queue);


  // Parameters
  auto freq = node.param<double>("publish_frequency", 10);

  std::string urdf;
  if (!ros::param::get("robot_description", urdf))
  {
    ROS_FATAL("Failed to get parameter: 'robot_description'");
    return 1;
  }

  double loop_hz;
  if (!node.getParam("/rail/hardware_interface/loop_hz", loop_hz))
  {
    std::string param_name = node.resolveName("/rail/hardware_interface/loop_hz");
    ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
    return 1;
  }

  std::vector<std::string> joints;
  if (!node.getParam("/rail/hardware_interface/joints", joints))
  {
    std::string param_name = node.resolveName("/rail/hardware_interface/joints");
    ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
    return 1;
  }

  std::vector<std::string> actuators;
  if (!node.getParam("/rail/hardware_interface/actuators", actuators))
  {
    std::string param_name = node.resolveName("/rail/hardware_interface/actuators");
    ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
    return 1;
  }


  ros::AsyncSpinner spinner(2, &callback_queue);
  spinner.start();

  // Hardware Interface
  esa::ewdl::ServoHW servo_hw(node);
  if (servo_hw.init(loop_hz, joints, actuators))
  {
    ROS_INFO("Hardware Interface initialized correctly");
  }
  else
  {
    ROS_FATAL("Failed to initialize Hardware Interface");
    return 1;
  }

  // Transmission Interface
  transmission_interface::RobotTransmissions servo_tr;
  transmission_interface::TransmissionInterfaceLoader transmission_loader(&servo_hw, &servo_tr);
  if (transmission_loader.load(urdf))
  {
    servo_hw.act_to_jnt_state_interface = servo_tr.get<transmission_interface::ActuatorToJointStateInterface>();
    servo_hw.jnt_to_act_pos_interface = servo_tr.get<transmission_interface::JointToActuatorPositionInterface>();
    ROS_INFO("Transmission Interface loaded from URDF.");
  }
  else
  {
    ROS_FATAL("Failed to load Transmission Interface from URDF");
    return 1;
  }

  // Advertised Services
  auto fault_reset_srv = node.advertiseService("fault_reset", &esa::ewdl::ServoHW::fault_reset, &servo_hw);
  auto ready_to_switch_on_srv = node.advertiseService("ready_to_switch_on", &esa::ewdl::ServoHW::ready_to_switch_on, &servo_hw);
  auto switch_on_srv = node.advertiseService("switch_on", &esa::ewdl::ServoHW::switch_on, &servo_hw);
  auto switch_off_srv = node.advertiseService("switch_off", &esa::ewdl::ServoHW::switch_off, &servo_hw);
  auto enable_operation_srv = node.advertiseService("enable_operation", &esa::ewdl::ServoHW::enable_operation, &servo_hw);
  auto disable_operation_srv = node.advertiseService("disable_operation", &esa::ewdl::ServoHW::disable_operation, &servo_hw);
  auto start_homing_srv = node.advertiseService("start_homing", &esa::ewdl::ServoHW::start_homing, &servo_hw);
  auto start_motion_srv = node.advertiseService("start_motion", &esa::ewdl::ServoHW::start_motion, &servo_hw);
  auto halt_srv = node.advertiseService("halt", &esa::ewdl::ServoHW::halt, &servo_hw);
  auto quick_stop_srv = node.advertiseService("quick_stop", &esa::ewdl::ServoHW::quick_stop, &servo_hw);
  auto set_zero_position_srv = node.advertiseService("set_zero_position", &esa::ewdl::ServoHW::set_zero_position, &servo_hw);

  // Advertised Topics
  auto homing_attained_pub = node.advertise<std_msgs::Bool>("homing_attained", 10);
  auto homing_error_pub = node.advertise<std_msgs::Bool>("homing_error", 10);
  auto status_pub = node.advertise<industrial_msgs::RobotStatus>("status", 10);

  // Start
  auto cpu_affinity = node.param<int>("cpu_affinity", 0);
  if (servo_hw.start(cpu_affinity))
  {
    ROS_INFO("Hardware Interface started.");
  }
  else
  {
    ROS_FATAL("Failed to start Hardware Interface");
    return 1;
  }

  // Loop
  ros::Rate rate(freq);
  while (ros::ok())
  {
    rate.sleep();
    const ros::Time time = ros::Time::now();

    std_msgs::Bool msg;
    msg.data = servo_hw.status.homing_attained;
    homing_attained_pub.publish(msg);
    msg.data = servo_hw.status.homing_error;
    homing_error_pub.publish(msg);

    bool e_stopped = servo_hw.status.switch_on_disabled;
    bool drives_powered = servo_hw.status.switched_on;
    bool motion_possible = servo_hw.status.operation_enabled;
    bool in_error = servo_hw.status.fault;

    industrial_msgs::RobotStatus status_msg;
    status_msg.header.stamp = time;
    status_msg.mode.val = industrial_msgs::RobotMode::UNKNOWN;
    status_msg.e_stopped.val = (e_stopped) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
    status_msg.drives_powered.val = (drives_powered) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
    status_msg.motion_possible.val = (motion_possible) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
    status_msg.in_motion.val = industrial_msgs::TriState::UNKNOWN;
    status_msg.in_error.val = (in_error) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
    status_msg.error_code = 0;

    if (servo_hw.status.fault)
    {
      uint16 slave_idx = 1;

      uint16 error_code;
      servo_hw.get_error_code(slave_idx, error_code);
      status_msg.error_code = error_code;
      ROS_ERROR_THROTTLE(5.0, "ERROR CODE: 0x%.4X", error_code);

      uint32 alarm_code;
      servo_hw.get_alarm_code(slave_idx, alarm_code);
      ROS_ERROR_THROTTLE(5.0, "ALARM CODE: 0x%.8X", alarm_code);
    }

    if (servo_hw.status.warning)
    {
      uint16 slave_idx = 1;

      uint16 error_code;
      servo_hw.get_error_code(slave_idx, error_code);
      ROS_WARN_THROTTLE(5.0, "ERROR CODE: 0x%.4X", error_code);

      uint32 alarm_code;
      servo_hw.get_alarm_code(slave_idx, alarm_code);
      ROS_WARN_THROTTLE(5.0, "ALARM CODE: 0x%.8X", alarm_code);
    }

    status_pub.publish(status_msg);
  }


  servo_hw.close();
  return 0;
}
