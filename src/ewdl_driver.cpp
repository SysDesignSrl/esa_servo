// STL
#include <string>
#include <vector>

// roscpp
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
// std_srvs
#include <std_srvs/Trigger.h>
// transmission_interface
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
// controller_manager
#include <controller_manager/controller_manager.h>
// xmlrpcpp
#include <XmlRpcValue.h>

// esa_servo
#include "esa_servo/ewdl/hardware_interface/ewdl_hardware_interface.h"


int main (int argc, char* argv[])
{
  ros::init(argc, argv, "ewdl_driver");

  // Node
  ros::NodeHandle node("~");

  ros::CallbackQueue callback_queue;
  node.setCallbackQueue(&callback_queue);


  // Parameters
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
  if (!servo_hw.init(loop_hz, joints, actuators))
  {
    ROS_FATAL("Failed to initialize Hardware Interface!");
    servo_hw.close();
    return 1;
  }


  // Transmission Interface
  transmission_interface::RobotTransmissions servo_tr;

  transmission_interface::TransmissionInterfaceLoader transmission_loader(&servo_hw, &servo_tr);
  if (!transmission_loader.load(urdf))
  {
    ROS_FATAL("Failed to load Transmission Interface from URDF!");
    return 1;
  }

  auto act_to_jnt_state_interface = servo_tr.get<transmission_interface::ActuatorToJointStateInterface>();
  // auto jnt_to_act_pos_interface = servo_tr.get<transmission_interface::JointToActuatorPositionInterface>();
  auto jnt_to_act_vel_interface = servo_tr.get<transmission_interface::JointToActuatorVelocityInterface>();


  // Advertised Services
  auto start_srv = node.advertiseService("start", &esa::ewdl::ServoHW::start, &servo_hw);
  auto stop_homing_srv = node.advertiseService("enable_motion", &esa::ewdl::ServoHW::enable_motion, &servo_hw);
  auto start_homing_srv = node.advertiseService("start_homing", &esa::ewdl::ServoHW::start_homing, &servo_hw);
  auto start_motion_srv = node.advertiseService("start_motion", &esa::ewdl::ServoHW::start_motion, &servo_hw);
  auto halt_srv = node.advertiseService("halt", &esa::ewdl::ServoHW::halt, &servo_hw);
  auto quick_stop_srv = node.advertiseService("quick_stop", &esa::ewdl::ServoHW::quick_stop, &servo_hw);
  auto set_zero_position_srv = node.advertiseService("set_zero_position", &esa::ewdl::ServoHW::set_zero_position, &servo_hw);

  // Controller Manager
  controller_manager::ControllerManager controller_manager(&servo_hw, node);
  if (!servo_hw.start())
  {
    ROS_FATAL("Failed to start Hardware Interface!");
    servo_hw.close();
    return 1;
  }

  if (!servo_hw.enable_motion())
  {
    ROS_FATAL("Failed to enable Motion!");
    servo_hw.close();
    return 1;
  }

  // Loop
  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(loop_hz);
  while (ros::ok())
  {
    rate.sleep();

    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    ROS_DEBUG_THROTTLE(1.0, "Period: %fs", period.toSec());

    servo_hw.read(time, period);
    act_to_jnt_state_interface->propagate();

    controller_manager.update(time, period, servo_hw.reset_controllers);

    jnt_to_act_vel_interface->propagate();
    servo_hw.write(time, period);

    prev_time = time;
  }


  servo_hw.close();
  return 0;
}
