// STL
#include <string>
#include <vector>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
// std_srvs
#include <std_srvs/Trigger.h>
// controller_manager
#include <controller_manager/controller_manager.h>
// esa_servo
#include "esa_servo/ewdl/hardware_interface/ewdl_hardware_interface.h"


int main (int argc, char* argv[])
{
  ros::init(argc, argv, "ewdl_driver");

  ros::NodeHandle node("~");

  ros::CallbackQueue callback_queue;
  node.setCallbackQueue(&callback_queue);


  double loop_hz;
  if (!node.getParam("/rail/hardware_interface/loop_hz", loop_hz))
  {
    ROS_FATAL("Parameter 'loop_hz' not defined.");
    return -1;
  }


  ros::AsyncSpinner spinner(0, &callback_queue);
  spinner.start();


  // Hardware Interface
  esa::ewdl::EWDL_HardwareInterface ewdl_hw(node);

  if (!ewdl_hw.init())
  {
    ROS_FATAL("Failed to initialize Hardware Interface!");
    ewdl_hw.close();
    return -1;
  }

  // Advertised Services
  auto start_srv = node.advertiseService("start", &esa::ewdl::EWDL_HardwareInterface::start, &ewdl_hw);

  auto set_zero_position_srv = node.advertiseService("set_zero_position", &esa::ewdl::EWDL_HardwareInterface::set_zero_position, &ewdl_hw);

  auto start_homing_srv = node.advertiseService("start_homing", &esa::ewdl::EWDL_HardwareInterface::start_homing, &ewdl_hw);
  auto stop_homing_srv = node.advertiseService("stop_homing", &esa::ewdl::EWDL_HardwareInterface::stop_homing, &ewdl_hw);

  auto start_motion_srv = node.advertiseService("start_motion", &esa::ewdl::EWDL_HardwareInterface::start_motion, &ewdl_hw);
  auto stop_motion_srv = node.advertiseService("stop_motion", &esa::ewdl::EWDL_HardwareInterface::stop_motion, &ewdl_hw);


  // Controller Manager
  controller_manager::ControllerManager controller_manager(&ewdl_hw, node);


  if (!ewdl_hw.start())
  {
    ROS_FATAL("Failed to start Hardware Interface!");
    ewdl_hw.close();
    return -1;
  }


  if (!ewdl_hw.start_motion())
  {
    ROS_FATAL("Failed to start Motion!");
    ewdl_hw.close();
    return -1;
  }


  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(loop_hz);
  while (ros::ok())
  {
    rate.sleep();
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    ROS_DEBUG_THROTTLE(1.0, "Period: %fs", period.toSec());

    ewdl_hw.read();
    controller_manager.update(time, period);
    ewdl_hw.write();

    prev_time = time;
  }

  ewdl_hw.close();
  return 0;
}
