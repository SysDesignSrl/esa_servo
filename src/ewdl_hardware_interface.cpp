// STL
#include <string>
#include <vector>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// controller_manager
#include <controller_manager/controller_manager.h>
// esa_servo
#include "esa_servo/ewdl/ethercat/master.h"
#include "esa_servo/ewdl/hardware_interface/ewdl_hardware_interface.h"


int main (int argc, char* argv[])
{
  ros::init(argc, argv, "ewdl_hardware_interface");

  ros::NodeHandle node("~");


  double loop_hz;
  if (!node.getParam("/EWDL/hardware_interface/loop_hz", loop_hz))
  {
    ROS_FATAL("Parameter 'loop_hz' not defined.");
    return -1;
  }


  ros::AsyncSpinner spinner(0);
  spinner.start();


  // Hardware Interface
  esa::ewdl::EWDL_HardwareInterface ewdl_hw(node);

  if (!ewdl_hw.init())
  {
    ewdl_hw.close();
    return -1;
  }

  // Controller Manager
  controller_manager::ControllerManager controller_manager(&ewdl_hw, node);


  if (!ewdl_hw.start())
  {
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
    //ROS_DEBUG_THROTTLE(0, "Period: %fs", period.toSec());

    ewdl_hw.read();
    controller_manager.update(time, period);
    ewdl_hw.write();

    prev_time = time;
  }

  ewdl_hw.close();
  return 0;
}
