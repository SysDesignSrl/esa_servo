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


int main (int argc, char* argv[]) {

  ros::init(argc, argv, "ewdl_hardware_interface");

  // Node
  ros::NodeHandle node("~");

  // Parameters
  std::string ifname;
  std::vector<std::string> slaves;

  if (!node.getParam("ethercat/ifname", ifname))
    ROS_ERROR("EtherCAT master interface not defined");

  if (!node.getParam("ethercat/slaves", slaves))
    ROS_ERROR("EtherCAT slaves not defined");


  // EtherCAT Master
  ROS_INFO("EtherCAT Master interface: %s", ifname.c_str());

  for (int i=0; i < slaves.size(); i++)
    ROS_INFO("EtherCAT Slave[%d]: %s", i+1, slaves[i].c_str());

  esa::ewdl::ethercat::Master ec_master(ifname);
  ec_master.slaves = slaves;

  if (!ec_master.init()) {
    ROS_FATAL("EtherCAT failed to initialize communication");
    return -1;
  }


  ros::AsyncSpinner spinner(4);
  spinner.start();


  // Hardware Interface
  esa::ewdl::EWDL_HardwareInterface hw_ewdl(node);
  hw_ewdl.ec_master = &ec_master;

  // Controller Manager
  controller_manager::ControllerManager controller_manager(&hw_ewdl);


  if (!ec_master.start()) {
    ROS_FATAL("EtherCAT failed to enter in OPERATIONAL STATE");
    hw_ewdl.close();
    return -1;
  }

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(hw_ewdl.loop_hz);
  // Loop
  while (ros::ok()) {
    rate.sleep();
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    //ROS_DEBUG_THROTTLE(0, "Period: %fs", period.toSec());

    hw_ewdl.read(0x000F);
    controller_manager.update(time, period);

    prev_time = time;
  }

  hw_ewdl.close();
  return 0;
}
