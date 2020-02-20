#include "esa_servo/ewdl/hardware_interface/ewdl_hardware_interface.h"


bool esa::ewdl::EWDL_HardwareInterface::start()
{
  if (!ec_master.start())
  {
    return false;
  }

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::start(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (start())
  {
    res.success = true;
    res.message = "Hardware Interface running.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to run Hardware Interface.";
  }

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::set_zero_position()
{
  if (ec_master.set_zero_position() > 0)
  {
    ROS_INFO("Setted Zero Position.");
    return true;
  }
  else
  {
    ROS_ERROR("Failed to set Zero Position.");
    return false;
  }
}


bool esa::ewdl::EWDL_HardwareInterface::set_zero_position(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (set_zero_position())
  {
    res.success = true;
    res.message = "Setted Zero Position.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to set Zero Position.";
  }

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::start_homing()
{
  if (!ec_master.start_homing())
  {
    ROS_ERROR("Failed to start Homing preocedure.");
    return false;
  }

  reset_controllers = true;

  ROS_INFO("Homing started.");
  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::start_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (start_homing())
  {
    res.success = true;
    res.message = "Homing started.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to start Homing preocedure.";
  }

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::stop_homing()
{
  if (!ec_master.stop_homing())
  {
    ROS_ERROR("Failed to stop Homing preocedure.");
    return false;
  }

  reset_controllers = false;

  ROS_INFO("Homing stopped.");
  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::stop_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (stop_homing())
  {
    res.success = true;
    res.message = "Homing stopped.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to stop Homing procedure.";
  }

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::start_motion()
{
  if (!ec_master.start_cyclic_syncronous_velocity())
  {
    ROS_ERROR("Failed to start motion.");
    return false;
  }

  ROS_INFO("Motion started.");
  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::start_motion(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (start_motion())
  {
    res.success = true;
    res.message = "Motion started.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to start motion.";
  }

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::stop_motion()
{
  if (!ec_master.stop_cyclic_syncronous_velocity())
  {
    ROS_ERROR("Failed to stop motion.");
    return false;
  }

  ROS_INFO("Motion stopped.");
  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::stop_motion(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (stop_motion())
  {
    res.success = true;
    res.message = "Halt Motion.";
  }
  else
  {
    res.success = false;
    res.message = "Halt motion failed.";
  }

  return true;
}
