#include "esa_servo/ewdl/hardware_interface/ewdl_hardware_interface.h"


bool esa::ewdl::ServoHW::start()
{
  if (!ec_master.start())
  {
    return false;
  }

  return true;
}


bool esa::ewdl::ServoHW::start(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
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


bool esa::ewdl::ServoHW::set_zero_position()
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


bool esa::ewdl::ServoHW::set_zero_position(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
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


bool esa::ewdl::ServoHW::start_homing()
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


bool esa::ewdl::ServoHW::start_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
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


bool esa::ewdl::ServoHW::stop_homing()
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


bool esa::ewdl::ServoHW::stop_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
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


bool esa::ewdl::ServoHW::start_motion()
{
  if (!ec_master.start_cyclic_syncronous_velocity())
  {
    ROS_ERROR("Failed to start motion.");
    return false;
  }

  reset_controllers = false;

  ROS_INFO("Motion started.");
  return true;
}


bool esa::ewdl::ServoHW::start_motion(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
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


bool esa::ewdl::ServoHW::halt()
{
  if (!ec_master.halt())
  {
    ROS_ERROR("Halt failed!");
    return false;
  }

  ROS_INFO("Halt!");
  return true;
}


bool esa::ewdl::ServoHW::halt(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (halt())
  {
    res.success = true;
    res.message = "Halt!";
  }
  else
  {
    res.success = false;
    res.message = "Halt failed!";
  }

  return true;
}
