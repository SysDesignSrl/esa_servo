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
  control_word = 0x001F;
  mode_of_operation = esa::ewdl::ethercat::mode_of_operation_t::HOMING;

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
  control_word = 0x000F;
  mode_of_operation = esa::ewdl::ethercat::mode_of_operation_t::HOMING;

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
  control_word = 0x000F;
  mode_of_operation = esa::ewdl::ethercat::mode_of_operation_t::CYCLIC_SYNCHRONOUS_VELOCITY;

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::start_motion(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (start_motion())
  {
    res.success = true;
    res.message = "Motion enabled.";
  }
  else
  {
    res.success = false;
    res.message = "Enablining Motion failed.";
  }

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::stop_motion()
{
  control_word = 0x010F;
  mode_of_operation = esa::ewdl::ethercat::mode_of_operation_t::CYCLIC_SYNCHRONOUS_VELOCITY;

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
