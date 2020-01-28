#include "esa_servo/ewdl/hardware_interface/ewdl_hardware_interface.h"


bool esa::ewdl::EWDL_HardwareInterface::run()
{
  if (!ec_master.run())
  {
    return false;
  }

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::run(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (run())
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
  mode_of_operation = esa::ewdl::ethercat::mode_of_operation_t::HOMING;
  control_word = 0x001F;

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::start_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (start_homing())
  {
    res.success = true;
    res.message = "Homing preocedure started successfully.";
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
  mode_of_operation = esa::ewdl::ethercat::mode_of_operation_t::HOMING;
  control_word = 0x000F;

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::stop_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (stop_homing())
  {
    res.success = true;
    res.message = "Homing preocedure stopped successfully.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to stop Homing preocedure.";
  }

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::start_motion()
{
  mode_of_operation = esa::ewdl::ethercat::mode_of_operation_t::CYCLIC_SYNCHRONOUS_VELOCITY;
  control_word = 0x000F;

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::start_motion(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (start_motion())
  {
    res.success = true;
    res.message = "Motion Mode started successfully.";
  }
  else
  {
    res.success = false;
    res.message = "Starting Motion Mode failed.";
  }

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::stop_motion()
{
  mode_of_operation = esa::ewdl::ethercat::mode_of_operation_t::CYCLIC_SYNCHRONOUS_VELOCITY;
  control_word = 0x010F;

  return true;
}


bool esa::ewdl::EWDL_HardwareInterface::stop_motion(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (stop_motion())
  {
    res.success = true;
    res.message = "Motion stopped successfully.";
  }
  else
  {
    res.success = false;
    res.message = "Sopping motion failed.";
  }

  return true;
}
