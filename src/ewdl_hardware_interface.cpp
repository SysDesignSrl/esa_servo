#include "esa_servo/ewdl/hardware_interface/ewdl_hardware_interface.h"


bool esa::ewdl::ServoHW::start()
{
  if (ec_master.start())
  {
    reset_controllers = true;

    ROS_INFO("EtherCAT OPERATIONAL");
    return true;
  }
  else
  {
    ROS_ERROR("EtherCAT failed to enter state OPERATIONAL");
    return false;
  }
}


bool esa::ewdl::ServoHW::start(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (start())
  {
    res.success = true;
    res.message = "EtherCAT OPERATIONAL";
  }
  else
  {
    res.success = false;
    res.message = "EtherCAT failed to enter state OPERATIONAL";
  }

  return true;
}


bool esa::ewdl::ServoHW::fault_reset()
{
  if (ec_master.fault_reset())
  {
    reset_controllers = true;
    node.setParam("fault", false);

    ROS_INFO("Fault Reset");
    return true;
  }
  else
  {
    ROS_ERROR("Failed to Fault Reset");
    return false;
  }
}


bool esa::ewdl::ServoHW::fault_reset(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (fault_reset())
  {
    res.success = true;
    res.message = "Fault Reset";
  }
  else
  {
    res.success = false;
    res.message = "Failed to Fault Reset";
  }

  return true;
}


bool esa::ewdl::ServoHW::ready_to_switch_on()
{
  if (ec_master.ready_to_switch_on())
  {
    reset_controllers = true;

    ROS_INFO("Ready to Switch On");
    return true;
  }
  else
  {
    ROS_ERROR("Failed to enter state 'Ready to Switch On'");
    return false;
  }
}


bool esa::ewdl::ServoHW::ready_to_switch_on(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (ready_to_switch_on())
  {
    res.success = true;
    res.message = "Ready to Switch On";
  }
  else
  {
    res.success = false;
    res.message = "Failed to enter state 'Ready to Switch On'";
  }

  return true;
}


bool esa::ewdl::ServoHW::switch_on()
{
  if (ec_master.switch_on())
  {
    reset_controllers = true;

    ROS_INFO("Switch On");
    return true;
  }
  else
  {
    ROS_ERROR("Failed to Switch On");
    return false;
  }
}


bool esa::ewdl::ServoHW::switch_on(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (switch_on())
  {
    res.success = true;
    res.message = "Switch On";
  }
  else
  {
    res.success = false;
    res.message = "Failed to Switch On";
  }

  return true;
}


bool esa::ewdl::ServoHW::start_homing()
{
  if (ec_master.start_homing())
  {
    reset_controllers = true;

    ROS_INFO("Homing started...");
    return true;
  }
  else
  {
    ROS_ERROR("Failed to start Homing.");
    return false;
  }
}


bool esa::ewdl::ServoHW::start_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (start_homing())
  {
    res.success = true;
    res.message = "Homing started...";
  }
  else
  {
    res.success = false;
    res.message = "Failed to start Homing preocedure.";
  }

  return true;
}


bool esa::ewdl::ServoHW::start_motion()
{
  if (ec_master.start_cyclic_syncronous_position())
  {
    reset_controllers = false;
    node.setParam("motion_enabled", true);

    ROS_INFO("Motion started.");
    return true;
  }
  else
  {
    ROS_ERROR("Failed to start motion!");
    return false;
  }
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
    res.message = "Failed to start motion!";
  }

  return true;
}


bool esa::ewdl::ServoHW::halt()
{
  if (ec_master.halt())
  {
    ROS_INFO("Halt!");
    return true;
  }
  else
  {
    ROS_ERROR("Halt failed!");
    return false;
  }
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


bool esa::ewdl::ServoHW::quick_stop()
{
  if (ec_master.quick_stop())
  {
    ROS_WARN("Quick Stop!");
    return true;
  }
  else
  {
    ROS_FATAL("Quick Stop failed!!!");
    return false;
  }
}


bool esa::ewdl::ServoHW::quick_stop(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (quick_stop())
  {
    res.success = true;
    res.message = "Quick Stop!";
  }
  else
  {
    res.success = false;
    res.message = "Quick Stop failed!!!";
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
