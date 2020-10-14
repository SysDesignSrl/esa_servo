#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

// STL
#include <string>
#include <vector>
#include <chrono>
#include <thread>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
// std_mags
#include <std_msgs/Bool.h>
// std_srvs
#include <std_srvs/Trigger.h>
// industrial_msgs
#include <industrial_msgs/RobotStatus.h>
#include <industrial_msgs/RobotMode.h>
#include <industrial_msgs/TriState.h>
// transmission_interface
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
// xmlrpcpp
#include <XmlRpcValue.h>

// esa_servo
#include "esa_servo/ewdl/hardware_interface/ewdl_hardware_interface.h"


void* control_loop_routine(void* arg)
{
  esa::ewdl::ServoHW* servo_hw = (esa::ewdl::ServoHW*)arg;

  ros::WallRate rate(servo_hw->loop_hz);
  ros::SteadyTime prev_time = ros::SteadyTime::now();
  while (ros::ok())
  {
    rate.sleep();
    ros::SteadyTime curr_time = ros::SteadyTime::now();

    const ros::Time now = ros::Time::now();
    const ros::Duration period((curr_time - prev_time).toSec());
    // ROS_DEBUG("period: %lu us", period.toNSec() / 1000U);

    servo_hw->read(now, period);
    servo_hw->act_to_jnt_state_interface->propagate();

    servo_hw->controller_manager->update(now, period, servo_hw->reset_controllers);
    servo_hw->reset_controllers = false;

    servo_hw->jnt_to_act_pos_interface->propagate();
    servo_hw->write(now, period);

    prev_time = curr_time;
  }
}


int main (int argc, char* argv[])
{
  ros::init(argc, argv, "ewdl_driver");

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
  auto start_srv = node.advertiseService("start", &esa::ewdl::ServoHW::start, &servo_hw);
  auto fault_reset_srv = node.advertiseService("fault_reset", &esa::ewdl::ServoHW::fault_reset, &servo_hw);
  auto ready_to_switch_on_srv = node.advertiseService("ready_to_switch_on", &esa::ewdl::ServoHW::ready_to_switch_on, &servo_hw);
  auto switch_on_srv = node.advertiseService("switch_on", &esa::ewdl::ServoHW::switch_on, &servo_hw);
  auto switch_off_srv = node.advertiseService("switch_off", &esa::ewdl::ServoHW::switch_off, &servo_hw);
  auto start_homing_srv = node.advertiseService("start_homing", &esa::ewdl::ServoHW::start_homing, &servo_hw);
  auto start_motion_srv = node.advertiseService("start_motion", &esa::ewdl::ServoHW::start_motion, &servo_hw);
  auto halt_srv = node.advertiseService("halt", &esa::ewdl::ServoHW::halt, &servo_hw);
  auto quick_stop_srv = node.advertiseService("quick_stop", &esa::ewdl::ServoHW::quick_stop, &servo_hw);
  auto set_zero_position_srv = node.advertiseService("set_zero_position", &esa::ewdl::ServoHW::set_zero_position, &servo_hw);

  // Advertised Topics
  auto ready_to_switch_on_pub = node.advertise<std_msgs::Bool>("ready_to_switch_on", 1);
  auto switched_on_pub = node.advertise<std_msgs::Bool>("switched_on", 1);
  auto operation_enabled_pub = node.advertise<std_msgs::Bool>("operation_enabled", 1);
  auto fault_pub = node.advertise<std_msgs::Bool>("fault", 1);
  auto voltage_enabled_pub = node.advertise<std_msgs::Bool>("voltage_enabled", 1);
  auto quick_stop_pub = node.advertise<std_msgs::Bool>("quick_stop", 1);
  auto switch_on_disabled_pub = node.advertise<std_msgs::Bool>("switch_on_disabled", 1);
  auto warning_pub = node.advertise<std_msgs::Bool>("warning", 1);

  auto homing_attained_pub = node.advertise<std_msgs::Bool>("homing_attained", 1);
  auto homing_error_pub = node.advertise<std_msgs::Bool>("homing_error", 1);

  auto status_pub = node.advertise<industrial_msgs::RobotStatus>("status", 10);


  if (servo_hw.start())
  {
    ROS_INFO("Hardware Interface started.");
  }
  else
  {
    ROS_FATAL("Failed to start Hardware Interface");
    return 1;
  }


  pthread_t pthread;
  pthread_attr_t pthread_attr;

  errno = pthread_attr_init(&pthread_attr);
  if (errno != 0)
  {
    perror("pthread_attr_init");
    exit(EXIT_FAILURE);
  }

  errno = pthread_create(&pthread, &pthread_attr, &control_loop_routine, &servo_hw);
  if (errno != 0)
  {
    perror("pthread_create");
    exit(EXIT_FAILURE);
  }

  int policy = SCHED_FIFO;
  sched_param param { .sched_priority = 90 };
  errno = pthread_setschedparam(pthread, policy, &param);
  if (errno != 0)
  {
    perror("pthread_setschedparam");
    exit(EXIT_FAILURE);
  }

  errno = pthread_attr_destroy(&pthread_attr);
  if (errno != 0)
  {
    perror("pthread_attr_destroy");
    exit(EXIT_FAILURE);
  }


  // Loop
  ros::Rate rate(freq);
  while (ros::ok())
  {
    rate.sleep();

    const ros::Time time = ros::Time::now();


    std_msgs::Bool msg;

    msg.data = servo_hw.status.ready_to_switch_on;
    ready_to_switch_on_pub.publish(msg);

    msg.data = servo_hw.status.switched_on;
    switched_on_pub.publish(msg);

    msg.data = servo_hw.status.operation_enabled;
    operation_enabled_pub.publish(msg);

    msg.data = servo_hw.status.fault;
    fault_pub.publish(msg);

    msg.data = servo_hw.status.voltage_enabled;
    voltage_enabled_pub.publish(msg);

    msg.data = servo_hw.status.quick_stop;
    quick_stop_pub.publish(msg);

    msg.data = servo_hw.status.switch_on_disabled;
    switch_on_disabled_pub.publish(msg);

    msg.data = servo_hw.status.warning;
    warning_pub.publish(msg);


    msg.data = servo_hw.status.homing_attained;
    homing_attained_pub.publish(msg);

    msg.data = servo_hw.status.homing_error;
    homing_error_pub.publish(msg);


    industrial_msgs::RobotStatus status_msg;
    status_msg.header.stamp = time;
    status_msg.mode.val = industrial_msgs::RobotMode::UNKNOWN;
    status_msg.e_stopped.val = (!servo_hw.status.quick_stop) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
    status_msg.drives_powered.val = (servo_hw.status.switched_on) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
    status_msg.motion_possible.val = (servo_hw.status.operation_enabled) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
    status_msg.in_motion.val = industrial_msgs::TriState::UNKNOWN;
    status_msg.in_error.val = (servo_hw.status.fault) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
    status_msg.error_code = 0;

    // if (servo_hw.status.fault)
    // {
    //   uint16 error_code;
    //   servo_hw.get_error_code(1, error_code);
    //
    //   status_msg.error_code = error_code;
    // }
    //
    // if (servo_hw.status.warning)
    // {
    //   uint32 alarm_code;
    //   servo_hw.get_alarm_code(1, alarm_code);
    // }

    status_pub.publish(status_msg);
  }


  servo_hw.close();
  return 0;
}
