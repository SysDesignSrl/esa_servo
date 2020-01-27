// STL
#include <cmath>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// std_msgs
#include <std_msgs/Float64.h>


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "step_test");

  // Node
  ros::NodeHandle node("~");

  // Parameters
  auto freq = node.param<double>("publish_frequency", 10);

  double min_value = node.param<double>("min_value", 0.0);
  double max_value = node.param<double>("max_value", 0.0);

  // Published Topics
  auto command_pub = node.advertise<std_msgs::Float64>("/rail/controller/position/rail_joint/command", 10);


  int count = 0;
  double value = 0.0;

  // Loop
  ros::Rate rate(freq);
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Time now = ros::Time::now();

    std_msgs::Float64 command_msg;
    command_msg.data = value;
    command_pub.publish(command_msg);

    value = (count < 50) ? min_value : max_value;
    count++;
    count %= 100;
    rate.sleep();
  }

  return 0;
}
