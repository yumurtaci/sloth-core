#include <ros/ros.h>
#include <state_machine/state_machine.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_node");

  ros::NodeHandle nh;
  statemachine::StateMachine StateMachine(nh);
  ros::spin();

  return 0;
}