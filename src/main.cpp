#include <ros/ros.h>
#include <signal.h>
#include "../include/fusion_node.h"

bool keepRunning = true;

void sigintHandler(int sig) {
  keepRunning = false;
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "awv_fusion_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  signal(SIGINT, sigintHandler);

  AWV_Fusion::ESKF_Fusion fusion_node(nh);

  // Loop until Ctrl+C is pressed
  while (keepRunning && ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}