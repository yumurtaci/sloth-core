/**
 * @file planner_node.cpp
 *
 * @date 03 June 2022
 * 
 * @brief Trajectory Planner using mav_trajectory_generation
 * 
 * @author Batuhan Yumurtaci <batuhan.yumurtaci@tum.de>
 */

#include <ros/ros.h>
#include <planner/planner.h>

#include <iostream>

int main(int argc, char** argv) 
{
    //ros::init(argc, argv, "simple_planner");
    ros::init(argc, argv, "trajectory_planner");

    ros::NodeHandle nh;	
    BasicPlanner planner(nh);
    ros::spin();
    
    return 0;
}
