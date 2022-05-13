/****************************************************************************
 *
 *   Copyright (c) 2022-2025 Batuhan Yumurtaci. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file planner.h
 *
 * @date 03 June 2022
 * 
 * @brief Trajectory Planner using mav_trajectory_generation
 * 
 * @author Batuhan Yumurtaci <batuhan.yumurtaci@tum.de>
 */
#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ros/ros.h>

#include <Eigen/Dense>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

class BasicPlanner {
public:
    BasicPlanner(ros::NodeHandle& nh);

    // Callback function to get the current pose
    void localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // Callback function to get the current linear and angular velocity
    void localvelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    // Trigger that generates and publishes trajectory
    void plannerTriggerCallback(const std_msgs::Bool::ConstPtr& msg);

    // Get the value for activation flag
    bool getActiveFlag();

    // Set the maximum speed read from yaml file
    void setMaxSpeed(double max_v);

    // Plans a trajectory to take off from the current position and
    // fly to the given altitude (while maintaining x,y, and yaw).
    bool add_vertex(mav_trajectory_generation::Vertex::Vector* p_vertices,
                    mav_trajectory_generation::Vertex* p_middle,
                    Eigen::Vector4d pos,
                    Eigen::Vector4d vel,
                    Eigen::Vector4d acc);

    bool planTrajectory(mav_trajectory_generation::Trajectory* trajectory);

    bool planTrajectory(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        const Eigen::VectorXd& start_pos,
                        const Eigen::VectorXd& start_vel,
                        double v_max, double a_max,
                        mav_trajectory_generation::Trajectory* trajectory);

    bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

    // Initialize publishers, subscribers and servies
    void initializePublishers();
    void initializeSubscribers();

    // Read the parameters from yaml file
    void readParameters(); 
    
private:
    
    ros::Publisher pub_markers_;
    ros::Publisher pub_trajectory_;
    
    ros::Subscriber state_machine_sub_;         // Subscriber for state machine trigger
    ros::Subscriber local_pos_pose_sub_;        // Position & Orientation
    ros::Subscriber local_pos_vel_sub_;         // Linear & Angular velocity 

    ros::NodeHandle& nh_;
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;
    double max_v_;                              // Maximum velocity constraint [m/s]
    double max_a_;                              // Maximum acceleration constraint [m/s^2]
    double max_ang_v_;
    double max_ang_a_;

    bool planner_active_;
    bool planner_done_;

};

#endif // PLANNER_H

