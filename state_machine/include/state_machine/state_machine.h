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
 * @file state_machine.h
 *
 * @date 03 June 2022
 * 
 * @brief State Machine Mavros
 * 
 * @author Batuhan Yumurtaci <batuhan.yumurtaci@tum.de>
 */

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <ros/ros.h>
#include <stdlib.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>

#include "planner/GetTrajectory.h"
#include "controller_msgs/FlatTarget.h"

#include <thread>
#include <mutex>

namespace statemachine
{  
  // Timer class to perform time based actions
  class Timer
  {
  public:
    Timer(){};
    ros::Time start_time_;

    void startTimer()
    {start_time_ = ros::Time::now();}

    double getTime()
    {return ros::Duration(ros::Time::now() - start_time_).toSec();}
  };
  
  // State machine class to manage the mission
  class StateMachine
  {

  public:
    StateMachine(ros::NodeHandle nh);

    enum States
    {
      ARMED = 0,
      TAKEOFF = 1,
      WAYPOINT = 2,
      TRAJECTORY = 3,
      LANDING = 4,
      PERCHING = 5,
    };
    
    // Define vectors and matrices
    typedef Eigen::Vector3d Vec3;
    typedef Eigen::Matrix<double, 4, Eigen::Dynamic> WaypointMatrix;
    
  private:
    bool wp_completed_;                         // Flag used for waypoint mission
    bool request_input_ = false;                // Keyboard input trigger

    int n_wp_;                                  // Total number of waypoints
    int state_;                                 // State of the state machine
    int input_;                                 // Keyboard input
    int current_wp_;                            // Current waypoint index
    int perching_type_;                         // Vertical or inclined perching

    double ttc_threshold_;                  // Time to contact threshold
    double ttc_threshold_ver_;                  // Time to contact threshold for vertical perching
    double ttc_threshold_inc_;                  // Time to contact threshold for inclined perching
    double wp_converged_threshold_;             // Waypoint threshold for absolute distance in 3D space
    double wp_stopped_threshold_;               // Waypoint threshold for absolute velocity in 3D space 

    ros::Rate rate_ = 20;                       // Loop rate
    
    ros::NodeHandle nh_;                        // Node handles
    
    ros::Publisher rpy_pub_;                    // Roll, pitch and yaw publisher
    ros::Publisher local_pos_pub_;              // Desired Position & Orientation
    ros::Publisher timeToContact_pub_;          // Time to contact publisher
    ros::Publisher actuator_control_pub_;       // Actuator control for servos 
    ros::Publisher controller_trigger_pub_;     // Trigger for geometric controller
    
    
    ros::Subscriber state_sub_;                 // FCU state subscriber
    ros::Subscriber local_pos_pose_sub_;        // Position & Orientation
    ros::Subscriber local_pos_vel_sub_;         // Linear & Angular velocity 

    ros::ServiceClient land_client_;            // Landing ros service
    ros::ServiceClient arming_client_;          // Arming ros service
    ros::ServiceClient planner_client_;         // Getting trajectory ros service
    ros::ServiceClient set_mode_client_;        // Setting mode ros service
    ros::ServiceClient force_disarm_client_;    // Setting mode ros service
    
    std_msgs::Float64 timeToContact_msg_;       // Time to contact message
    std_msgs::Bool controller_activation_;      // Activation flag for controller
    std_msgs::Int32 actuator_control_msg_;      // Control arm actuator servos 
    // IDLE = 0, FLIGHT = 1, PREPARE = 2, CLOSE = 3, RELEASE = 4,
    
    mavros_msgs::State current_state_;          // FCU state
    mavros_msgs::CommandTOL land_cmd_;          // Land command for landing service
    mavros_msgs::CommandBool arm_cmd_;          // Arm command for arming service
    mavros_msgs::SetMode offb_mode_cmd_;        // Offboard command for mode service
    mavros_msgs::CommandLong force_disarm_cmd_; // Force disarm command for perching

    geometry_msgs::PoseStamped des_pose_;       // Target pose
    geometry_msgs::PoseStamped zero_pose_;      // Target pose for initial states
    geometry_msgs::PoseStamped takeoff_pose_;   // Take-off pose
    geometry_msgs::PoseStamped current_pose_;   // Current pose
    geometry_msgs::PoseStamped ver_perch_pose_; // Target pose for vertical perching
    geometry_msgs::PoseStamped inc_perch_pose_; // Target pose for inclined perching
    geometry_msgs::TwistStamped current_vel_;   // Current velocity
    geometry_msgs::Vector3 rpy_;                // Current roll, pitch and yaw

    planner::GetTrajectory waypoint_cmd_;       // Trigger waypoint w. planner ros service
    planner::GetTrajectory trajectory_cmd_;     // Trigger minimum snap polynomial w. planner ros service
    planner::GetTrajectory perching_trj_cmd_;   // Trigger minimum snap polynomial f. perching w. planner ros service
    planner::GetTrajectory localtakeoff_cmd_;   // Trigger local takeoff w. planner ros service
    planner::GetTrajectory globaltakeoff_cmd_;  // Trigger global takeoff w. planner ros service
    
    Vec3 current_RPY_;                          // Current Roll, Pitch and Yaw [rad]
    
    WaypointMatrix wp_matrix_;                  // Matrix to store Waypoints

    Timer traj_timer_;                          // Timer object for trajectory
    Timer perching_timer_;                      // Timer object for perching to release actuators
    
    std::mutex input_mutex_;                    // Non blocking state machine

    
    // Callback function to get the current fcu status
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    
    // Callback function to get the current pose
    void localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // Callback function to get the current linear and angular velocity
    void localvelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    
    // Transition conditions of the state machine
    void updateState();     
    
    // Commands in a particular state
    void publishCommand();

    // Publish target body rates and thrust for geometric controller
    void pubRateCommands();

    // Set and send the next waypoint for trajectory generation
    void sendNextWaypoint(int index);
    
    // Read the waypoints from YAML file
    void readWaypoints();
    
    // Check if arrived at target waypoint - position
    bool wpConvergence(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);
    
    // Check if arrived at target waypoint - current velocity
    bool wpStopped(); 

    // Time to target position based on current position and velocity
    double timeToContact(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);
    
    // Initialize the FCU connection for Mavros and PX4
    void initializeFCU();
    
    // Initialize publishers, subscribers and servies
    void initializePublishers();
    void initializeSubscribers();
    void initializeServices();
    void initializeParameters();  
    
    // Decouple threads of UpdateState and PublishCommand as waiting for user input
    // Required for uniterrupted advertisement of the desired pose 
    // Otherwise the offboard mode is deactivated
    int getInput();
    void getKeyboardInput();
    void setInput(int input);
    void requestKeyboardInput();

  };
}

#endif