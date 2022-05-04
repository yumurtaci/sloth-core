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

#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>

#include <thread>
#include <mutex>

namespace statemachine
{
  
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
    };
    
    // Define vectors and matrices
    typedef Eigen::Vector3d Vec3;
    typedef Eigen::Vector4d Vec4;
    typedef Eigen::Matrix<double, 6, 1> StateVectorEuler;
    typedef Eigen::Matrix<double, 7, 1> StateVectorQuaternion;
    typedef Eigen::Matrix<double, 4, Eigen::Dynamic> WaypointMatrix;
    
  private:
    bool wp_completed_;                         // Flag used for waypoint mission
    bool request_input_ = false;                // Keyboard input trigger

    int n_wp_;                                  // Total number of waypoints
    int state_;                                 // State of the state machine
    int input_;                                 // Keyboard input
    int current_wp_;                            // Current waypoint number

    ros::Rate rate_ = 20;                       // Loop rate
    
    ros::NodeHandle nh_;                        // Node handles
    
    ros::Publisher local_pos_pub_;              // Desired Position & Orientation publisher
    
    ros::Subscriber state_sub_;                 // FCU state subscriber
    ros::Subscriber local_pos_pose_sub_;        // Position & Orientation 

    ros::ServiceClient arming_client_;          // Arming ros service
    ros::ServiceClient set_mode_client_;        // Setting mode ros service
    ros::ServiceClient land_client_;            // Landing ros service

    mavros_msgs::State current_state_;          // FCU state
    mavros_msgs::SetMode offb_mode_cmd_;        // Offboard command for mode service
    mavros_msgs::CommandBool arm_cmd_;          // Arm command for arming service
    mavros_msgs::CommandTOL land_cmd_;          // Land command for landing service

    geometry_msgs::PoseStamped des_pose_;       // Target pose
    geometry_msgs::PoseStamped takeoff_pose_;   // Take-off pose
    geometry_msgs::PoseStamped localtakeoff_pose_;   // Local Take-off pose
    geometry_msgs::PoseStamped current_pose_;   // Current pose 

    WaypointMatrix wp_matrix_;                  // Matrix to store Waypoints

    std::mutex input_mutex_;                    // Non blocking state machine
    
    // Callback function to get the current fcu status
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    
    // Callback function to get the current pose
    void localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    
    // Transition conditions of the state machine
    void UpdateState();     
    
    // Commands in a particular state
    void PublishCommand();
    
    // Read the waypoints from YAML file
    void ReadWaypoints();
    
    // Initialize the FCU connection for Mavros and PX4
    void InitializeFCU();
    
    // Initialize publishers, subscribers and servies
    void InitializePublishers();
    void InitializeSubscribers();
    void InitializeServices();  
    
    // Check if arrived at target waypoint
    bool WPConvergence(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2); 
    
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