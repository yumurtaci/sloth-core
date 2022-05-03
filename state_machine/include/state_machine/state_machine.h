#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <mutex>
#include <thread>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>

#include <geometry_msgs/PoseStamped.h>

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
        LANDING = 4
    };

    //Eigen::Vector3d quad_pos;
    typedef Eigen::Vector3d Vec3;
    typedef Eigen::Vector4d Vec4;
    typedef Eigen::Matrix<double, 6, 1> StateVectorEuler;
    typedef Eigen::Matrix<double, 7, 1> StateVectorQuaternion;
    
  private:
    // Node handles
    ros::NodeHandle nh_;
    // Publishers
    ros::Publisher local_pos_pub_;              // Desired Position & Oritentation
    // Subscribers
    ros::Subscriber state_sub_;                 // FCU state
    ros::Subscriber local_pos_pose_sub_;        // Position & Orientation 
    // Services
    ros::ServiceClient arming_client_;          // Arming ros service
    ros::ServiceClient set_mode_client_;        // Setting mode ros service
    ros::ServiceClient land_client_;            // Sanding ros service
    
    // Parameters
    int state_;                                 // State of the state machine
    const int rcl_except_ = 4;                  // Radio setting for offboard mode
    int input_;                                 // Keyboard input
    bool request_input_ = false;                // Keyboard input trigger
    std::mutex input_mutex_;                    // Non blocking state machine

    // Memory for sharing information between functions
    //Eigen::Vector3d quad_pos_;
    //Eigen::Vector3d goal_pos_;
    //Eigen::Vector3d quad_vel_;
    //Eigen::Matrix3d quad_rot_;
    //Eigen::Vector3d quad_rate_;

    mavros_msgs::State current_state_;          // FCU state
    mavros_msgs::SetMode offb_mode_cmd_;        // Offboard command for mode service
    mavros_msgs::CommandBool arm_cmd_;          // Arm command for arming service
    mavros_msgs::CommandBool disarm_cmd_;       // Disarm command for arming service
    mavros_msgs::CommandTOL land_cmd_;          // Land command for landing service

    geometry_msgs::PoseStamped des_pose_;       // Target pose
    geometry_msgs::PoseStamped takeoff_pose_;   // Take-off pose
    geometry_msgs::PoseStamped localtakeoff_pose_;   // Local Take-off pose
    geometry_msgs::PoseStamped current_pose_;   // Current pose 
    
    // Callback Functions
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    
    // Functions
    void UpdateState();                         // Update the states of the state machine
    void PublishCommand();                      // Commands in a particular state
    
    //bool WPConvergence(Vec3 quad_pos1, Vec3 quad_pos2); // Check if arrived at target waypoint
    bool WPConvergence(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2); // Check if arrived at target waypoint
    
    int getInput();
    void getKeyboardInput();
    void setInput(int input);
    void requestKeyboardInput();

  };
}

#endif