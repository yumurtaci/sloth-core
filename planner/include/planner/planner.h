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

#include "planner/GetTrajectory.h"

class BasicPlanner {
public:
    BasicPlanner(ros::NodeHandle& nh);

    enum States
    {
        WAYPOINT = 0,       // From current pose directly to an end goal
        TRAJECTORY = 1,     // Multiple waypoints from YAML file
        PERCHING = 2,       // Parametric parabolic trajectory for perching
    };

    // Callback function to get the goal pose
    void goalposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    
    // Callback function to get the current pose
    void localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // Callback function to get the current linear and angular velocity
    void localvelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    // Trigger that generates and publishes trajectory
    //void plannerTriggerCallback(const std_msgs::Bool::ConstPtr& msg);

    // Get the value for activation flag
    bool getActiveFlag();

    // Set the maximum speed read from yaml file
    void setMaxSpeed(double max_v);
    
    // Function to set or removes constraints from middle waypoints
    bool add_vertex(mav_trajectory_generation::Vertex::Vector* p_vertices,
                    mav_trajectory_generation::Vertex* p_middle,
                    Eigen::Vector4d pos,
                    Eigen::Vector4d vel,
                    Eigen::Vector4d acc);

    // Plans a trajectory from the current position to a goal position and velocity
    bool planTrajectory(mav_trajectory_generation::Trajectory* trajectory);

    bool planTrajectory(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        const Eigen::VectorXd& start_pos,
                        const Eigen::VectorXd& start_vel,
                        double v_max, double a_max,
                        mav_trajectory_generation::Trajectory* trajectory);

    bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

    // Initialize publishers, subscribers and services
    void initializePublishers();
    void initializeSubscribers();
    void initializeServices();

    bool plannerTriggerServiceCallback(planner::GetTrajectory::Request &req,
                                       planner::GetTrajectory::Response &res);

    // Read the parameters from yaml file, max velocity and acceleration
    void readParameters();
        
    
private:
    
    ros::NodeHandle& nh_;

    ros::Publisher pub_markers_;                // RVIZ trajectory markers
    ros::Publisher pub_trajectory_;             // Polynomial trajectory
    
    //ros::Subscriber goal_pose_sub_;             // Position & Orientation of the goal
    //ros::Subscriber state_machine_sub_;         // State machine trigger
    ros::Subscriber local_pos_vel_sub_;         // Linear & Angular velocity from mavros
    ros::Subscriber local_pos_pose_sub_;        // Position & Orientation from mavros

    ros::ServiceServer trigger_service_;        // ####################

    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;

    Eigen::Affine3d goal_pose_affine_;
    Eigen::Vector4d goal_pose_;
    Eigen::Vector4d goal_velocity_;
    Eigen::Vector4d goal_acceleration_;
    
    bool planner_active_;                       // Trigger flag to generate a trajectory
    bool planner_done_;                         // Flag used to stop after publishing trajectory

    int state_;                                 // State to define the type of generated trajectory
    int dimension_;                             // 3D or 4D trajectories, 4th dim is heading 
    int derivative_to_optimize_;                // Optimize up to 4th order derivative (SNAP)

    double max_v_;                              // Maximum velocity constraint [m/s]
    double max_a_;                              // Maximum acceleration constraint [m/s^2]
    double max_ang_v_;
    double max_ang_a_;

    std::vector<int> valid_states_;          // Includes the neums of valid states



};

#endif // PLANNER_H

