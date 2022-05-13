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
 * @file planner.cpp
 *
 * @date 03 June 2022
 * 
 * @brief Trajectory Planner using mav_trajectory_generation
 * 
 * @author Batuhan Yumurtaci <batuhan.yumurtaci@tum.de>
 */

#include <planner/planner.h>

BasicPlanner::BasicPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        max_v_(0.2),
        max_a_(0.2),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity()) 
{
    // Initialize parameters
    planner_done_ = false;
    
    readParameters();
    initializePublishers();
    initializeSubscribers();
    
}

void BasicPlanner::localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    tf::poseMsgToEigen(msg->pose, current_pose_);
}

void BasicPlanner::localvelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    tf::vectorMsgToEigen(msg->twist.linear, current_velocity_);
}

void BasicPlanner::plannerTriggerCallback(const std_msgs::Bool::ConstPtr& msg){
    planner_active_ = msg->data;
    if (planner_active_ && !planner_done_){

        mav_trajectory_generation::Trajectory trajectory;
    
        planTrajectory(&trajectory);
        publishTrajectory(trajectory);
    
        ROS_WARN_STREAM("[PLANNER] TRAJECTORY PUBLISHED");
        planner_done_ = true;
    } else if (!planner_active_ && planner_done_)
    {
        planner_done_ = false;
    }
    else
    {
        // Waiting for the trigger IDLE for planner
    }
    
}

bool BasicPlanner::getActiveFlag(){
    return planner_active_;
}

// Method to set maximum speed.
void BasicPlanner::setMaxSpeed(const double max_v) {
    max_v_ = max_v;
}

bool BasicPlanner::add_vertex(mav_trajectory_generation::Vertex::Vector *p_vertices, // pointer to vertices
                                mav_trajectory_generation::Vertex *p_middle,  // pointer to middle
                                Eigen::Vector4d pos = Eigen::Vector4d(-1, -1, -1, -1),  // some defaults
                                Eigen::Vector4d vel = Eigen::Vector4d(-1, -1, -1, -1),
                                Eigen::Vector4d acc = Eigen::Vector4d(-1, -1, -1, -1)) {

    /* CK on December 18, 2020
     * function sets or removes constraints from middle waypoints
     * why is this helpful? Per assignment, we are only allowed to use the "middle" mav_trajectory_generation::Vertex.
     * if we don't do it like this it is a lot of copy/pasting in the code to remove the vel=0, acc=0 constraints again
     * after the first round (compare earlier minimal working commit).
    */

    /* how to set and remove constraints:
     middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos);
     middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, vel);
     middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, acc);

     middle.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
     middle.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
     middle.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);

     vertices.push_back(middle);
    */

    if (pos[1] != -1){  // not default, add constraint
        p_middle -> addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos);
    }
    else{
        p_middle -> removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
    }

    if (pos[1] != -1){  // not default, add constraint
        p_middle -> addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION, pos);
    }
    else{
        p_middle -> removeConstraint(mav_trajectory_generation::derivative_order::ORIENTATION);
    }


    if (vel[1] != -1){  // not default, add constraint
        p_middle -> addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, vel);
    }
    else{
        p_middle -> removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
    }

    if (acc[1] != -1){  // not default, add constraint
        p_middle -> addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, acc);
    }
    else{
        p_middle -> removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);
    }

    p_vertices->push_back(*p_middle);
    return true;
}

// Plans a trajectory from the current position to a goal position and velocity
bool BasicPlanner::planTrajectory(mav_trajectory_generation::Trajectory* trajectory) {
	assert(trajectory);
	trajectory->clear();
  
    // 3 Dimensional trajectory => through cartesian space, no orientation; 4D: cartesian space incl. orientation
    const int dimension = 4;

    // Array for all waypoints and their constraints
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimize up to 4th order derivative (SNAP)
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

    // we have 2 vertices:
    // Start = current position
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

    /******* Configure start point *******/
    // set start point constraints to current position and set all derivatives to zero
	Eigen::Vector4d current_p, current_v;
    current_p << current_pose_.translation(),mav_msgs::yawFromQuaternion((Eigen::Quaterniond)current_pose_.rotation());
	current_v << current_velocity_, 0.0;
	
	std::cout<<"current position: x: "<<current_p[0]<<", y: "<<current_p[1]<<", z: "<<current_p[2]<<", h: "<<current_p[3]<<std::endl;
    start.makeStartOrEnd(current_p, derivative_to_optimize);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_v);
    vertices.push_back(start);    // add waypoint to list

    /******* Configure trajectory *******/
	Eigen::Vector4d pos, vel, acc;
    bool no_fail = true;
    int i = 0;

    while (no_fail){
        std::cout<<"wp "<<i<<": preparing for import"<<std::endl;
        std::string idx = "/"+ std::to_string(i) + "/";
        std::string wp_str = ros::this_node::getName() + "/waypoints" + idx;

        // positions
        float pos_x, pos_y, pos_z, pos_h;
        no_fail = nh_.getParam(wp_str + "pos/x", pos_x);
        no_fail = nh_.getParam(wp_str + "pos/y", pos_y);
        no_fail = nh_.getParam(wp_str + "pos/z", pos_z);
        no_fail = nh_.getParam(wp_str + "pos/h", pos_h);
        if (!no_fail){
            std::cout<<"      failed to add waypoint"<<std::endl;
            std::cout<<"   -> ending import loop"<<std::endl;
            std::cout<<"      now starting to compute trajectory"<<std::endl;
            break;
        }
        pos << pos_x, pos_y, pos_z, pos_h;  // write position constraints to vector
        std::cout<<"      position constraints received"<<std::endl;
        std::cout<<"      wp pos: "<<pos_x<<" "<<pos_y<<" "<<pos_z<<" "<<pos_h<<std::endl;

        // velocities
        float vel_x, vel_y, vel_z, vel_h;
        no_fail = nh_.getParam(wp_str + "vel/x", vel_x);
        no_fail = nh_.getParam(wp_str + "vel/y", vel_y);
        no_fail = nh_.getParam(wp_str + "vel/z", vel_z);
        no_fail = nh_.getParam(wp_str + "vel/h", vel_h);
        if (!no_fail){
            std::cout<<"      failed to add velocity constraint"<<std::endl;
            std::cout<<"      wp "<<i<<" added with position constraint only"<<std::endl;
            std::cout<<std::endl;
            BasicPlanner::add_vertex(&vertices, &middle, pos);
            no_fail = true;  // reset failure indicator for next while loop
            i++;  // increase counter since we added the waypoint with position constraints only
            continue;
        }
        vel << vel_x, vel_y, vel_z, vel_h;  // write velocity constraints to vector; we got the position at least!
        std::cout<<"      velocity constraints received"<<std::endl;
        std::cout<<"      wp vel: "<<vel_x<<" "<<vel_y<<" "<<vel_z<<" "<<vel_h<<std::endl;

        // accelerations
        float acc_x, acc_y, acc_z, acc_h;
        no_fail = nh_.getParam(wp_str + "acc/x", acc_x);
        no_fail = nh_.getParam(wp_str + "acc/y", acc_y);
        no_fail = nh_.getParam(wp_str + "acc/z", acc_z);
        no_fail = nh_.getParam(wp_str + "acc/h", acc_h);
        if (!no_fail){
            std::cout<<"      failed to add acceleration constraint"<<std::endl;
            std::cout<<"      wp "<<i<<" added with position and velocity constraints only"<<std::endl;
            std::cout<<std::endl;
            BasicPlanner::add_vertex(&vertices, &middle, pos, vel);
            no_fail = true;  // reset failure indicator for next while loop; we got the position at least!
            i++;  // increase counter since we added the waypoint with position and velocity constraints only
            continue;
        }
        acc << acc_x, acc_y, acc_z, acc_h;  // write acceleration constraints to vector
        std::cout<<"      acceleration constraints received"<<std::endl;
        std::cout<<"      wp acc: "<<acc_x<<" "<<acc_y<<" "<<acc_z<<" "<<acc_h<<std::endl;
        std::cout<<"      wp "<<i<<" added with position, velocity and acceleration constraints"<<std::endl;
        std::cout<<std::endl;

        BasicPlanner::add_vertex(&vertices, &middle, pos, vel, acc);
        i++;  // increase counter
    }

    /******* Estimate initial segment times *******/
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    /******* Set up polynomial solver and compute trajectory *******/
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
	trajectory->scaleSegmentTimesToMeetConstraints(max_v_, max_a_);
	
    return true;
}

bool BasicPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance = 0.2; // Distance by which to separate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";

    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    pub_markers_.publish(markers);

    // send trajectory to be executed on UAV
    mav_planning_msgs::PolynomialTrajectory4D msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
    msg.header.frame_id = "world";
    pub_trajectory_.publish(msg);

    return true;
}

void BasicPlanner::initializePublishers(){
    // Create publisher for RVIZ markers
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
    pub_trajectory_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);
}

void BasicPlanner::initializeSubscribers(){
    //sub_odom_ = nh_.subscribe("odom", 1, &BasicPlanner::uavOdomCallback, this);
    
    state_machine_sub_ = nh_.subscribe("planner_activation", 1, &BasicPlanner::plannerTriggerCallback, this);

    local_pos_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &BasicPlanner::localposeCallback, this);

    local_pos_vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity", 10, &BasicPlanner::localvelCallback, this);
}

void BasicPlanner::readParameters(){
    nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_v", max_v_);
    ROS_INFO("[PLANNER] Max Vel: %.1f m/s", max_v_);

    nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_a", max_a_);
    ROS_INFO("[PLANNER] Max Acc: %.1f m/s^2", max_a_);
}