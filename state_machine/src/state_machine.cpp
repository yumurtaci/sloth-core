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
 * @file state_machine.cpp
 *
 * @date 03 June 2022
 * 
 * @brief State Machine Mavros
 * 
 * @author Batuhan Yumurtaci <batuhan.yumurtaci@tum.de>
 */

#include <state_machine/state_machine.h>

#define FCU_TIME_BUFFER 3.0f

// TODO : Refactorize wpConvergence value with orientation and velocity
// TODO : Add heading for waypoints and convergence checks
// TODO : Geometric controller activation
// TODO : Trajectory state

namespace statemachine
{
  StateMachine::StateMachine(ros::NodeHandle nh) : nh_(nh)
  {
    ROS_INFO("[INIT]: Started State Machine");

    initializePublishers();
    initializeSubscribers();
    initializeServices();

    // Commands for ros services
    offb_mode_cmd_.request.custom_mode = "OFFBOARD";
    arm_cmd_.request.value = true;
    land_cmd_.request.yaw = 0;
    land_cmd_.request.latitude = 0;
    land_cmd_.request.longitude = 0;
    land_cmd_.request.altitude = 0;

    // Initialize the variables
    state_ = ARMED;

    n_wp_ = 0;
    current_wp_ = 0;
    wp_completed_ = false;

    // Initialize messages
    planner_trigger_msg_.data = false;

    // ENU frame is used -> PX4 transforms to NED
    des_pose_.pose.position.x = 0;
    des_pose_.pose.position.y = 0;
    des_pose_.pose.position.z = 0;

    // Read the take-off pose
    nh_.getParam(ros::this_node::getName() + "/takeoff/x", takeoff_pose_.pose.position.x);
    nh_.getParam(ros::this_node::getName() + "/takeoff/y", takeoff_pose_.pose.position.y);
    nh_.getParam(ros::this_node::getName() + "/takeoff/z", takeoff_pose_.pose.position.z);
    
    readWaypoints();
    initializeFCU();                   
    
    // Start state machine
    if (current_state_.armed){
      if(getInput() == 0) 
          state_ = ARMED;
      requestKeyboardInput();
    } else {
      setInput(100);
      }  

    while (ros::ok())
    {
      ros::spinOnce();
      updateState();
      publishCommand();
      rate_.sleep();
    }

  }

  // Define Callback functions
  void StateMachine::stateCallback(const mavros_msgs::State::ConstPtr &msg)
  {
    current_state_ = *msg;
    }
  
  void StateMachine::localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    current_pose_ = *msg;
    }

  void StateMachine::localvelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
  {
    current_vel_ = *msg;
    }

  void StateMachine::updateState()
  {
    switch (state_)
    {

    case StateMachine::ARMED:
    {
      // Arming sequence
      if (current_state_.armed){
        ROS_INFO("[ARMED] : Press ENTER for Take-off: ");
        if(getInput() == 0) 
            state_ = TAKEOFF;
        requestKeyboardInput();
      } else {
        ROS_INFO("[ARMED] : Arming ...");
        setInput(100);
        }
      }
    break;

    case StateMachine::TAKEOFF:
    {
      // Climb to a user defined safe altitude at the current position
      // Once the altitude is reached go to the take off location

      bool arrivedWP = false;
      arrivedWP = wpConvergence( current_pose_, takeoff_pose_);
      
      if (arrivedWP){
        ROS_INFO("[TAKEOFF] : Take-off completed!");
        if(getInput() == 0)
            state_ = TRAJECTORY;
        requestKeyboardInput(); 
      } else {
        ROS_INFO("[TAKEOFF] : Fasten your seatbelts ...");
        setInput(100);
        }
      }
    break;

    case StateMachine::WAYPOINT:
    {
      // Follow the waypoints in the YAML file 
      // If the mission is completed ask the user to switch to next state
      
      bool arrivedWP = false;
      arrivedWP = wpConvergence( current_pose_, des_pose_);
      
      if(arrivedWP && current_wp_ < n_wp_){
        ROS_INFO("[WAYPOINT] Arrived at WP %d!", current_wp_);
        if(getInput() == 0)
            current_wp_++;
        requestKeyboardInput();
      } else if (arrivedWP && (current_wp_ == n_wp_)){ //n_wp_-1
        ROS_INFO("[WAYPOINT] Mission completed!");
        if(getInput() == 0)
          state_ = TRAJECTORY;
        requestKeyboardInput();
      } else {
        ROS_INFO("[WAYPOINT] Flying to WP %d ...", current_wp_);
        setInput(100);
      }
      
      }
    break;

    case StateMachine::TRAJECTORY:
    {
      bool arrivedWP = false;
      arrivedWP = wpConvergence( current_pose_, des_pose_);

      if (arrivedWP){
        ROS_INFO("[TRAJECTORY] : Trajectory state!");
        if(getInput() == 0)
            state_ = WAYPOINT;
        requestKeyboardInput(); 
      } else {
        ROS_INFO("[TRAJECTORY] : Following trajectory!");
        setInput(100);
        }

      }
    break;
    
    case StateMachine::LANDING:
    {
      // Lands at the current position

      // If the system status is MAV_STATE_ACTIVE
      if (current_state_.system_status == 4){
        ROS_INFO("[LANDING] : Going down ...");
      } else {
        ROS_INFO("[LANDING] : Landing completed!");
        }

      }
    break;
    
    }
  }
  
  void StateMachine::publishCommand()
  {
    switch (state_)
    {

    case StateMachine::ARMED:
    {
        
      local_pos_pub_.publish(des_pose_);

      // Take-off preparation by grabbing the current pose and target altitude
      localtakeoff_pose_.pose.position.x = current_pose_.pose.position.x;
      localtakeoff_pose_.pose.position.y = current_pose_.pose.position.y;
      localtakeoff_pose_.pose.position.z = takeoff_pose_.pose.position.z;
        
      }
    break;

    case StateMachine::TAKEOFF:
    {
    
      // Go to local origin at take-off height
      if (abs(current_pose_.pose.position.z - takeoff_pose_.pose.position.z) < 0.10 ){ 
        local_pos_pub_.publish(takeoff_pose_);
      } else {
        // Climb to the take-off altitude at current position
        local_pos_pub_.publish(localtakeoff_pose_);
        }
    
      }
    break;

    case StateMachine::WAYPOINT:
    { 
      // Publish from waypoint matrix till all waypoints are completed
      if (current_wp_ < n_wp_){
        des_pose_.pose.position.x = wp_matrix_(0, current_wp_);
        des_pose_.pose.position.y = wp_matrix_(1, current_wp_);
        des_pose_.pose.position.z = wp_matrix_(2, current_wp_);
      }
      
      planner_trigger_msg_.data = false;
      planner_trigger_pub_.publish(planner_trigger_msg_);
      local_pos_pub_.publish(des_pose_);
      
      }
    break;

    case StateMachine::TRAJECTORY:
    {
      // TODO Add basic trajectory generation; minimum snap

      des_pose_.pose.position.x = 0.0;
      des_pose_.pose.position.y = 0.0;
      des_pose_.pose.position.z = 2.0;
      
      planner_trigger_msg_.data = true;
      planner_trigger_pub_.publish(planner_trigger_msg_);
      local_pos_pub_.publish(des_pose_);
      
      }
    break;

    case StateMachine::LANDING:
    {
      // Call the mavros landing service
      if (!land_cmd_.response.success){
        land_client_.call(land_cmd_);
        }
     
      }
    break;
    
    }
  }

  void StateMachine::readWaypoints()
  {
    // Read the waypoints from YAML file
    double pos_x_wp;
    double pos_y_wp;
    double pos_z_wp;
    double pos_h_wp;
    bool no_fail = true;

    ROS_INFO("[INIT]: Reading waypoints ...");
    while(ros::ok() && no_fail){
      
      std::string idx = "/"+ std::to_string(n_wp_) + "/";
      std::string wp_str = ros::this_node::getName() + "/waypoints" + idx;

      //std::cout << wp_str << std::endl;
      no_fail = nh_.getParam(wp_str + "pos/x", pos_x_wp);

      if (!no_fail){
        //std::cout << "Number of waypoints: " << n_wp_ << std::endl;
        ROS_INFO("[INIT]: Number of waypoints: %d", n_wp_);  
        break;
      }else{ 
        n_wp_++;
      }
    }

    // Init Eigen::Matrix4Xd
    wp_matrix_.resize(4, n_wp_);
    
    for (int i = 0; i < n_wp_; i++){
      
      std::string idx = "/"+ std::to_string(i) + "/";
      std::string wp_str = ros::this_node::getName() + "/waypoints" + idx;

      no_fail = nh_.getParam(wp_str + "pos/x", pos_x_wp);
      no_fail = nh_.getParam(wp_str + "pos/y", pos_y_wp);
      no_fail = nh_.getParam(wp_str + "pos/z", pos_z_wp);
      no_fail = nh_.getParam(wp_str + "pos/h", pos_h_wp);
      
      if(no_fail){
        wp_matrix_.col(i) << pos_x_wp, pos_y_wp, pos_z_wp, pos_h_wp;
      }else{
        ROS_ERROR("[INIT]: Reading waypoints failed!");
          
      }
    }

    ROS_INFO("[INIT]: Waypoints ready");

  }

  void StateMachine::initializeFCU()
  {
    // FCU Initialisation Sequence 
    ros::Time last_request = ros::Time::now();

    // Wait for FCU connection before publishing anything
    while(ros::ok() && !current_state_.connected && 
    (ros::Time::now() - last_request > ros::Duration(5.0))){
      ros::spinOnce();
      rate_.sleep();
      ROS_INFO("[INIT]: Connecting to FCT ...");
    }
    
    // Send a few setpoints before starting, 
    // Otherwise cant switch to Offboard
    for(int i = 100; ros::ok() && i > 0; --i){
      local_pos_pub_.publish(des_pose_);
      ros::spinOnce();
      rate_.sleep();
    }

    // Timer used to prevent FCU overflowing
    //ros::Time last_request = ros::Time::now();
    last_request = ros::Time::now();
        
    local_pos_pub_.publish(des_pose_);

    // Change to Offboard mode and arm w. time gaps
    while(ros::ok() && !current_state_.armed){
      if( current_state_.mode != "OFFBOARD" && 
      (ros::Time::now() - last_request > ros::Duration(FCU_TIME_BUFFER))){
          if( set_mode_client_.call(offb_mode_cmd_) && offb_mode_cmd_.response.mode_sent){
              ROS_INFO("[INIT]: Offboard enabled");
          }
          last_request = ros::Time::now();
      } else {
          if( current_state_.mode == "OFFBOARD" && !current_state_.armed &&
              (ros::Time::now() - last_request > ros::Duration(FCU_TIME_BUFFER))){
              if( arming_client_.call(arm_cmd_) &&
                  arm_cmd_.response.success){
                  ROS_INFO("[INIT]: Vehicle armed");
              }
              last_request = ros::Time::now();
          }
      }
      local_pos_pub_.publish(des_pose_);
      ros::spinOnce();
      rate_.sleep();
    }   
  }

  void StateMachine::initializePublishers()
  {
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 1);
    planner_trigger_pub_ = nh_.advertise<std_msgs::Bool>
            ("planner_activation", 1);
  }

  void StateMachine::initializeSubscribers()
  {
    state_sub_ = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &StateMachine::stateCallback, this);

    local_pos_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &StateMachine::localposeCallback, this);
    
    local_pos_vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity", 10, &StateMachine::localvelCallback, this);
  }

  void StateMachine::initializeServices()
  {
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
  }

  bool StateMachine::wpConvergence(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
  {
    // TODO : Add velocity and orientation checks
    
    Vec3 v1;
    Vec3 v2;

    v1(0) = pose1.pose.position.x;
    v1(1) = pose1.pose.position.y;
    v1(2) = pose1.pose.position.z;

    v2(0) = pose2.pose.position.x;
    v2(1) = pose2.pose.position.y;
    v2(2) = pose2.pose.position.z;

    bool converged = (v1 - v2).norm() < 0.10;

    return converged;
  }

  void StateMachine::requestKeyboardInput()
  {
    if (request_input_)
      return;
    setInput(100);
    std::thread keyboard(&StateMachine::getKeyboardInput, this);
    keyboard.detach();
  }

  int StateMachine::getInput()
  {
    int input;
    input_mutex_.lock();
    input = input_;
    input_mutex_.unlock();
    return input;
  }

  void StateMachine::setInput(int input)
  {
    input_mutex_.lock();
    input_ = input;
    input_mutex_.unlock();
  }

  void StateMachine::getKeyboardInput()
  {
    request_input_ = true;
    int input;
    std::cin.ignore();
    setInput(0);
    request_input_ = false;
  }
} // namespace statemachine
