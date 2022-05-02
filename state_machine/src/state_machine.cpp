#include <state_machine/state_machine.h>

namespace statemachine
{

  StateMachine::StateMachine(ros::NodeHandle nh) : nh_(nh)
  {
    ROS_INFO("Started State Machine");
    ros::Rate rate(100);

    // Publishers
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 1);

    // Subscribers
    state_sub_ = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &StateMachine::stateCallback, this);
    local_pos_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &StateMachine::localposeCallback, this);
        
    // Services
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    land_client_ = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    
    // Initialize the variables
    state_ = ARMED; 

    // Commands for ros services
    offb_mode_cmd_.request.custom_mode = "OFFBOARD";
    arm_cmd_.request.value = true;
    disarm_cmd_.request.value = false;
    land_cmd_.request.yaw = 0;
    land_cmd_.request.latitude = 0;
    land_cmd_.request.longitude = 0;
    land_cmd_.request.altitude = 0;

    // ENU frame is used -> PX4 transforms to NED
    des_pose_.pose.position.x = 0;
    des_pose_.pose.position.y = 0;
    des_pose_.pose.position.z = 0;

    takeoff_pose_.pose.position.x = 0;
    takeoff_pose_.pose.position.y = 0;
    takeoff_pose_.pose.position.z = 1.5;

    // FCU Initialisation Sequence 
    ros::Time last_request = ros::Time::now();

    // Wait for FCU connection before publishing anything
    while(ros::ok() && !current_state_.connected && 
    (ros::Time::now() - last_request > ros::Duration(5.0))){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("[INIT]: Connecting to FCT...");
    }
    
    // Send a few setpoints before starting, 
    // Otherwise cant switch to Offboard
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub_.publish(des_pose_);
        ros::spinOnce();
        rate.sleep();
    }

    // Timer used to prevent FCU overflowing
    //ros::Time last_request = ros::Time::now();
    last_request = ros::Time::now();
        
    local_pos_pub_.publish(des_pose_);

    // Change to Offboard mode and arm w. 5 second gaps
    while(ros::ok() && !current_state_.armed){
        if( current_state_.mode != "OFFBOARD" && 
        (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client_.call(offb_mode_cmd_) && offb_mode_cmd_.response.mode_sent){
                ROS_INFO("[INIT]: Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( current_state_.mode == "OFFBOARD" && !current_state_.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client_.call(arm_cmd_) &&
                    arm_cmd_.response.success){
                    ROS_INFO("[INIT]: Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub_.publish(des_pose_);
        ros::spinOnce();
        rate.sleep();
    }

    // State Machine
    while (ros::ok())
    {
      ros::spinOnce();
      UpdateState();
      PublishCommand();
      rate.sleep();
    }

  }

  // Define Callback functions
  void StateMachine::stateCallback(const mavros_msgs::State::ConstPtr &msg)
  {
    this->current_state_ = *msg;

    }

  void StateMachine::localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
      this->current_pose_ = *msg;
    }

  void StateMachine::UpdateState()
  {
    switch (state_)
    {

    case StateMachine::ARMED:
    {
      // Arming sequence
      if (current_state_.armed){
          // Switch to TAKEOFF
          ROS_INFO("[ARMED] : Press ENTER for Take-off: ");
          if(getInput() == 0)
              state_ = TAKEOFF;
          requestKeyboardInput();       
      } else {
          ROS_INFO("[ARMED] : Arming...");
          setInput(100);
          }
      
      }
    break;

    case StateMachine::TAKEOFF:
    {
      // TODO add velocity convergence
      // TODO add heading convergence
      
      bool arrivedWP = false;
      arrivedWP = WPConvergence(current_pose_, takeoff_pose_);
      
      if (arrivedWP){
        ROS_INFO("[TAKEOFF] : Take-off completed!");
        if(getInput() == 0)
            state_ = WAYPOINT;
        requestKeyboardInput(); 
      } else {
        ROS_INFO("[TAKEOFF] : Fasten your seatbelts");
        setInput(100);
      }


          //if (reached_goal && speed_converged){
        //      ROS_INFO("[TAKEOFF] : Take-off completed!");
              //if(getInput() == 0)
              //    state_ = WAYPOINT;
              //requestKeyboardInput(); 
          /*
          } else {
              ROS_INFO("[AIRBONE] : Hovering ...");
              setInput(100);
          }
          */
        }
    break;

    case StateMachine::WAYPOINT:
    {
      ROS_INFO("[WAYPOINT] : Changing to LANDING!");
        
        state_ = TRAJECTORY;
        }
    break;

    case StateMachine::TRAJECTORY:
    {
        ROS_INFO("[WAYPOINT] : Changing to LANDING!");
        
        state_ = LANDING;
        }
    break;
    
    case StateMachine::LANDING:
    {
        ROS_INFO("[LANDING] : Going down!");
        
        state_= ARMED;
        }
    break;
    }
    }
  
  void StateMachine::PublishCommand()
  {
    switch (state_)
    {

    case StateMachine::ARMED:
    {
        
        local_pos_pub_.publish(des_pose_);
        }
    break;

    case StateMachine::TAKEOFF:
    {
        // bool arrivedWP = false;
        local_pos_pub_.publish(takeoff_pose_);
        // arrivedWP = WPConvergence(current_pose_, takeoff_pose_);
        
        // if (arrivedWP){
        //   ROS_INFO("ARRIVED");
        // } else {
        //   ROS_INFO("Arriving...");
        // }
        
        }
    break;

    case StateMachine::WAYPOINT:
    {
        
        }
    break;

    case StateMachine::TRAJECTORY:
    {
        
        }
    break;

    case StateMachine::LANDING:
    {
        local_pos_pub_.publish(takeoff_pose_);
        }
    break;
    }
  }

  bool StateMachine::WPConvergence(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
  {
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
}