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

// TODO add service to receive the global coordinates of the goal position

BasicPlanner::BasicPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        max_v_(0.2),
        max_a_(0.2),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity()) 
{
    ROS_WARN("[INIT]: Starting Trajectory Planner ...");

    // Initialize parameters
    state_ = 0;
    dimension_ = 4;
    planner_done_ = false;
    derivative_to_optimize_ = mav_trajectory_generation::derivative_order::SNAP;

    goal_velocity_ << 0, 0, 0, 0;
    goal_acceleration_ << 0, 0, 0, 0;
    // valid_states_ = {0, 1, 2}; ################# Change this after implementing perching
    valid_states_ = {0, 1};
    
    readParameters();
    initializePublishers();
    initializeSubscribers();
    initializeServices();

    ROS_WARN("[INIT]: Trajectory Planner is ready!");
    
}

void BasicPlanner::goalposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{  
    tf::poseMsgToEigen(msg->pose, goal_pose_affine_);
    goal_pose_ << goal_pose_affine_.translation(),
        mav_msgs::yawFromQuaternion((Eigen::Quaterniond)goal_pose_affine_.rotation());
}


void BasicPlanner::localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    tf::poseMsgToEigen(msg->pose, current_pose_);
}

void BasicPlanner::localvelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    tf::vectorMsgToEigen(msg->twist.linear, current_velocity_);
}

/*
void BasicPlanner::plannerTriggerCallback(const std_msgs::Bool::ConstPtr& msg)
{
    planner_active_ = msg->data;
    
    bool planned;
    bool published;
    
    if (planner_active_ && !planner_done_){

        mav_trajectory_generation::Trajectory trajectory;

        planned = planTrajectory(&trajectory);
        published = publishTrajectory(trajectory);
    
        ROS_WARN_STREAM("[PLANNER] TRAJECTORY PUBLISHED");
        planner_done_ = planned && published;
    } 
    //else if (!planner_active_ && planner_done_)
    else if (planner_done_)
    {
        planner_done_ = false;
        ROS_WARN_STREAM("[PLANNER] PLANNER DONE!");
    }
    else
    {
        // ROS_WARN_STREAM("[PLANNER] PLANNER WAITING ...");
        // Waiting for the trigger IDLE for planner
    }
    
}
*/

bool BasicPlanner::getActiveFlag()
{
    return planner_active_;
}

void BasicPlanner::setMaxSpeed(const double max_v) 
{
    max_v_ = max_v;
}

bool BasicPlanner::add_vertex(mav_trajectory_generation::Vertex::Vector *p_vertices, // pointer to vertices
                                mav_trajectory_generation::Vertex *p_middle,  // pointer to middle
                                Eigen::Vector4d pos = Eigen::Vector4d(-1, -1, -1, -1),  // some defaults
                                Eigen::Vector4d vel = Eigen::Vector4d(-1, -1, -1, -1),
                                Eigen::Vector4d acc = Eigen::Vector4d(-1, -1, -1, -1)) 
{

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

bool BasicPlanner::planTrajectory(mav_trajectory_generation::Trajectory* trajectory) 
{
	assert(trajectory);
	trajectory->clear();

    // Array for all waypoints and their constraints
    mav_trajectory_generation::Vertex::Vector vertices;

    // Start:   current position
    // Middle:  all middle waypoints with optional derivative constraints
    // End:     final position with all derivates equal zero  
    mav_trajectory_generation::Vertex start(dimension_), middle(dimension_), end(dimension_);

    /******* Configure start point *******/
	Eigen::Vector4d current_p, current_v;
    current_p << current_pose_.translation(),mav_msgs::yawFromQuaternion((Eigen::Quaterniond)current_pose_.rotation());
	current_v << current_velocity_, 0.0;
	// Set start point constraints to current position 
    start.makeStartOrEnd(current_p, derivative_to_optimize_);
    // Set all derivatives to zero
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_v);
    // Add waypoint to list
    vertices.push_back(start);    

    
    /******* Configure trajectory *******/
    Eigen::Vector4d pos, vel, acc;
    bool no_fail = true;
    int i = 0;

    switch (state_)
    {

    case BasicPlanner::WAYPOINT:
    {
        /******* Configure end point *******/
        // Set end point constraints to desired position
        end.makeStartOrEnd(goal_pose_, derivative_to_optimize_);
        // Set all derivatives to zero
        end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, goal_velocity_);
        end.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, goal_acceleration_);
        // Add waypoint to list
        vertices.push_back(end);
      }
    break;
    case BasicPlanner::TRAJECTORY:
    {
        while (no_fail){
            // std::cout<<"wp "<<i<<": preparing for import"<<std::endl;
            std::string idx = "/"+ std::to_string(i) + "/";
            std::string wp_str = ros::this_node::getName() + "/waypoints" + idx;

            // positions
            float pos_x, pos_y, pos_z, pos_h;
            no_fail = nh_.getParam(wp_str + "pos/x", pos_x);
            no_fail = nh_.getParam(wp_str + "pos/y", pos_y);
            no_fail = nh_.getParam(wp_str + "pos/z", pos_z);
            no_fail = nh_.getParam(wp_str + "pos/h", pos_h);
            if (!no_fail){
                // std::cout<<"      failed to add waypoint"<<std::endl;
                // std::cout<<"   -> ending import loop"<<std::endl;
                // std::cout<<"      now starting to compute trajectory"<<std::endl;
                ROS_WARN("[PLANNER] RECEIVED ALL WAYPOINTS, COMPUTING TRAJECTORY");
                break;
            }
            pos << pos_x, pos_y, pos_z, pos_h;  // write position constraints to vector
            // std::cout<<"      position constraints received"<<std::endl;
            // std::cout<<"      wp pos: "<<pos_x<<" "<<pos_y<<" "<<pos_z<<" "<<pos_h<<std::endl;

            // velocities
            float vel_x, vel_y, vel_z, vel_h;
            no_fail = nh_.getParam(wp_str + "vel/x", vel_x);
            no_fail = nh_.getParam(wp_str + "vel/y", vel_y);
            no_fail = nh_.getParam(wp_str + "vel/z", vel_z);
            no_fail = nh_.getParam(wp_str + "vel/h", vel_h);
            if (!no_fail){
                // std::cout<<"      failed to add velocity constraint"<<std::endl;
                // std::cout<<"      wp "<<i<<" added with position constraint only"<<std::endl;
                // std::cout<<std::endl;
                BasicPlanner::add_vertex(&vertices, &middle, pos);
                no_fail = true;  // reset failure indicator for next while loop
                i++;  // increase counter since we added the waypoint with position constraints only
                continue;
            }
            vel << vel_x, vel_y, vel_z, vel_h;  // write velocity constraints to vector; we got the position at least!
            // std::cout<<"      velocity constraints received"<<std::endl;
            // std::cout<<"      wp vel: "<<vel_x<<" "<<vel_y<<" "<<vel_z<<" "<<vel_h<<std::endl;

            // accelerations
            float acc_x, acc_y, acc_z, acc_h;
            no_fail = nh_.getParam(wp_str + "acc/x", acc_x);
            no_fail = nh_.getParam(wp_str + "acc/y", acc_y);
            no_fail = nh_.getParam(wp_str + "acc/z", acc_z);
            no_fail = nh_.getParam(wp_str + "acc/h", acc_h);
            if (!no_fail){
                // std::cout<<"      failed to add acceleration constraint"<<std::endl;
                // std::cout<<"      wp "<<i<<" added with position and velocity constraints only"<<std::endl;
                // std::cout<<std::endl;
                BasicPlanner::add_vertex(&vertices, &middle, pos, vel);
                no_fail = true;  // reset failure indicator for next while loop; we got the position at least!
                i++;  // increase counter since we added the waypoint with position and velocity constraints only
                continue;
            }
            acc << acc_x, acc_y, acc_z, acc_h;  // write acceleration constraints to vector
            // std::cout<<"      acceleration constraints received"<<std::endl;
            // std::cout<<"      wp acc: "<<acc_x<<" "<<acc_y<<" "<<acc_z<<" "<<acc_h<<std::endl;
            // std::cout<<"      wp "<<i<<" added with position, velocity and acceleration constraints"<<std::endl;
            // std::cout<<std::endl;

            BasicPlanner::add_vertex(&vertices, &middle, pos, vel, acc);
            i++;  // increase counter
        }
      }
    break;

    case BasicPlanner::PERCHING:
    {
        // Parametric parabola or function till perching point
        // will be received from perception node
        // decide if the receiveing topic should be the same as waypoint case

      }
    break;

    }

    
    /*
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
    */

    /******* Estimate initial segment times *******/
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);
    
    // std::copy(segment_times.begin(), segment_times.end(), std::ostream_iterator<double>(std::cout," "));
    // std::cout<<std::endl;


    /******* Set up polynomial solver and compute trajectory *******/
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension_, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);

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

bool BasicPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory)
{
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance = 0.4; // Distance by which to separate additional markers. Set 0.0 to disable.
    std::string frame_id = "map";

    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    pub_markers_.publish(markers);

    // send trajectory to be executed on UAV
    mav_planning_msgs::PolynomialTrajectory4D msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
    msg.header.frame_id = "map";
    pub_trajectory_.publish(msg);

    return true;
}

void BasicPlanner::initializePublishers()
{

    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
    pub_trajectory_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);
}

void BasicPlanner::initializeSubscribers()
{

    // goal_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
    //         ("planner/goal_pose", 10, &BasicPlanner::goalposeCallback, this);

    // state_machine_sub_ = nh_.subscribe
    //         ("planner/activation", 1, &BasicPlanner::plannerTriggerCallback, this);

    local_pos_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &BasicPlanner::localposeCallback, this);

    local_pos_vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity", 10, &BasicPlanner::localvelCallback, this);
    
}

void BasicPlanner::initializeServices()
{
    trigger_service_ = nh_.advertiseService
            ("planner/trigger", &BasicPlanner::plannerTriggerServiceCallback,this);
    
}

bool BasicPlanner::plannerTriggerServiceCallback(planner::GetTrajectory::Request &req,
                                                 planner::GetTrajectory::Response &res)
{
    int key = req.type;
    // Check if the type is valid, if not land on current position
    if(std::count(valid_states_.begin(), valid_states_.end(), key)){
        state_ = req.type;
        if (key==0)
            goal_pose_ << req.x, req.y, req.z, req.h;
    }else{
        state_ = 0;
        Eigen::Vector4d current_p;
        current_p << current_pose_.translation(),mav_msgs::yawFromQuaternion((Eigen::Quaterniond)current_pose_.rotation());
        goal_pose_ << current_p(0), current_p(1), 0.0, current_p(3);
        ROS_ERROR("[PLANNER] INVALID TYPE, GOING DOWN!");
    }
    
    mav_trajectory_generation::Trajectory trajectory;

    planTrajectory(&trajectory);
    publishTrajectory(trajectory);
    
    ROS_WARN_STREAM("[PLANNER] TRAJECTORY PUBLISHED");

    res.success = true;

    return true;
}

void BasicPlanner::readParameters()
{
    nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_v", max_v_);
    ROS_WARN("[PLANNER] Max Vel: %.1f m/s", max_v_);

    nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_a", max_a_);
    ROS_WARN("[PLANNER] Max Acc: %.1f m/s^2", max_a_);
}
