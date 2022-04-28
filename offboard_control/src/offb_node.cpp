/*
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>

// Callback function to save the current mode
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
      ("mavros/cmd/land");

    // The setpoint publishing rate MUST be faster than 2Hz.
    // Otherwise, the commander might switch back to the last mode before Offboard. 
    // Therefore enter Offboard mode from Position mode. 
    ros::Rate rate(20.0);

    // Wait for FCU connection before publishing anything
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // ENU frame is used -> PX4 transforms to NED
    geometry_msgs::PoseStamped pose0;
    pose0.pose.position.x = 0;
    pose0.pose.position.y = 0;
    pose0.pose.position.z = 2;

    tf::Quaternion quaternion0;
    quaternion0 = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);
    pose0.pose.orientation.x = quaternion0.getX();
    pose0.pose.orientation.y = quaternion0.getY();
    pose0.pose.orientation.z = quaternion0.getZ();
    pose0.pose.orientation.w = quaternion0.getW();

    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 2;
    pose1.pose.position.z = 2;

    tf::Quaternion quaternion1;
    quaternion1 = tf::createQuaternionFromRPY(0.0, 0.0, 90.0);
    pose1.pose.orientation.x = quaternion1.getX();
    pose1.pose.orientation.y = quaternion1.getY();
    pose1.pose.orientation.z = quaternion1.getZ();
    pose1.pose.orientation.w = quaternion1.getW();

    // Send a few setpoints before starting, otherwise no switch to Offboard
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose0);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        // 5 seconds gap to avoid autopilot request overflow
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        // Continue publishing requested pose
        if( current_state.armed && 
            (ros::Time::now() - last_request < ros::Duration(10.0))){
                local_pos_pub.publish(pose0); 
                ROS_INFO("Waypoint 0");       
        } else {
            local_pos_pub.publish(pose1);
            ROS_INFO("Waypoint 1");
                if(ros::Time::now() - last_request > ros::Duration(20.0)){
                    ROS_INFO("Landing");
                    if( land_client.call(land_cmd) && land_cmd.response.success){
                        ROS_INFO("Landed");
                        arm_cmd.request.value = false;
                        ROS_INFO("Disarmed");
                    }  
                }
        }




        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
*/



#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define FLIGHT_ALTITUDE 1.5f

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
      ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT...");
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    ros::Time last_request = ros::Time::now();

    // change to offboard mode and arm
    while(ros::ok() && !current_state.armed){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
          ROS_INFO(current_state.mode.c_str());
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // go to the first waypoint
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("going to the first way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("first way point finished!");


    // go to the second waypoint
    pose.pose.position.x = 0;
    pose.pose.position.y = 1;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    //send setpoints for 10 seconds
    ROS_INFO("going to second way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("second way point finished!");

    // go to the third waypoint
    pose.pose.position.x = 1;
    pose.pose.position.y = 1;
    pose.pose.position.z = FLIGHT_ALTITUDE;
    //send setpoints for 10 seconds
    ROS_INFO("going to third way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("third way point finished!");
    
    // go to the forth waypoint
    pose.pose.position.x = 1;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;
    //send setpoints for 10 seconds
    ROS_INFO("going to forth way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("forth way point finished!");
    
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;
    ROS_INFO("going back to the first point!");
    //send setpoints for 10 seconds
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }

    ROS_INFO("tring to land");
    while (!(land_client.call(land_cmd) &&
            land_cmd.response.success)){
      //local_pos_pub.publish(pose);
      ROS_INFO("tring to land");
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}