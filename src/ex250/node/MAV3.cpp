#include "ros/ros.h"
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Bool.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Mavlink.h>
#include "std_msgs/Int16.h"
#define KILL 2
mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;

std_msgs::Bool take_single;
std_msgs::Int16 traj;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void trajectory_mode_cb(const std_msgs::Int16::ConstPtr& msg)
{
    traj = *msg;
}

//void takeoff_cb(const std_msgs::Bool::ConstPtr& msg)
//{
  //  take_single = *msg;
//}



int main(int argv,char** argc)
{
    ros::init(argv,argc,"MAV3");
    ros::NodeHandle nh;

    ros::Subscriber trajectory_mode = nh.subscribe<std_msgs::Int16>
        ("/system/trajectory",1000,trajectory_mode_cb); 
   
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 1000, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 1000);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

  //  ros::Subscriber wait_takeoff =nh.subscribe<std_msgs::Bool>
       // ("/MAV/takeoff",10,takeoff_cb);
    ros::Rate rate(100);

    
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
 
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;

    ros::topic::waitForMessage<mavros_msgs::Mavlink>("mavlink/to");
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

   // sleep(3);

    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("GUIDED enabled");
    }

    if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    }
    ros::topic::waitForMessage<std_msgs::Bool>("/MAV/takeoff");
    
    // ros::Time last_request = ros::Time::now();
    // while(ros::ok()){
    //     if( current_state.mode != "OFFBOARD" &&
    //         (ros::Time::now() - last_request > ros::Duration(10))){
    //         if( set_mode_client.call(offb_set_mode) &&
    //             offb_set_mode.response.mode_sent){
    //             ROS_INFO("GUIDED enabled");
    //         }
    //         last_request = ros::Time::now();
    //     } else {
    //         if( !current_state.armed &&
    //             (ros::Time::now() - last_request > ros::Duration(10))){
    //             if( arming_client.call(arm_cmd) &&
    //                 arm_cmd.response.success){
    //                 ROS_INFO("Vehicle armed");
    //             }
    //             last_request = ros::Time::now();
    //         }
    //     }
    while(ros::ok() && traj.data != 2){
        
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();   
    }
    //pose.pose.position.z = 5;
    //ros::Time time_out = ros::Time::now();
    //while (ros::ok() && ros::Time::now()-time_out < ros::Duration(10.0))
    //{
     //   local_pos_pub.publish(pose);
     //   ros::spinOnce();
     //   rate.sleep();

    //}
    

    ROS_WARN("kill!");
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    set_mode_client.call(offb_set_mode);
    arm_cmd.request.value = false;
    arming_client.call(arm_cmd);
    sleep(3);
    return 0;
}