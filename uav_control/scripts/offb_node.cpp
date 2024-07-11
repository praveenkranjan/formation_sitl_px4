/*
 * Copyright (c) [2024] [Praveen Kumar Ranjan], [UTSA]
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met: 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * Author: Praveen Kumar Ranjan
 * Public Release Date: [11/07/2024]
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <mavros_msgs/SetMode.h> // Include the service message header for MAVROS SetMode service
#include <mavros_msgs/CommandBool.h> // Include the service message header for MAVROS CommandBool service
#include <mavros_msgs/CommandTOL.h>


bool flag_home_set = false;
bool flag_sim_start = false;
double t_arm_ofb =5.0, t_start=10.0, t_end=30.0; //time parameters
double del_x =1, del_y=-3, del_z=-1; // Formation parametrs
double in_x_P=0, in_y_P=0, in_z_P=2, in_x_T=2, in_y_T=2, in_z_T=2;
double x_P_c = 0.0, y_P_c = 0.0, z_P_c = 0.0; // temp variables
double x_T_c = 0.0, y_T_c = 0.0, z_T_c = 0.0; // 
double vtx = 0.0, vty = 0.0, vtz = 0.0; // 
double vpx = 0.0, vpy = 0.0, vpz = 0.0; // 

double K1 = 0.7, K2= 0.7, K3 = 0.7; // Controller gains

mavros_msgs::PositionTarget setpointMsg_T;
mavros_msgs::PositionTarget setpointMsg_P;


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

mavros_msgs::State current_state_T;
void state_cb_T(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_T = *msg;
}

mavros_msgs::HomePosition hom_pos_P;
void homePositionCallback_P(const mavros_msgs::HomePosition::ConstPtr& msg)
{
    hom_pos_P = *msg;
}

nav_msgs::Odometry cur_x_P;
void x_P_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    cur_x_P = *msg;
}

nav_msgs::Odometry cur_x_T;
void x_T_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    cur_x_T = *msg;
}

double saturate(double value, double min_val, double max_val) 
{
    if (value < min_val) {
        return min_val;
    } else if (value > max_val) {
        return max_val;
    } else {
        return value;
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/uav0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");
    ros::Subscriber state_sub_T = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10, state_cb_T);
    ros::Publisher local_pos_pub_T = nh.advertise<geometry_msgs::PoseStamped>("/uav1/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client_T = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client_T = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");
    ros::Subscriber home_position_sub_P = nh.subscribe<mavros_msgs::HomePosition>("/uav0/mavros/home_position/home", 10, homePositionCallback_P);
    ros::Publisher set_gp_origin_pub_T = nh.advertise<geographic_msgs::GeoPointStamped>("/uav1/mavros/global_position/set_gp_origin", 10);
    ros::Subscriber x_P = nh.subscribe<nav_msgs::Odometry>("/uav0/mavros/local_position/odom", 10, x_P_cb);
    ros::Subscriber x_T = nh.subscribe<nav_msgs::Odometry>("/uav1/mavros/local_position/odom", 10, x_T_cb);
    ros::Publisher local_setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav0/mavros/setpoint_raw/local", 10);
    ros::Publisher local_setpoint_pub_T = nh.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 10);
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/uav0/mavros/cmd/land");
    ros::ServiceClient land_client_T = nh.serviceClient<mavros_msgs::CommandTOL>("/uav1/mavros/cmd/land");                 


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(40.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected && !current_state_T.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ros::Time last_request = ros::Time::now();
    
    // Synchronising origin for every vehicle

    if(!flag_home_set)
    {
        geographic_msgs::GeoPointStamped set_gp_msg;
        while (ros::Time::now() - last_request < ros::Duration(1.0))
        {
            set_gp_msg.position.latitude = hom_pos_P.geo.latitude;
            set_gp_msg.position.longitude = hom_pos_P.geo.longitude;
            set_gp_msg.position.altitude = hom_pos_P.geo.altitude;
            ROS_INFO("setting home");
            set_gp_origin_pub_T.publish(set_gp_msg); 
            // Setting origin of Target as the pursuer home 
            rate.sleep();        
        }             
               
        flag_home_set=true;
    }
    
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = in_x_P;
    pose.pose.position.y = in_y_P;
    pose.pose.position.z = in_z_P;

    geometry_msgs::PoseStamped pose_T;
    pose_T.pose.position.x = in_x_T;
    pose_T.pose.position.y = in_y_T;
    pose_T.pose.position.z = in_z_T;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        local_pos_pub_T.publish(pose_T);
        ros::spinOnce();
        rate.sleep();
    }    

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetMode offb_set_mode_T;
    offb_set_mode_T.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd_T;
    arm_cmd_T.request.value = true;

    last_request = ros::Time::now();

    //main Loop

    while(ros::ok())
    {
        // enabling offboaard mode and arming vehicle. Initialization Phase
        if( current_state.mode != "OFFBOARD" || current_state_T.mode != "OFFBOARD"  && (ros::Time::now() - last_request > ros::Duration(t_arm_ofb)))
        {
            // Pursuer offboard service
            if( current_state.mode != "OFFBOARD" && set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Pursuer Offboard enabled");
            }
            // Target offboard service
            if( current_state_T.mode != "OFFBOARD" && set_mode_client_T.call(offb_set_mode_T) && offb_set_mode_T.response.mode_sent)
            {
                ROS_INFO("Target Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed || !current_state_T.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                // Pursuer Arming service
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Pursuer armed");
                }
                // Target Arming service
                if( arming_client_T.call(arm_cmd_T) && arm_cmd_T.response.success)
                {
                    ROS_INFO("Target armed");
                }
                last_request = ros::Time::now();
            }
        }
        
        // Formation Guidance phase
        if(current_state.armed && current_state_T.armed && ros::Time::now() - last_request > ros::Duration(t_start) &&ros::Time::now() - last_request < ros::Duration(t_end))
        {
            
            flag_sim_start=true; 
            x_P_c = cur_x_P.pose.pose.position.x;
            y_P_c = cur_x_P.pose.pose.position.y;
            z_P_c = cur_x_P.pose.pose.position.z;

            x_T_c = cur_x_T.pose.pose.position.x;
            y_T_c = cur_x_T.pose.pose.position.y;
            z_T_c = cur_x_T.pose.pose.position.z;

            vtx = cur_x_T.twist.twist.linear.x;
            vty = cur_x_T.twist.twist.linear.y;
            vtz = cur_x_T.twist.twist.linear.z; 

            // control logic the control logic can be changed here

            vpx = saturate(vtx - K1*(x_P_c-x_T_c+del_x),-15,15);
            vpy = saturate(vty - K2*(y_P_c-y_T_c+del_y),-15,15);
            vpz = saturate(vtz - K3*(z_P_c-z_T_c+del_z),-15,15);

            setpointMsg_T.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            
            setpointMsg_T.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |   mavros_msgs::PositionTarget::IGNORE_PY |
                          mavros_msgs::PositionTarget::IGNORE_PZ |  mavros_msgs::PositionTarget::IGNORE_AFX |
                          mavros_msgs::PositionTarget::IGNORE_AFY |  mavros_msgs::PositionTarget::IGNORE_AFZ |
                          mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;
            setpointMsg_P.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED ;
            setpointMsg_P.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |   mavros_msgs::PositionTarget::IGNORE_PY |
                          mavros_msgs::PositionTarget::IGNORE_PZ |  mavros_msgs::PositionTarget::IGNORE_AFX |
                          mavros_msgs::PositionTarget::IGNORE_AFY |  mavros_msgs::PositionTarget::IGNORE_AFZ |
                          mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;
            

            setpointMsg_P.velocity.x=vpx;
            setpointMsg_P.velocity.y= vpy;
            setpointMsg_P.velocity.z= vpz;

            setpointMsg_T.velocity.x= 2.0;
            setpointMsg_T.velocity.y= 0.0;
            setpointMsg_T.velocity.z= 0.0;
            
            
        }

        // landing phase
        if(ros::Time::now() - last_request > ros::Duration(30.0) )
        {
            flag_sim_start=false;
            x_P_c = cur_x_P.pose.pose.position.x;
            y_P_c = cur_x_P.pose.pose.position.y;
            z_P_c = cur_x_P.pose.pose.position.z;

            x_T_c = cur_x_T.pose.pose.position.x;
            y_T_c = cur_x_T.pose.pose.position.y;
            z_T_c = cur_x_T.pose.pose.position.z;

            double dist_T = sqrt(pow(x_T_c - in_x_T, 2) + pow(y_T_c - in_y_T, 2) + pow(z_T_c - in_z_T, 2));
            double dist = sqrt(pow(x_P_c - in_x_P, 2) + pow(y_P_c - in_y_P, 2) + pow(z_P_c - in_z_P, 2));
            ROS_INFO("no hit Value_P : %f : value_T %f\n", dist,dist_T);
            // Initiating landing
            if (abs(dist)<0.3 && abs(dist_T)<0.3){ 
                ROS_INFO("hit");
                mavros_msgs::CommandTOL land_cmd;
                land_cmd.request.altitude=0;
                mavros_msgs::CommandTOL land_cmd_T;
                land_cmd_T.request.altitude=0;
                // Pursuer land service
                if (land_client.call(land_cmd) && land_cmd.response.success ) 
                {
                    ROS_INFO("Pursuer Landed");                    
                } 
                else 
                {
                    ROS_ERROR("Failed to call landing service");
                    }
                // Target land service
                if (land_client_T.call(land_cmd_T) && land_cmd_T.response.success ) 
                {
                    ROS_INFO("Target Landed");
                    
                } 
                else 
                {
                    ROS_ERROR("Failed to call landing service");
                }
                if (land_cmd.response.success && land_cmd_T.response.success )
                {
                    return 0;
                }
            }
        }
        // Publishing control setpoints
        if (flag_sim_start)
        {
            // only for Formation guidance phase
            local_setpoint_pub.publish(setpointMsg_P);
            local_setpoint_pub_T.publish(setpointMsg_T);
            
        } else {
            local_pos_pub.publish(pose);
            local_pos_pub_T.publish(pose_T);

        }              

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}