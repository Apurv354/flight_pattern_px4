/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>

float x,y, theta;
int flag = 0;
double roll, pitch, yaw;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void First_Circle(){
	    x = 10 + 4 * cos(theta);
        y = 12 + 4 * sin(theta);
}

void Second_Circle(){
	x = 10 + 4 * cos(-theta);
    y = 20 + 4 * sin(-theta);
}

int main(int argc, char **argv)
{
	flag =0;
    ros::init(argc, argv, "offb_node_1");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");

    //Quat

    tf2::Quaternion myQuaternion;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(6.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 10;
    pose.pose.position.y = 12;
    pose.pose.position.z = 5;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 0;

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

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
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
                ROS_INFO("\n \n \n Task 3 : Flying Drone in a Circle of 4m around (10,12) and (10, 20) in a figure of 8 \n");
                last_request = ros::Time::now();
            }
        }

        if(flag == 0){
        	First_Circle();
        }
        else{
        	Second_Circle();
        }

        if (theta>6.28){
            theta = 0;
            if (flag == 0){
            	flag = 1;
            }
            else{
            	flag = 0;
            }
        }
        theta = theta + 0.1225;
        myQuaternion.setRPY( 0, 0, theta);
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.x = myQuaternion[0];
        pose.pose.orientation.y = myQuaternion[1];
        pose.pose.orientation.z = myQuaternion[2];
        pose.pose.orientation.w = myQuaternion[3];
        //ROS_INFO("\n %f \n ", cos(theta));
        //ROS_INFO("\n %f \n ", x);
        //ROS_INFO_STREAM(myQuaternion[0]);
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
