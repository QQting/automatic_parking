#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <chrono>
#include <boost/array.hpp>
#include "controller.h"

// Setting linear and angular velocity
void setVel(float x, float y, float yaw, auto robot_point){
    static geometry_msgs::Twist vel_msg;
    double ideal_theta_1 = 0;
    double ideal_theta_2 = -1.57; //angle 
    double angle_threshold = 0.01; //maximum error of angle
   
    if (step == 0){
        vel_msg.linear.x = 0;
        ROS_INFO("docking......, step_1_spin_%d",step1_count);
        if (((-(ideal_theta_1)-angle_threshold)<=yaw) && (yaw<=(-(ideal_theta_1)+angle_threshold))){
            vel_msg.angular.z = 0;
            vel_pub.publish(vel_msg);
            step = 1;

            if (step1_count==0){
                x_origin = x;
                robot_point_temp = robot_point ;
            }  
        }
        else if (((-(ideal_theta_1)+angle_threshold)<=yaw) && (yaw<(-(ideal_theta_1)+threshold_w))){
            vel_msg.angular.z = min_w;
        }
        else if (((-(ideal_theta_1)+threshold_w)<=yaw) && (yaw< 3.14)){
            vel_msg.angular.z = max_w;
        }
        else if (((-(ideal_theta_1)-threshold_w)<=yaw) && (yaw<(-(ideal_theta_1)-angle_threshold))){
            vel_msg.angular.z = -min_w;
        }
        else {
            vel_msg.angular.z = -max_w;
        }
    }
    
    else if (step == 1){
        ROS_INFO("docking......, step_1_move_%d",step1_count);
        
        //printf("%f\n",dist(robot_point_temp,robot_point));
        if ((fabs(x) <= dist_to_center) or (dist(robot_point_temp,robot_point) >= fabs(x_origin/split_num))){
            vel_msg.linear.x = 0;
            vel_pub.publish(vel_msg);
            step1_count += 1;
            if (fabs(x) <= dist_to_center){
                step = 2;
            }
            else{
                robot_point_temp = robot_point;
                step = 0;
            }
            
            
        }

        else if (x<0){
            
            if ((dist_to_center<fabs(x)) && (fabs(x)<(dist_to_center+threshold_v))){
                vel_msg.linear.x = -min_v;
            }

            else {
                vel_msg.linear.x = -max_v;
            }
        }
        else if (x>0){
            
            if ((dist_to_center<fabs(x)) && (fabs(x)<(dist_to_center+threshold_v))){
                vel_msg.linear.x = min_v;
                }
            else {
                vel_msg.linear.x = max_v;
                }
        }    
           
    }
    else if (step == 2){
        ROS_INFO("docking......, step_2_spin_%d",step2_count);
         if (((-(ideal_theta_2)-angle_threshold)<=yaw) && (yaw<=(-(ideal_theta_2)+angle_threshold))){
            vel_msg.angular.z = 0;
            vel_pub.publish(vel_msg);
            step = 3;
            if (step2_count==0){
                x_origin = x;
                robot_point_temp = robot_point ;
            } 
        }
        else if (((-(ideal_theta_2)+angle_threshold)<=yaw) && (yaw<(-(ideal_theta_2)+threshold_w))){
            vel_msg.angular.z = min_w;
        }
        else if (((-(ideal_theta_2)+threshold_w)<=yaw) && (yaw<(3.14))){
            vel_msg.angular.z = max_w;
        }
        else if (((-(ideal_theta_2)-threshold_w)<=yaw) && (yaw<(-(ideal_theta_2)-angle_threshold))){
            vel_msg.angular.z = -min_w;
        }
        else {
            vel_msg.angular.z = -max_w;
        }
    }
    else if (step == 3){
        ROS_INFO("docking......, step_2_move_%d",step2_count);
        
        if ((fabs(x)<dist_to_dock) or (dist(robot_point_temp,robot_point) >= fabs(x_origin/split_num)) or ((fabs(y) > tune_distense) and (fabs(x) <= tune_threshold) )){
            vel_msg.linear.x = 0;
            vel_pub.publish(vel_msg);
            step2_count += 1;
            if (fabs(x)<dist_to_dock){
                step = 4;
            }
            else if ((fabs(y) > dist_to_center) and (fabs(x) <= 0.6)){
                step = 0;
                robot_point_temp = robot_point;
            }
            else{
                robot_point_temp = robot_point;
                step = 2;
            }
            
        }
        else if (fabs(x)<=(dist_to_dock+threshold_v)){
            vel_msg.linear.x = -min_v;
        }
        else {
            vel_msg.linear.x = -max_v;
        }
    }
    else if (step == 4){
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        vel_pub.publish(vel_msg);
        ROS_INFO("Finish Docking!");
        step = 5;
    }
    vel_pub.publish(vel_msg);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle n_;

    // Load Parameters
    n_.param<std::string>("base_frame_id",base_frame_id, "base_link");
    n_.param<double>("min_v",min_v, 0.1);
    n_.param<double>("min_w",min_w, 0.1);
    n_.param<double>("max_v",max_v, 0.3);
    n_.param<double>("max_w",max_w, 0.3);
    n_.param<double>("dist_to_dock",dist_to_dock, 0.30);
    n_.param<double>("dist_to_center",dist_to_center, 0.03);
    n_.param<double>("threshold_v",threshold_v, 0.3);
    n_.param<double>("threshold_w",threshold_w, 0.4);
    
    vel_pub = n_.advertise<geometry_msgs::Twist>("cmd_vel", 20);
    tf::TransformListener listener_dock;
    ros::Rate rate(20.0);
    while(ros::ok()){

        if (step == 5) {
            break;
        }

        tf::StampedTransform tf_dock;
        tf::StampedTransform tf_odom;
        try {
            listener_dock.waitForTransform("base_link","dock_frame",ros::Time(0),ros::Duration(3.0));
            listener_dock.lookupTransform("base_link","dock_frame",ros::Time(0),tf_dock);
        }
        catch (tf::TransformException &ex) {
            // ROS_WARN("%s",ex.what());
            ROS_WARN("Can't recognize the docking station, please move the robot to be closer.");
            ros::Duration(1.0).sleep();
            continue;
        }
        listener_dock.waitForTransform("odom","base_link",ros::Time(0),ros::Duration(3.0));
        listener_dock.lookupTransform("odom","base_link",ros::Time(0),tf_odom);
        // Dock_frame's origin and yaw
        float dock_x = tf_dock.getOrigin().x();
        float dock_y = tf_dock.getOrigin().y();
        float dock_yaw = tf::getYaw(tf_dock.getRotation());
        odom[0] = tf_odom.getOrigin().x();
        odom[1] = tf_odom.getOrigin().y();
        ros::spinOnce();
        setVel(dock_x, dock_y, dock_yaw, odom);
        rate.sleep();
    }
    return 0;
}