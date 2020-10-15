#include "controller.h"

using namespace automatic_parking;

// Setting linear and angular velocity
void autodock_controller::setVel(float x, float y, float yaw, boost::array<float,2> robot_point){
    static geometry_msgs::msg::Twist vel_msg;
    double ideal_theta_1 = 0;
    double ideal_theta_2 = -1.57; //angle 
    double angle_threshold = 0.05; //maximum error of angle
   
    if (step == 0){
        vel_msg.linear.x = 0;
        RCLCPP_INFO(get_logger(),"docking......, step_1_spin_%d",step1_count);
        if (((-(ideal_theta_1)-angle_threshold)<=yaw) && (yaw<=(-(ideal_theta_1)+angle_threshold))){
            vel_msg.angular.z = 0;
            vel_pub->publish(vel_msg);
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
        RCLCPP_INFO(get_logger(),"docking......, step_1_move_%d",step1_count);
        
        //printf("%f\n",dist(robot_point_temp,robot_point));
        if ((fabs(x) <= dist_to_center) or (dist(robot_point_temp,robot_point) >= fabs(x_origin/split_num))){
            vel_msg.linear.x = 0;
            vel_pub->publish(vel_msg);
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
        RCLCPP_INFO(get_logger(),"docking......, step_2_spin_%d",step2_count);
         if (((-(ideal_theta_2)-angle_threshold)<=yaw) && (yaw<=(-(ideal_theta_2)+angle_threshold))){
            vel_msg.angular.z = 0;
            vel_pub->publish(vel_msg);
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
        RCLCPP_INFO(get_logger(),"docking......, step_2_move_%d",step2_count);
        
        if ((fabs(x)<dist_to_dock) or (dist(robot_point_temp,robot_point) >= fabs(x_origin/split_num)) or ((fabs(y) > tune_distense) and (fabs(x) <= tune_threshold) )){
            vel_msg.linear.x = 0;
            vel_pub->publish(vel_msg);
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
        vel_pub->publish(vel_msg);
        RCLCPP_INFO(get_logger(),"Finish Docking!");
    }
    vel_pub->publish(vel_msg);
}

void autodock_controller::receive_tf(){
    while(rclcpp::ok()){
        try{
            //autodock_controller::tf_listener_.waitForTransform("base_link","dock_frame",rclcpp::Time(0),rclcpp::Duration(3.0));
            tf_dock = buffer_dock->lookupTransform("base_link","dock_frame",tf2::TimePoint(std::chrono::seconds(0)),tf2::Duration(std::chrono::seconds(3)));
            //tf_dock = buffer_dock->lookupTransform("base_link","dock_frame",tf2::TimePoint(std::chrono::milliseconds(3000)),tf2::Duration(3));
            break;
        }
        catch (tf2::TransformException &ex){
            RCLCPP_ERROR(get_logger(),"%s",ex.what());
            RCLCPP_ERROR(get_logger(),"Did not find the pattern!");
            rclcpp::sleep_for(std::chrono::seconds(3));
        }
    }
    //tf_listener_.waitForTransform("odom","base_link",ros::Time(0),ros::Duration(3.0));
    tf_odom = buffer_odom->lookupTransform("odom","base_link",tf2::TimePoint(std::chrono::milliseconds(0)),tf2::Duration(std::chrono::seconds(3)));

    //tf_odom = buffer_odom->lookupTransform("odom","base_link",tf2::Duration(3));

}
void autodock_controller::run(){
    receive_tf();
    // Dock_frame's origin and yaw
    float dock_x = tf_dock.transform.translation.x;
    float dock_y = tf_dock.transform.translation.y;
    float dock_yaw = tf2::getYaw(tf_dock.transform.rotation);
    odom[0] = tf_odom.transform.translation.x;
    odom[1] = tf_odom.transform.translation.y;
    setVel(dock_x, dock_y, dock_yaw, odom);
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto autodock_controller_node = std::make_shared<automatic_parking::autodock_controller>();
    rclcpp::WallRate rate(20.0);

    while(rclcpp::ok()){
        autodock_controller_node->run();
        rclcpp::spin_some(autodock_controller_node);
        rate.sleep();
    }
    return 0;
}