#include "pattern.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace automatic_parking;

bool autodock_pattern::calAngle(double a, double b, double angle_ab){
    double angle;
    angle = fabs(a-b);
     if (angle < M_PI and angle > 1.7 ){
        angle = angle - M_PI_2 ;
    }
    else if (angle> (M_PI_2 + M_PI)){ 
        angle = angle - M_PI - M_PI_2;
    }
    else if (angle> M_PI and angle < (M_PI_2 + M_PI)){
        angle = angle - M_PI ;
    }
    //RCLCPP_INFO(get_logger(),"angle:%f , ab:%f \n",angle,angle_ab);
    if (fabs(angle_ab-angle)<=detect_angle_tolerance){
        return true;}
    else return false;
}

void autodock_pattern::populateTF(double x, double y, double theta){
    // publish dock_frame
    auto now = get_clock()->now();
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    geometry_msgs::msg::TransformStamped tf_dock; 
    tf_dock.header.stamp = now; 
    tf_dock.header.frame_id = base_frame_id;
    tf_dock.child_frame_id = frame_name ;
    tf_dock.transform.translation.x = x;
    tf_dock.transform.translation.y = y;
    tf_dock.transform.translation.z = 0;
    tf_dock.transform.rotation.x = q.x(); 
    tf_dock.transform.rotation.y = q.y(); 
    tf_dock.transform.rotation.z = q.z(); 
    tf_dock.transform.rotation.w = q.w(); 
    tf_broadcaster_->sendTransform(tf_dock);
}

void autodock_pattern::updateVectors(){
    point_set = point_temp;
    //printf("Upate vectors!\n");
    RCLCPP_INFO(get_logger(),"Upate vectors!");
}

bool autodock_pattern::check_center(std::vector<int> &dock_vector , std::vector<far_docking_msgs::msg::LineSegment_<std::allocator<void>>> &vectors){
    for(int i=0; i<3; i++){
        for(int j(i+1); j<=3 ; j++){
            
            if (calAngle(vectors[dock_vector[i]].angle,vectors[dock_vector[j]].angle, 3.14-pattern_angle2)){
                return true;
            }
        }
    }    
    return false;
}

void autodock_pattern::temp_vector(int &i , int &j ,int &angle_count, std::vector<far_docking_msgs::msg::LineSegment_<std::allocator<void>>> &vectors ){
    
    if (angle_count == 1){
        point_temp.vector_a = mid_point(vectors[i]);
        point_temp.vector_b = mid_point(vectors[j]);
        dock_vector[0] = i;
        dock_vector[1] = j;

    }
    else{     
        dock_vector[2] = i;
        dock_vector[3] = j;
        point_temp.vector_d = mid_point(vectors[i]);
        point_temp.vector_c = mid_point(vectors[j]);
        temp_point_1 = mid_two_point(point_temp.vector_a , point_temp.vector_b);
        temp_point_2 = mid_two_point(point_temp.vector_c , point_temp.vector_d);
        //RCLCPP_INFO(get_logger(),"dist = %f\n",dist(temp_point_1,temp_point_2));
        if (dist(temp_point_1,temp_point_2) <= 0.3 and check_center(dock_vector , vectors)){
            //printf("%s\n", check_center(dock_vector , vectors ) ? "true" : "false");
            updateVectors();
            check_angle = true;
        }
    }  
}

void autodock_pattern::patternCallback(const far_docking_msgs::msg::LineSegmentList::SharedPtr msg){
    std::vector<far_docking_msgs::msg::LineSegment_<std::allocator<void>>> vectors = msg->line_segments;
   
    // Number of the line
    int lineNum = vectors.size();
    int angle_count = 0;
    bool check_vec_size = true;
    check_angle = false;
    
    //RCLCPP_INFO(get_logger(),"lineNum = %d\n",lineNum);

    // Check whether topic line_segments is publishing
    if (lineNum < 4){
        RCLCPP_ERROR(get_logger(),"There isn't enough line in the laser field!");
        check_vec_size = false;
    }
    if (check_vec_size){
        for(int i=0; i<lineNum; i++){
            for(int j(i+1); j<=lineNum ; j++){
                if (mid_dist(vectors[i] , vectors[j]) <= group_dist_tolerance) {
                    if (calAngle(vectors[i].angle,vectors[j].angle, pattern_angle1-3.14)){
                        angle_count+=1;
                        temp_vector(i , j ,angle_count , vectors );
                        //RCLCPP_INFO(get_logger(),"angle_count = %d\n",angle_count);
                        
                    }
                }
            }
        }
    }
    if (check_angle){
        //Set origin of frame
        
        boost::array<double, 2> theta_point_1 = mid_two_point(point_set.vector_a , point_set.vector_b);
        boost::array<double, 2> theta_point_2 = mid_two_point(point_set.vector_c , point_set.vector_d);
        boost::array<double, 2> goal_point = mid_two_point(theta_point_1 , theta_point_2);
        
        //RCLCPP_INFO(get_logger(),"x:%f , y:%f ",(theta_point_1[0]-theta_point_2[0]),(theta_point_1[1]-theta_point_2[1]));
        double x_modify = theta_point_1[0]-theta_point_2[0] ;
        double theta = atan2((theta_point_1[1]-theta_point_2[1]),x_modify);
        RCLCPP_INFO(get_logger(),"x:%f , y:%f , theta:%f",goal_point[0],goal_point[1],theta);

        // populate dock_frame
        populateTF(goal_point[0],goal_point[1],theta );
    }
}

int main(int argc, char** argv){

    //std::shared_ptr<automatic_parking::autodock_pattern> autodock_pattern_node;
    rclcpp::init(argc, argv);
    auto autodock_pattern_node = std::make_shared<automatic_parking::autodock_pattern>();
    rclcpp::WallRate rate(20.0);

    while (rclcpp::ok()){
        rclcpp::spin_some(autodock_pattern_node);
        rate.sleep();
    }

    return 0;
}

