#include <ros/ros.h>
#include <laser_line_extraction/LineSegmentList.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <boost/array.hpp>
#include <vector>
#include <cmath>
#include "pattern.h"

bool calAngle(double a, double b, double angle_ab, double detect_angle_tolerance){
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
    //printf("angle:%f , ab:%f \n",angle,angle_ab);
    if (fabs(angle_ab-angle)<=detect_angle_tolerance){
        return true;}
    else return false;
}

void populateMarkerMsg(double x, double y){
    points_msg.id = 1;
    points_msg.ns = "points";
    points_msg.type = visualization_msgs::Marker::POINTS;
    points_msg.action = visualization_msgs::Marker::ADD;
    points_msg.scale.x = 0.03;
    points_msg.scale.y = 0.03;
    points_msg.color.r = 1.0;
    points_msg.color.g = 0.0;
    points_msg.color.b = 1.0;
    points_msg.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
    points_msg.points.push_back(p);
    points_msg.header.frame_id = laser_frame_id;
    points_msg.header.stamp = ros::Time::now();
}

void populateTF(double x, double y, double theta, std::string name){
    // publish dock_frame
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x,y,0));
    tf::Quaternion q;
    q.setRPY(0,0,theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),laser_frame_id,name));
}

void updateVectors(){
    point_set = point_temp;
    printf("Upate vectors!\n");
}

bool check_center(auto &dock_vector , auto &vectors){
    for(int i=0; i<4; i++){
        for(int j(i+1); j<=4 ; j++){
            if (calAngle(vectors[i].angle,vectors[j].angle, 3.14-pattern_angle2, detect_angle_tolerance)){
                return true;
            }
        }
    }    
    return false;
}

void temp_vector(int i , int j , int angle_count, auto &vectors ){
    
    dock_vector.push_back(j);
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
        if (dist(temp_point_1,temp_point_2) <= 0.3 and check_center(dock_vector , vectors)){
            //printf("%s\n", check_center(dock_vector , vectors ) ? "true" : "false");
            updateVectors();
            check_angle = true;
        }
    }  
}



void patternCallback(const laser_line_extraction::LineSegmentList::ConstPtr& msg){
    std::vector<laser_line_extraction::LineSegment_<std::allocator<void>>> vectors = msg->line_segments;

    // Number of the line
    int lineNum = vectors.size();
    int angle_count = 0;
    bool check_vec_size = true;
    check_angle = false;

    // Check whether topic line_segments is publishing
    if (lineNum < 4){
        ROS_ERROR("There isn't enough line in the laser field!");
        check_vec_size = false;
    }
    if (check_vec_size){
        for(int i=0; i<lineNum; i++){
            for(int j(i+1); j<=lineNum ; j++){
                if (mid_dist(vectors[i] , vectors[j]) <= group_dist_tolerance) {
                    if (calAngle(vectors[i].angle,vectors[j].angle, pattern_angle1-3.14, detect_angle_tolerance)){
                        angle_count+=1;
                        temp_vector(i , j ,angle_count , vectors );

                        //printf("angle_count = %d\n",angle_count);
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

        double theta = atan2((theta_point_1[1]-theta_point_2[1]),(theta_point_1[0]-theta_point_2[0]));
        printf("x:%f , y:%f , theta:%f\n",goal_point[0],goal_point[1],theta);

        // populate dock origin marker publish to rviz
        populateMarkerMsg(goal_point[0],goal_point[1]);
        // populate dock_frame
        populateTF(goal_point[0],goal_point[1],theta,"dock_frame");
    }
}
   
int main(int argc, char** argv){
    ros::init(argc, argv, "pattern_node");
    //ROS_INFO("Start Pattern Recognition");
    ros::NodeHandle nh_;

    // Load Parameters
    nh_.param<double>("pattern_angle1",pattern_angle1, 3.9);
    nh_.param<double>("pattern_angle2",pattern_angle2, 1.57);
    nh_.param<double>("pattern_angle3",pattern_angle3, 3.9);
    nh_.param<double>("detect_angle_tolerance",detect_angle_tolerance, 0.25);
    nh_.param<double>("group_dist_tolerance",group_dist_tolerance, 0.20);
    nh_.param<std::string>("laser_frame_id",laser_frame_id, "laser_frame");
    #if 0
    nh_.setParam("pattern_angle1",pattern_angle1);
    nh_.setParam("pattern_angle2",pattern_angle2);
    nh_.setParam("pattern_angle3",pattern_angle3);
    nh_.setParam("detect_angle_tolerance",detect_angle_tolerance);
    nh_.setParam("group_dist_tolerance",group_dist_tolerance);
    nh_.setParam("pattern_angle1",pattern_angle1);
    nh_.setParam("laser_frame_id",laser_frame_id);
    #endif

    ros::Subscriber line_sub_ = nh_.subscribe("line_segments", 10, patternCallback);
    ros::Publisher marker_pub_ =nh_.advertise<visualization_msgs::Marker>("origin_markers",1);
    ros::Publisher lr_pub_ =nh_.advertise<std_msgs::Bool>("l_or_r",100);

    ros::Rate rate(20.0);

    while(ros::ok()){
        marker_pub_.publish(points_msg);
        lr_pub_.publish(lr_msg);

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
