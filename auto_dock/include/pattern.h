#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <auto_dock.h>
#include "laser_line_msgs/msg/line_segment_list.hpp"
#include "laser_line_msgs/msg/line_segment.hpp"
#include "laser_line_extraction/line_extraction.h"
#include <vector>

namespace automatic_parking {

    class autodock_pattern : public rclcpp::Node{
        public: 
            autodock_pattern(): Node("autodock_pattern"){
                this->declare_parameter<double>("pattern_angle1",3.84);
                this->get_parameter("pattern_angle1", pattern_angle1);
                this->declare_parameter<double>("pattern_angle2",1.57);
                this->get_parameter("pattern_angle2" ,pattern_angle2);
                this->declare_parameter<double>("pattern_angle3",3.84);
                this->get_parameter("pattern_angle3" , pattern_angle3);
                this->declare_parameter<double>("detect_angle_tolerance",0.23);
                this->get_parameter("detect_angle_tolerance" ,detect_angle_tolerance);
                this->declare_parameter<double>("group_dist_tolerance",0.20);
                this->get_parameter("group_dist_tolerance" , group_dist_tolerance);
                this->declare_parameter<std::string>("laser_frame_id","laser_frame");
                this->get_parameter("laser_frame_id", laser_frame_id);
                this->declare_parameter<std::string>("base_frame_id","base_link");
                this->get_parameter("base_frame_id" , base_frame_id);

                line_sub_ = this->create_subscription<laser_line_msgs::msg::LineSegmentList>("line_segments",rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
                    std::bind(&autodock_pattern::patternCallback, this, std::placeholders::_1));
                tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            }

        private:

            struct Point_set{
                boost::array<double, 2> vector_a;
                boost::array<double, 2> vector_b;
                boost::array<double, 2> vector_c;
                boost::array<double, 2> vector_d;
            } ;

            
            void patternCallback(const laser_line_msgs::msg::LineSegmentList::SharedPtr msg );
            void temp_vector(int& , int& ,int& , std::vector<laser_line_msgs::msg::LineSegment_<std::allocator<void>>>&);
            bool check_center(std::vector<laser_line_msgs::msg::LineSegment_<std::allocator<void>>>&);
            void updateVectors();
            void populateTF(double x, double y, double theta );
            bool calAngle(double a, double b, double angle_ab);

            rclcpp::Subscription<laser_line_msgs::msg::LineSegmentList>::SharedPtr line_sub_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            std::vector<int> dock_vector = {0 ,0 ,0 ,0};
            Point_set point_set,point_temp;
            boost::array<double, 2> temp_point_1 , temp_point_2;
            bool check_angle = false;
            std::string frame_name = "dock_frame";
            double pattern_angle1 , pattern_angle2 ,pattern_angle3 ,detect_angle_tolerance ,group_dist_tolerance;
            std::string laser_frame_id ,base_frame_id;
            boost::array<double, 2> center_point;
    };
}
