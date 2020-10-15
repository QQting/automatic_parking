#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>
#include <auto_dock.h>
#include "laser_line_extraction/line_extraction.h"
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#define _USE_MATH_DEFINES

namespace automatic_parking {
    class autodock_controller : public rclcpp::Node{
        public: 
            autodock_controller(): Node("autodock_controller"){
                this->declare_parameter<double>("min_v",0.1);
                this->get_parameter("min_v", min_v);
                this->declare_parameter<double>("min_w",0.1);
                this->get_parameter("min_w" ,min_w);
                this->declare_parameter<double>("max_v",0.3);
                this->get_parameter("max_v" , max_v);
                this->declare_parameter<double>("max_w",0.3);
                this->get_parameter("max_w" , max_w);
                this->declare_parameter<double>("dist_to_dock",0.30);
                this->get_parameter("dist_to_dock" ,dist_to_dock);
                this->declare_parameter<double>("dist_to_center",0.03);
                this->get_parameter("dist_to_center" , dist_to_center);
                this->declare_parameter<double>("threshold_v",0.3);
                this->get_parameter("threshold_v" , threshold_v);
                this->declare_parameter<double>("threshold_w",0.4);
                this->get_parameter("threshold_w" , threshold_w);
                this->declare_parameter<std::string>("base_frame_id","base_link");
                this->get_parameter("base_frame_id" , base_frame_id);

                vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
                buffer_dock = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                buffer_odom = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                tf_listener_dock = std::make_unique<tf2_ros::TransformListener>(*buffer_dock);
                tf_listener_odom = std::make_unique<tf2_ros::TransformListener>(*buffer_odom);

            }
            void run();
        private:
            void receive_tf();
            void setVel(float x, float y, float yaw, boost::array<float,2> robot_point);
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
            std::unique_ptr<tf2_ros::Buffer> buffer_dock;
            std::unique_ptr<tf2_ros::Buffer> buffer_odom;
            std::unique_ptr<tf2_ros::TransformListener> tf_listener_dock ,tf_listener_odom ;
            double min_v, min_w, max_v, max_w , dist_to_dock, dist_to_center,threshold_v, threshold_w;
            std::string base_frame_id;
            int step = 0;
            int step1_count = 0 , step2_count = 0;
            int split_num = 2;
            float tune_distense = 0.05;
            float tune_threshold = 0.5;
            float  x_origin;
            boost::array<float, 2> odom , robot_point_temp;
            geometry_msgs::msg::TransformStamped tf_dock , tf_odom;

    };
}
