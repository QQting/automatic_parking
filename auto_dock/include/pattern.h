#include "auto_dock.h"

// ROS
visualization_msgs::Marker points_msg;

// Parameters
double pattern_angle1, pattern_angle2, pattern_angle3;
double detect_angle_tolerance, group_dist_tolerance;
std::string laser_frame_id;

struct point_set{
    boost::array<double, 2> vector_a;
    boost::array<double, 2> vector_b;
    boost::array<double, 2> vector_c;
    boost::array<double, 2> vector_d;
} ;

struct point_set point_temp;
struct point_set point_set;
std::vector<int> dock_vector(4);
boost::array<double, 2> temp_point_1 , temp_point_2;
bool check_angle = false;

