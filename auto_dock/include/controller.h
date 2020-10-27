#include "auto_dock.h"
// ROS
ros::Publisher vel_pub;

//parameter setting
std::string base_frame_id;
double min_v, min_w, max_v, max_w;
double dist_to_dock, dist_to_center;
double threshold_v, threshold_w;
float  x_origin;
int step1_count = 0 , step2_count = 0;
bool reach_target_pos = false;
bool reach_target_ang = false;
bool target_alignment = false;
bool left = false;
bool left_check = false;
bool pattern = false;
int step = 0;
int split_num = 2;
boost::array<float, 2> odom , robot_point_temp;
float tune_distense = 0.05;
float tune_threshold = 0.6;