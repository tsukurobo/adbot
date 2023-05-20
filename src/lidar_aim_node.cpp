#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#define ANGLE_MIN 500 //0~1079, 1step 0.25°, center 539
#define ANGLE_MAX 580
#define MAX_DISTANCE 10 

using namespace std;

std_msgs::Float64 error_angle;
ros::Publisher pub_angle;
ros::Publisher cmd_angle;

bool bool_lidar = true;
int point;

double degToRad(double deg)
{
    return deg * 3.1415926535 / 180;
}

double distance_angle_list[2][11] = {{3.758, 1.97, 3.758, 4.083, 4.083, 5.17, 4.355, 4.355, 8.961, 8.37, 8.961}, {degToRad(-45), degToRad(0), degToRad(45), degToRad(-30), degToRad(30), degToRad(0), degToRad(-15), degToRad(15), degToRad(-20), degToRad(0), degToRad(20)}};
double right_angle;
double distance_to_pole;
double around_pole = 0.3;
double angle_diff;
bool bool_pole = false;

void scan_callback(sensor_msgs::LaserScan msg)
{
    // ANGLE_MIN〜ANGLE_MAXで最も近い障害物を検出
    float detect_range = MAX_DISTANCE;
    for (int i = ANGLE_MIN; i < ANGLE_MAX; i++) {
        if (msg.ranges[i] < distance_to_pole + around_pole && msg.ranges[i] > distance_to_pole - around_pole) {
            if (msg.ranges[i] < detect_range) {
                detect_range = msg.ranges[i];
                point = i;
            }
        }
    }
    error_angle.data = degToRad((point - 539) * 0.25);
}

void lidar_callback(std_msgs::Bool cmd_toggle_Lidar) {
    bool_lidar = cmd_toggle_Lidar.data;
}

void pole_callback(std_msgs::Int16 pole_type) {
    distance_to_pole = distance_angle_list[0][pole_type.data];
    right_angle = distance_angle_list[1][pole_type.data];
}

void angle_callback(std_msgs::Float64 angle) {
    angle_diff = abs(angle.data - right_angle);
}

// 自動モード
void cmd_angle_callback(std_msgs::Float64 cmd_angle) {
    bool_pole = true;
}

// 手動モード
void cmd_angle_ad_callback(std_msgs::Float64 cmd_angle_ad) {
    bool_pole = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_aim_node");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");
    ros::Subscriber scan_sub = n.subscribe("scan", 1000, scan_callback);
    ros::Subscriber lidar_sub = n.subscribe("cmd_toggle_Lidar", 1000, lidar_callback);
    ros::Subscriber pole_type = n.subscribe("cmd_aiming_pole", 1000, pole_callback);
    ros::Subscriber now_angle_sub = n.subscribe("angle", 1000, angle_callback);
    ros::Subscriber angle_sub = n.subscribe("cmd_angle", 1000, cmd_angle_callback);
    ros::Subscriber cmd_angle_adjust = n.subscribe("cmd_angle_adjust", 1000, cmd_angle_ad_callback);

    pub_angle = n.advertise<std_msgs::Float64>("error_angle", 1000);
    cmd_angle = n.advertise<std_msgs::Float64>("cmd_angle", 1000);

    sleep(5);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        if (bool_lidar) {
            if (bool_pole) {
                if (angle_diff < 0.5) {
                    pub_angle.publish(error_angle);
                    cmd_angle.publish(error_angle);
                } else {
                    pub_angle.publish(error_angle);
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
