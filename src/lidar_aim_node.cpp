#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#define ANGLE_MIN 500 // 0~1079, 1step 0.25°, center 539
#define ANGLE_MAX 580
#define MAX_DISTANCE 10

// m単位
constexpr double TYPE1_X = 1.8 - 0.05;
constexpr double TYPE1_Y = 3.2;
constexpr double TYPE2_X = 3.7 - 0.05;
constexpr double TYPE2_Y = 1.3;
constexpr double BACK_TYPE1_X = 8.2 - 0.05;
constexpr double BACK_TYPE1_Y = 3.2;
constexpr double BACK_TYPE2_X = 6.3 - 0.05;
constexpr double BACK_TYPE2_Y = 1.3;
constexpr double TIPE12_r = 0.0505;
// #define TIPE3_r 0.0699
// 直径
// 1:101mm 2:101
// 3:139.8

using namespace std;

std_msgs::Float32 error_angle;
ros::Publisher pub_angle;
ros::Publisher cmd_angle;

bool bool_lidar = true;
int point;
double lidar_position;

double degToRad(double deg)
{
    return deg * 3.1415926535 / 180;
}

double distance_angle_list[2][11]; // = {{3.758, 1.97, 3.758, 4.083, 4.083, 1.97, 4.355, 4.355, 8.961, 8.37, 8.961}, {degToRad(-45), degToRad(0), degToRad(45), degToRad(-30), degToRad(30), degToRad(0), degToRad(-15), degToRad(15), degToRad(-20), degToRad(0), degToRad(20)}};
double right_angle;
double distance_to_pole;
double around_pole = 0.1;
double angle_diff;
bool bool_pole = false;
bool detect;

void scan_callback(sensor_msgs::LaserScan msg)
{
    // ANGLE_MIN〜ANGLE_MAXで最も近い障害物を検出
    float detect_range = MAX_DISTANCE;
    detect = false;
    for (int i = ANGLE_MIN; i < ANGLE_MAX; i++)
    {
        if (msg.ranges[i] < distance_to_pole + around_pole && msg.ranges[i] > distance_to_pole - around_pole)
        {
            if (msg.ranges[i] < detect_range)
            {
                detect_range = msg.ranges[i];
                point = i;
                detect = true;
            }
        }
    }
    error_angle.data = -degToRad((point - 539) * 0.25);
}

void lidar_callback(std_msgs::Bool cmd_toggle_Lidar)
{
    bool_lidar = cmd_toggle_Lidar.data;
}

void pole_callback(std_msgs::Int16 pole_type)
{
    distance_to_pole = distance_angle_list[0][pole_type.data];
    right_angle = distance_angle_list[1][pole_type.data];
}

void angle_callback(std_msgs::Float32 angle)
{
    angle_diff = abs(angle.data - right_angle);
}

// 自動モード
void cmd_angle_callback(std_msgs::Float32 cmd_angle)
{
    bool_pole = true;
}

// 手動モード
void cmd_angle_ad_callback(std_msgs::Float32 cmd_angle_ad)
{
    bool_pole = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_aim_node");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");
    pnh.getParam("lidar_pos", lidar_position);
    // 距離計算
    distance_angle_list[0][0] = sqrt(pow(lidar_position + TYPE1_X, 2) + pow(TYPE1_Y, 2)) - TIPE12_r;
    distance_angle_list[0][4] = TYPE1_X - TIPE12_r;
    distance_angle_list[0][10] = distance_angle_list[0][0];
    distance_angle_list[0][2] = sqrt(pow(lidar_position + TYPE2_X, 2) + pow(TYPE2_Y, 2)) - TIPE12_r;
    distance_angle_list[0][8] = distance_angle_list[0][3];
    distance_angle_list[0][5] = TYPE1_X - TIPE12_r;
    distance_angle_list[0][3] = sqrt(pow(lidar_position + BACK_TYPE2_X, 2) + pow(BACK_TYPE2_Y, 2)) - TIPE12_r;
    distance_angle_list[0][7] = distance_angle_list[0][6];
    distance_angle_list[0][1] = sqrt(pow(lidar_position + BACK_TYPE1_X, 2) + pow(BACK_TYPE1_Y, 2)) - TIPE12_r;
    distance_angle_list[0][6] = TYPE1_X - TIPE12_r;
    distance_angle_list[0][9] = distance_angle_list[0][8];
    // 角度計算
    distance_angle_list[1][0] = -atan(TYPE1_Y / (lidar_position + TYPE1_X));
    distance_angle_list[1][4] = 0;
    distance_angle_list[1][10] = atan(TYPE1_Y / (lidar_position + TYPE1_X));
    distance_angle_list[1][2] = -atan(TYPE2_Y / (lidar_position + TYPE2_X));
    distance_angle_list[1][8] = atan(TYPE2_Y / (lidar_position + TYPE2_X));
    distance_angle_list[1][5] = 0;
    distance_angle_list[1][3] = -atan(BACK_TYPE2_Y / (lidar_position + BACK_TYPE2_X));
    distance_angle_list[1][7] = atan(BACK_TYPE2_Y / (lidar_position + BACK_TYPE2_X));
    distance_angle_list[1][1] = -atan(BACK_TYPE1_Y / (lidar_position + BACK_TYPE1_X));
    distance_angle_list[1][6] = 0;
    distance_angle_list[1][9] = atan(BACK_TYPE1_Y / (lidar_position + BACK_TYPE1_X));

    ros::Subscriber scan_sub = n.subscribe("scan", 1000, scan_callback);
    ros::Subscriber lidar_sub = n.subscribe("cmd_toggle_Lidar", 1000, lidar_callback);
    ros::Subscriber pole_type = n.subscribe("cmd_aiming_pole", 1000, pole_callback);
    ros::Subscriber now_angle_sub = n.subscribe("angle", 1000, angle_callback);
    ros::Subscriber angle_sub = n.subscribe("cmd_angle", 1000, cmd_angle_callback);
    ros::Subscriber cmd_angle_adjust = n.subscribe("cmd_angle_adjust", 1000, cmd_angle_ad_callback);

    pub_angle = n.advertise<std_msgs::Float32>("error_angle", 1000);
    cmd_angle = n.advertise<std_msgs::Float32>("cmd_angle", 1000);

    sleep(5);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (bool_lidar)
        {
            if (bool_pole && detect)
            {
                if (angle_diff < 0.3)
                {
                    pub_angle.publish(error_angle);
                    cmd_angle.publish(error_angle);
                }
                else
                {
                    pub_angle.publish(error_angle);
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
