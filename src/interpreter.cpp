#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

ros::Publisher cmdToggleShootPub;
ros::Publisher cmdAnglePub;
ros::Publisher cmdAngleAdjustPub;
ros::Publisher cmdToggleBeltPub;
ros::Publisher cmdShootingDutyPub;
ros::Publisher cmdEmergencyStopPub;
ros::Subscriber sub;


int main(int argc, char **argv ){
    ros::init(argc, argv, "interpreter");
    ros::NodeHandle nh;
    cmdToggleShootPub = nh.advertise<std_msgs::Bool>("cmd_toggle_shoot", 10);

    // sub = nh.subscribe("joy", 10, joyCallBack)
    return 0;
}