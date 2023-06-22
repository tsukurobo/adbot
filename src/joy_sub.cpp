#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "joy_controller.hpp"

ros::Publisher cmdToggleShootPub;
ros::Publisher cmdAimingPolePub;
ros::Publisher cmdReceivePub;
// ros::Publisher cmdAnglePub;
ros::Publisher cmdShootingDutyPub;
ros::Publisher cmdAngleAdjustPub;
ros::Publisher cmdToggleBeltPub;
ros::Publisher cmdEmergencyStopPub;
ros::Subscriber joySub;
ros::Subscriber cmdAimingPoleSub;
ros::Subscriber cmdShootingDutySub;
ros::Subscriber cmdToggleBeltSub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_sub");
    ros::NodeHandle nh;
    JoyController joyController(nh);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        joyController.update();
        loop_rate.sleep();
    }

    return 0;
}