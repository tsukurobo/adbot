#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

ros::Publisher cmdToggleShootPub;
ros::Publisher cmdAimingPolePub;
// ros::Publisher cmdAnglePub;
// ros::Publisher cmdShootingDutyPub;
ros::Publisher cmdAngleAdjustPub;
ros::Publisher cmdToggleBeltPub;
ros::Publisher cmdEmergencyStopPub;
ros::Subscriber joySub;
ros::Subscriber cmdAimingPoleSub;

int aimingPole = 6;

enum class XBOX_AXES
{
    JOY_LEFT_VER,
    JOY_LEFT_HOR,
    LT,
    JOY_RIGT_VER,
    JOY_RIGHT_HOR,
    RT,
    CROSS_VER,
    CROSS_HOR
};

enum class XBOX_BUTTONS
{
    A,
    B,
    X,
    Y,
    LB,
    RB,
    BACK,
    START
};

float getJoyValue(const sensor_msgs::Joy &joymsg, XBOX_AXES axis)
{
    return joymsg.axes[static_cast<int>(axis)];
}

int getJoyValue(const sensor_msgs::Joy &joymsg, XBOX_BUTTONS button)
{
    return joymsg.buttons[static_cast<int>(button)];
}

void joyCb(const sensor_msgs::Joy &joymsg)
{
    // getJoyValue(joymsg, XBOX_AXES::CROSS_VER);

    if (getJoyValue(joymsg, XBOX_BUTTONS::RB))
    {
        std_msgs::Bool pub;
        pub.data = true;
        cmdToggleShootPub.publish(pub);
    }
    else if (getJoyValue(joymsg, XBOX_BUTTONS::LB))
    {
        std_msgs::Bool pub;
        pub.data = true;
        cmdToggleBeltPub.publish(pub);
    }
    else if (getJoyValue(joymsg, XBOX_BUTTONS::BACK))
    {
        std_msgs::Bool pub;
        pub.data = true;
        cmdEmergencyStopPub.publish(pub);
    }
    else if (const int value = getJoyValue(joymsg, XBOX_AXES::CROSS_VER))
    {
        std_msgs::Int16 pub;
        int pole = aimingPole + (value == 1 ? -1 : 1);
        if (pole < 0)
            pole = 0;
        else if (pole > 10)
            pole = 10;
        pub.data = pole;
        cmdAimingPolePub.publish(pub);
    }
    // else if (const int value = getJoyValue(joymsg, XBOX_AXES::JOY_LEFT_VER))
    else
    {
        std_msgs::Float64 pub;
        pub.data = -getJoyValue(joymsg, XBOX_AXES::JOY_LEFT_VER);
        cmdAngleAdjustPub.publish(pub);
    }
}

void aimingPoleCb(const std_msgs::Int16 &polemsg)
{
    aimingPole = polemsg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interpreter");
    ros::NodeHandle nh;
    cmdToggleShootPub = nh.advertise<std_msgs::Bool>("cmd_toggle_shoot", 10);
    cmdAimingPolePub = nh.advertise<std_msgs::Int16>("cmd_aiming_pole", 10);
    cmdToggleBeltPub = nh.advertise<std_msgs::Bool>("cmd_toggle_belt", 10);
    cmdEmergencyStopPub = nh.advertise<std_msgs::Bool>("cmd_emergency_stop", 10);
    cmdAngleAdjustPub = nh.advertise<std_msgs::Float64>("cmd_angle_adjust", 10);

    joySub = nh.subscribe("joy", 10, joyCb);
    cmdAimingPoleSub = nh.subscribe("cmd_aiming_pole", 10, aimingPoleCb);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}