#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

ros::Publisher cmdAnglePub;
ros::Publisher cmdShootingDutyPub;

ros::Subscriber cmdAimingPoleSub;

constexpr double degToRad(int deg)
{
    return deg * 3.1415926535 / 180;
}
int poleTargetDuties[] = {470, 370, 470, 530, 530, 640, 620, 620, 620, 620, 620 };
double poleTargetAngles[] = {degToRad(-45), degToRad(0), degToRad(45), degToRad(-30), degToRad(30), degToRad(0), degToRad(-15), degToRad(15), degToRad(-20), degToRad(0), degToRad(20) };

void aimingPoleCb(const std_msgs::Int16 &polemsg)
{
    int aimingPole = polemsg.data;
    std_msgs::Float64 angle_pub;
    angle_pub.data = poleTargetAngles[aimingPole];
    std_msgs::Int16 duty_pub;
    duty_pub.data = poleTargetDuties[aimingPole];
    cmdAnglePub.publish(angle_pub);
    cmdShootingDutyPub.publish(duty_pub);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_sub");
    ros::NodeHandle nh;
    cmdAnglePub = nh.advertise<std_msgs::Float64>("cmd_angle", 10);
    cmdShootingDutyPub = nh.advertise<std_msgs::Int16>("cmd_shooting_duty", 10);

    cmdAimingPoleSub = nh.subscribe("cmd_aiming_pole", 10, aimingPoleCb);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}