#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

ros::Publisher cmdAnglePub;
ros::Publisher cmdShootingDutyPub;
ros::Publisher cmdShootingVelocityPub;

ros::Subscriber cmdAimingPoleSub;
ros::Subscriber cmdReceiveSub;

// m単位
#define TYPE1_X 1.8
#define TYPE1_Y 3.2
#define TYPE2_X 3.7
#define TYPE2_Y 1.3
#define BACK_TYPE1_X 8.2
#define BACK_TYPE1_Y 3.2
#define BACK_TYPE2_X 6.3
#define BACK_TYPE2_Y 1.3
#define TIPE12_r 0.0505

constexpr double degToRad(int deg)
{
    return deg * 3.1415926535 / 180;
}
int poleTargetDuties[11] = {};
double poleTargetVelocities[11] = {};
double poleTargetAngles[] = {degToRad(-50), degToRad(0), degToRad(50), degToRad(-18), degToRad(18), degToRad(0), degToRad(-11), degToRad(11), degToRad(-20), degToRad(0), degToRad(20)};



void aimingPoleCb(const std_msgs::Int16 &polemsg)
{
    int aimingPole = polemsg.data;
    std_msgs::Float64 angle_pub;
    angle_pub.data = poleTargetAngles[aimingPole];
    std_msgs::Int16 duty_pub;
    duty_pub.data = poleTargetDuties[aimingPole];
    std_msgs::Float64 velocity_pub;
    velocity_pub.data = poleTargetVelocities[aimingPole];
    cmdAnglePub.publish(angle_pub);
    cmdShootingDutyPub.publish(duty_pub);
    cmdShootingVelocityPub.publish(velocity_pub);
}

void receiveCb(const std_msgs::Bool &receiveMsg)
{
    std_msgs::Int16 duty_pub;
    duty_pub.data = 0;
    std_msgs::Float64 angle_pub;
    angle_pub.data = degToRad(receiveMsg.data ? -90 : 90);
    cmdAnglePub.publish(angle_pub);
    cmdShootingDutyPub.publish(duty_pub);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_sub");
    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");
    double lidar_position;
    pnh.getParam("lidar_pos", lidar_position);
    poleTargetAngles[0]=-atan(TYPE1_Y / (lidar_position + TYPE1_X));
    poleTargetAngles[4]=0;
    poleTargetAngles[10]=atan(TYPE1_Y / (lidar_position + TYPE1_X));
    poleTargetAngles[2]=-atan(TYPE2_Y / (lidar_position + TYPE2_X));
    poleTargetAngles[8]=atan(TYPE2_Y / (lidar_position + TYPE2_X));
    poleTargetAngles[5]=0;
    poleTargetAngles[3]=-atan(BACK_TYPE2_Y / (lidar_position + BACK_TYPE2_X));
    poleTargetAngles[7]=atan(BACK_TYPE2_Y / (lidar_position + BACK_TYPE2_X));
    poleTargetAngles[1]=-atan(BACK_TYPE1_Y / (lidar_position + BACK_TYPE1_X));
    poleTargetAngles[6]=0;
    poleTargetAngles[9]=atan(BACK_TYPE1_Y / (lidar_position + BACK_TYPE1_X));


    cmdAnglePub = nh.advertise<std_msgs::Float64>("cmd_angle", 10);
    cmdShootingDutyPub = nh.advertise<std_msgs::Int16>("cmd_shooting_duty", 10);
    cmdShootingVelocityPub = nh.advertise<std_msgs::Float64>("cmd_shooting_velocity", 10);

    cmdAimingPoleSub = nh.subscribe("cmd_aiming_pole", 10, aimingPoleCb);
    cmdReceiveSub = nh.subscribe("cmd_receive", 10, receiveCb);

    XmlRpc::XmlRpcValue pole_duty;
    XmlRpc::XmlRpcValue pole_velocity;
    nh.getParam("/pole/duty", pole_duty);
    nh.getParam("/pole/velocity", pole_velocity);

    
    /*To ensure the reading will happen if the data is provided in right format*/
    if (pole_duty.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        for (int i = 0; i < pole_duty.size(); i++)
        {
            poleTargetDuties[i] = pole_duty[i];
        }
    }
    if (pole_velocity.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        for (int i = 0; i < pole_velocity.size(); i++)
        {
            poleTargetVelocities[i] = pole_velocity[i];
            ROS_INFO("pole vel %lf", poleTargetVelocities[i]);
        }
    }

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}