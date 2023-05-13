#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"


namespace XBOX_CONTROLLER{
    constexpr uint8_t CROSS_HOR = 5;
    constexpr uint8_t CROSS_VER = 6;
};


ros::Publisher cmdToggleShootPub;
ros::Publisher cmdAnglePub;
ros::Publisher cmdAngleAdjustPub;
ros::Publisher cmdToggleBeltPub;
ros::Publisher cmdShootingDutyPub;
ros::Publisher cmdEmergencyStopPub;
ros::Subscriber sub;

enum class XBOX_AXES{
    JOY_LEFT_VER,
    JOY_LEFT_HOR,
    LT,
    JOY_RIGT_VER,
    JOY_RIGHT_HOR,
    RT,
    CROSS_VER,
    CROSS_HOR
};

enum class XBOX_BUTTONS{
    A,
    B,
    X,
    Y,
    LB,
    RB,
    BACK,
    START
};

float getJoyValue(const sensor_msgs::Joy &joymsg, XBOX_AXES axis){
    return joymsg.axes[static_cast<int>(axis)];
}

int getJoyValue(const sensor_msgs::Joy &joymsg, XBOX_BUTTONS button){
    return joymsg.buttons[static_cast<int>(button)];
}

void joyCb(const sensor_msgs::Joy &joymsg){
    getJoyValue(joymsg, XBOX_AXES::CROSS_VER);
}

int main(int argc, char **argv ){
    ros::init(argc, argv, "interpreter");
    ros::NodeHandle nh;
    cmdToggleShootPub = nh.advertise<std_msgs::Bool>("cmd_toggle_shoot", 10);

    sub = nh.subscribe("joy", 10, joyCb);
    return 0;
}