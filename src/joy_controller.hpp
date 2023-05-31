#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

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

struct Command
{
public:
    ros::Publisher Publisher;
    bool sent = false;
    Command(ros::Publisher Publisher);
    Command() {}
    ~Command() {}
};

Command::Command(ros::Publisher Publisher) : Publisher(Publisher)
{
}

struct BoolCommand : public Command
{
public:
    bool current = false;
    void publish();
    BoolCommand(ros::Publisher Publisher);
    BoolCommand() {}
    ~BoolCommand() {}
};

BoolCommand::BoolCommand(ros::Publisher Publisher) : Command(Publisher)
{
}

void BoolCommand::publish()
{
    std_msgs::Bool pub;
    pub.data = current;
    Publisher.publish(pub);
    sent = true;
}

struct Int16Command : public Command
{
public:
    int16_t current = 0;
    void publish();
    Int16Command(ros::Publisher Publisher);
    Int16Command() {}
    ~Int16Command() {}
};

Int16Command::Int16Command(ros::Publisher Publisher) : Command(Publisher)
{
}

void Int16Command::publish()
{
    std_msgs::Int16 pub;
    pub.data = current;
    Publisher.publish(pub);
    sent = true;
}

struct Float64Command : public Command
{
public:
    double current = 0;
    void publish();
    Float64Command(ros::Publisher Publisher);
    Float64Command() {}
    ~Float64Command() {}
};

Float64Command::Float64Command(ros::Publisher Publisher) : Command(Publisher)
{
}

void Float64Command::publish()
{
    std_msgs::Float64 pub;
    pub.data = current;
    Publisher.publish(pub);
    sent = true;
}

float getJoyValue(const sensor_msgs::Joy &joymsg, XBOX_AXES axis)
{
    return joymsg.axes[static_cast<int>(axis)];
}

int getJoyValue(const sensor_msgs::Joy &joymsg, XBOX_BUTTONS button)
{
    return joymsg.buttons[static_cast<int>(button)];
}

class JoyController
{
private:
    BoolCommand toggleShoot;
    BoolCommand toggleBelt;
    BoolCommand toggleLidar;
    BoolCommand emergencyStop;
    Int16Command aimingPole;
    BoolCommand receive;
    Int16Command shootingDuty;
    Float64Command shootingVelocity;
    Command angleAdjust;

    ros::Subscriber joySub;
    ros::Subscriber cmdAimingPoleSub;
    ros::Subscriber cmdShootingDutySub;
    ros::Subscriber cmdShootingVelocitySub;
    ros::Subscriber cmdToggleBeltSub;
    ros::Subscriber cmdToggleLidarSub;
    ros::Subscriber cmdEmergencyStopSub;

    sensor_msgs::Joy joymsg;
    sensor_msgs::Joy lastJoymsg;

    bool executed = true;

public:
    void joyCb(const sensor_msgs::Joy &joymsg);
    void cmdAimingPoleCb(const std_msgs::Int16 &msg);
    void cmdShootingDutyCb(const std_msgs::Int16 &msg);
    void cmdShootingVelocityCb(const std_msgs::Float64 &msg);
    void cmdToggleBeltCb(const std_msgs::Bool &msg);
    void cmdToggleLidarCb(const std_msgs::Bool &msg);
    void cmdEmergencyStopCb(const std_msgs::Bool &msg);

    JoyController(ros::NodeHandle &nh);
    void update();
    // void init(ros::NodeHandle &nh);
};

JoyController::JoyController(ros::NodeHandle &nh)
{
    toggleShoot = BoolCommand(nh.advertise<std_msgs::Bool>("cmd_toggle_shoot", 10));
    toggleBelt = BoolCommand(nh.advertise<std_msgs::Bool>("cmd_toggle_belt", 10));
    toggleLidar = BoolCommand(nh.advertise<std_msgs::Bool>("cmd_toggle_lidar", 10));
    aimingPole = Int16Command(nh.advertise<std_msgs::Int16>("cmd_aiming_pole", 10));
    receive = BoolCommand(nh.advertise<std_msgs::Bool>("cmd_receive", 10));
    shootingDuty = Int16Command(nh.advertise<std_msgs::Int16>("cmd_shooting_duty", 10));
    shootingVelocity = Float64Command(nh.advertise<std_msgs::Float64>("cmd_shooting_velocity", 10));
    angleAdjust = Command(nh.advertise<std_msgs::Float64>("cmd_angle_adjust", 10));
    emergencyStop = BoolCommand(nh.advertise<std_msgs::Bool>("cmd_emergency_stop", 10));

    joySub = nh.subscribe("joy", 10, &JoyController::joyCb, this);
    cmdAimingPoleSub = nh.subscribe("cmd_aiming_pole", 10, &JoyController::cmdAimingPoleCb, this);
    cmdShootingDutySub = nh.subscribe("cmd_shooting_duty", 10, &JoyController::cmdShootingDutyCb, this);
    cmdShootingVelocitySub = nh.subscribe("cmd_shooting_velocity", 10, &JoyController::cmdShootingVelocityCb, this);
    cmdToggleBeltSub = nh.subscribe("cmd_toggle_belt", 10, &JoyController::cmdToggleBeltCb, this);
    cmdToggleLidarSub = nh.subscribe("cmd_toggle_lidar", 10, &JoyController::cmdToggleLidarCb, this);
    cmdEmergencyStopSub = nh.subscribe("cmd_emergency_stop", 10, &JoyController::cmdEmergencyStopCb, this);
}

// void JoyController::init(ros::NodeHandle &nh){
// }

inline void JoyController::joyCb(const sensor_msgs::Joy &joymsg)
{
    ROS_INFO("Recieved joy msg");
    this->lastJoymsg = this->joymsg;
    this->joymsg = joymsg;
    this->executed = false;
}

inline void JoyController::cmdAimingPoleCb(const std_msgs::Int16 &msg)
{
    aimingPole.current = msg.data;
}

inline void JoyController::cmdShootingDutyCb(const std_msgs::Int16 &msg)
{
    shootingDuty.current = msg.data;
}

inline void JoyController::cmdShootingVelocityCb(const std_msgs::Float64 &msg){
    shootingVelocity.current = msg.data;
}

inline void JoyController::cmdToggleBeltCb(const std_msgs::Bool &msg)
{
    toggleBelt.current = msg.data;
}

inline void JoyController::cmdToggleLidarCb(const std_msgs::Bool &msg)
{
    toggleLidar.current = msg.data;
}

inline void JoyController::cmdEmergencyStopCb(const std_msgs::Bool &msg)
{
    emergencyStop.current = msg.data;
}

void JoyController::update()
{
    if (executed)
    {
        return;
    }
    ROS_INFO("Interpreting joymsg");
    // リング分離の起動
    ROS_INFO(toggleShoot.sent ? "sent" : "not sent");
    if (toggleShoot.sent && !getJoyValue(joymsg, XBOX_BUTTONS::B))
    {
        toggleShoot.sent = false;
    }
    else if (!toggleShoot.sent && getJoyValue(joymsg, XBOX_BUTTONS::B) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
    {
        toggleShoot.current = true;
        toggleShoot.publish();
        ROS_INFO("sending shooter message");
    }

    // ベルト起動
    if (toggleBelt.sent && !getJoyValue(joymsg, XBOX_BUTTONS::START))
    {
        toggleBelt.sent = false;
    }
    else if (!toggleBelt.sent && getJoyValue(joymsg, XBOX_BUTTONS::START) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
    {
        toggleBelt.current = !toggleBelt.current;
        toggleBelt.publish();
    }

    // LiDar
    if (toggleLidar.sent && !getJoyValue(joymsg, XBOX_BUTTONS::Y))
    {
        toggleLidar.sent = false;
    }
    else if (!toggleLidar.sent && getJoyValue(joymsg, XBOX_BUTTONS::Y) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
    {
        toggleLidar.current = !toggleLidar.current;
        toggleLidar.publish();
    }

    // 緊急停止
    if (emergencyStop.sent && !getJoyValue(joymsg, XBOX_BUTTONS::BACK))
    {
        emergencyStop.sent = false;
    }
    if (!getJoyValue(joymsg, XBOX_BUTTONS::LB))
    {
        if (!emergencyStop.sent && getJoyValue(joymsg, XBOX_BUTTONS::BACK) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
        {
            emergencyStop.current = !emergencyStop.current;
            emergencyStop.publish();
        }
    }
    else
    {
        if (getJoyValue(joymsg, XBOX_BUTTONS::BACK) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
        {   
            shootingDuty.current=0;
            shootingDuty.publish();
            shootingVelocity.current = 0;
            shootingVelocity.publish();
            emergencyStop.sent = true;
        }
    }

    // 自動照準
    if (getJoyValue(joymsg, XBOX_BUTTONS::LB))
    {
        if (aimingPole.sent && ((getJoyValue(joymsg, XBOX_AXES::CROSS_VER) != getJoyValue(lastJoymsg, XBOX_AXES::CROSS_VER))))
        {
            aimingPole.sent = false;
        }
        if (!aimingPole.sent && getJoyValue(joymsg, XBOX_AXES::CROSS_VER) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
        {
            aimingPole.current += getJoyValue(joymsg, XBOX_AXES::CROSS_VER) == 1 ? -1 : 1;
            if (aimingPole.current < 0)
            {
                aimingPole.current = 0;
            }
            else if (aimingPole.current > 10)
            {
                aimingPole.current = 10;
            }
            else
            {
                aimingPole.publish();
            }
        }
    }
    else
    {
        aimingPole.sent = false;
        int poleToAim = -1;
        if (getJoyValue(joymsg, XBOX_AXES::CROSS_VER) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
        {
            poleToAim = getJoyValue(joymsg, XBOX_AXES::CROSS_VER) == 1 ? 2 : 8;
            // aimingPole.publish();
        }
        else if (getJoyValue(joymsg, XBOX_AXES::CROSS_HOR) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
        {
            poleToAim = getJoyValue(joymsg, XBOX_AXES::CROSS_HOR) == 1 ? 5 : 4;
            // aimingPole.publish();
        }
        if (poleToAim != aimingPole.current && poleToAim > -1)
        {
            aimingPole.current = poleToAim;
            aimingPole.publish();
        }
    }
    // 受け渡し
    if (getJoyValue(joymsg, XBOX_AXES::RT) < -0.7 && getJoyValue(joymsg, XBOX_BUTTONS::RB))
    {
        receive.current = true;
        receive.publish();
    }
    else if (getJoyValue(joymsg, XBOX_AXES::LT) < -0.7 && getJoyValue(joymsg, XBOX_BUTTONS::RB))
    {
        receive.current = false;
        receive.publish();
    }

    // 射出Duty
    if (getJoyValue(joymsg, XBOX_BUTTONS::X) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
    {
        shootingVelocity.current += 0.25;
        shootingVelocity.publish();
        shootingDuty.current += 1;
        shootingDuty.publish();
    }
    else if (getJoyValue(joymsg, XBOX_BUTTONS::A) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
    {
        shootingVelocity.current -= 0.25;
        shootingVelocity.publish();
        shootingDuty.current -= 1;
        shootingDuty.publish();
    }

    // 射出角度調整
    if (getJoyValue(joymsg, XBOX_AXES::JOY_LEFT_VER))
    {
        std_msgs::Float64 pub;
        pub.data = -getJoyValue(joymsg, XBOX_AXES::JOY_LEFT_VER);
        angleAdjust.Publisher.publish(pub);
        angleAdjust.sent = true;
    }
    else if (angleAdjust.sent)
    {
        std_msgs::Float64 pub;
        pub.data = 0;
        angleAdjust.Publisher.publish(pub);
        angleAdjust.sent = false;
    }
    executed = true;
}