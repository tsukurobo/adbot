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
    ~Command();
};

Command::Command(ros::Publisher Publisher) : Publisher(Publisher)
{
}

Command::~Command()
{
}

struct BoolCommand : public Command
{
public:
    bool current = false;
    void publish();
    BoolCommand(ros::Publisher Publisher);
    ~BoolCommand();
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
    ~Int16Command();
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
    BoolCommand emergencyStop;
    Int16Command aimingPole;
    BoolCommand receive;
    Int16Command shootingDuty;
    Command angleAdjust;

    ros::Subscriber joySub;
    ros::Subscriber cmdAimingPoleSub;
    ros::Subscriber cmdShootingDutySub;
    ros::Subscriber cmdToggleBeltSub;
    ros::Subscriber cmdEmergencyStopSub;

    sensor_msgs::Joy &joymsg;
    sensor_msgs::Joy &lastJoymsg;

    void joyCb(const sensor_msgs::Joy &joymsg);
    void cmdAimingPoleCb(const std_msgs::Int16 &msg);
    void cmdShootingDutyCb(const std_msgs::Int16 &msg);
    void cmdToggleBeltCb(const std_msgs::Bool &msg);
    void cmdEmergencyStopCb(const std_msgs::Bool &msg);


public:
    JoyController(ros::NodeHandle &nh);
    void update();
};

JoyController::JoyController(ros::NodeHandle &nh)
{
    toggleShoot = BoolCommand(nh.advertise<std_msgs::Bool>("cmd_toggle_shoot", 10));
    toggleBelt = BoolCommand(nh.advertise<std_msgs::Bool>("cmd_toggle_belt", 10));
    aimingPole = Int16Command(nh.advertise<std_msgs::Int16>("cmd_aiming_pole", 10));
    receive = BoolCommand(nh.advertise<std_msgs::Bool>("cmd_receive", 10));
    shootingDuty = Int16Command(nh.advertise<std_msgs::Int16>("cmd_shooting_duty", 10));
    angleAdjust = Command(nh.advertise<std_msgs::Float64>("cmd_angle_adjust", 10));
    emergencyStop = BoolCommand(nh.advertise<std_msgs::Bool>("cmd_emergency_stop", 10));

    joySub = nh.subscribe("joy", 10, joyCb);
    cmdAimingPoleSub = nh.subscribe("cmd_aiming_pole", 10, cmdAimingPoleCb);
    cmdShootingDutySub = nh.subscribe("cmd_shooting_duty", 10, cmdShootingDutyCb);
    cmdToggleBeltSub = nh.subscribe("cmd_toggle_belt", 10, cmdToggleBeltCb);
    cmdEmergencyStopSub = nh.subscribe("cmd_emergency_stop", 10, cmdEmergencyStopCb);
}

inline void JoyController::joyCb(const sensor_msgs::Joy &joymsg)
{
    this->joymsg = joymsg;
}

inline void JoyController::cmdAimingPoleCb(const std_msgs::Int16 &msg)
{
    aimingPole.current = msg.data;
}

inline void JoyController::cmdShootingDutyCb(const std_msgs::Int16 &msg)
{
    shootingDuty.current = msg.data;
}

inline void JoyController::cmdToggleBeltCb(const std_msgs::Bool &msg)
{
    toggleBelt.current = msg.data;
}

inline void JoyController::cmdEmergencyStopCb(const std_msgs::Bool &msg)
{
    emergencyStop.current = msg.data;
}

void JoyController::update()
{
    // リング分離の起動
    if (toggleShoot.sent && !getJoyValue(joymsg, XBOX_BUTTONS::B))
    {
        toggleShoot.sent = false;
    }
    else if (!toggleShoot.sent && getJoyValue(joymsg, XBOX_BUTTONS::B) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
    {
        toggleShoot.current = true;
        toggleShoot.publish();
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

    // 緊急停止
    if (emergencyStop.sent && !getJoyValue(joymsg, XBOX_BUTTONS::BACK))
    {
        emergencyStop.sent = false;
    }
    else if (!emergencyStop.sent && getJoyValue(joymsg, XBOX_BUTTONS::BACK) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
    {
        emergencyStop.current = !toggleBelt.current;
        emergencyStop.publish();
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
        if (aimingPole.sent && ((getJoyValue(joymsg, XBOX_AXES::CROSS_VER) != getJoyValue(lastJoymsg, XBOX_AXES::CROSS_VER)) && (getJoyValue(joymsg, XBOX_AXES::CROSS_HOR) != getJoyValue(lastJoymsg, XBOX_AXES::CROSS_HOR))))
        {
            aimingPole.sent = false;
        }
        else if (!aimingPole.sent && getJoyValue(joymsg, XBOX_AXES::CROSS_VER) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
        {
            aimingPole.current = getJoyValue(joymsg, XBOX_AXES::CROSS_VER) == 1 ? 3 : 4;
            aimingPole.publish();
        }
        else if (!aimingPole.sent && getJoyValue(joymsg, XBOX_AXES::CROSS_HOR) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
        {
            std_msgs::Int16 pub;
            pub.data = getJoyValue(joymsg, XBOX_AXES::CROSS_HOR) == 1 ? 5 : 1;
            aimingPole.Publisher.publish(pub);
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
        shootingDuty.current += 1;
        shootingDuty.publish();
    }
    else if (getJoyValue(joymsg, XBOX_BUTTONS::A) && getJoyValue(joymsg, XBOX_BUTTONS::RB))
    {
        shootingDuty.current -= 2;
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
    lastJoymsg = joymsg;
}