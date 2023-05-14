// ROS
const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

ros.on('connection', function () {
    console.log('Connected to websocket server.');
});

ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function () {
    console.log('Connection to websocket server closed.');
});

// Define topics
const cmdAngle = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_angle',
    messageType: 'std_msgs/Float64'
});


const cmdDuty = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_shooting_duty',
    messageType: 'std_msgs/Int16'
});

const cmdAimingPole = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_aiming_pole',
    messageType: 'std_msgs/Int16'
});

const cmdReceive = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_receive',
    messageType: 'std_msgs/Bool'
});

const cmdToggleShoot = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_toggle_shoot',
    messageType: 'std_msgs/Bool'
});

const cmdToggleBelt = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_toggle_belt',
    messageType: 'std_msgs/Bool'
});

const cmdToggleLidar = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_toggle_Lidar',
    messageType: 'std_msgs/Bool'
});

const cmdEmergencyStop = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_emergency_stop',
    messageType: 'std_msgs/Bool'
});

const currentAngle = new ROSLIB.Topic({
    ros: ros,
    name: '/angle',
    messageType: 'std_msgs/Float64'
});

const distanceToPole = new ROSLIB.Topic({
    ros: ros,
    name: '/distance_to_pole',
    messageType: 'std_msgs/Float64'
});

const errorAngle = new ROSLIB.Topic({
    ros: ros,
    name: '/error_angle',
    messageType: 'std_msgs/Float64'
});


// global variables and functions
var targetAngle = 0;
var targetDuty = 0;

function updateAngle(newAngle, ctx, toBePublished = true) {
    targetAngle = newAngle;
    ctx.save();
    ctx.font = '48px "Roboto Mono", sans-serif';
    ctx.clearRect(1550, 750, 420, 60);
    ctx.fillText(targetAngle, 1550, 800);
    if (toBePublished) {
        const angle = new ROSLIB.Message({
            data: targetAngle
        })
        cmdAngle.publish(angle);
    }
    ctx.restore();
}

function updateDuty(newDuty, ctx, toBePublished = true) {
    targetDuty = newDuty;
    ctx.save();
    ctx.font = '48px "Roboto Mono", sans-serif';
    ctx.clearRect(1250, 750, 220, 60);
    ctx.fillText(targetDuty, 1250, 800);
    if (toBePublished) {
        const duty = new ROSLIB.Message({
            data: targetDuty
        })
        cmdDuty.publish(duty);
    }
    ctx.restore();
}


function updateLidar(state, ctx) {
    const toggle = new ROSLIB.Message({
        data: state
    })
    cmdToggleLidar.publish(toggle);
}

function degToRad(deg) {
    return deg * Math.PI / 180;
}



// canvas

class Rectangle {
    constructor(x, y, w, h, str, color) {
        this.x = x;
        this.y = y;
        this.w = w;
        this.h = h;
        this.str = str;
        this.color = color;
    }

    draw(ctx, color = this.color, isSelected = false) {
        ctx.save();
        ctx.clearRect(this.x - this.w / 2 - 10, this.y - this.h / 2 - 10, this.w + 20, this.h + 20);
        if (isSelected) {
            ctx.fillStyle = "yellow";
            ctx.fillRect(this.x - this.w / 2 - 10, this.y - this.h / 2 - 10, this.w + 20, this.h + 20);
        }
        ctx.fillStyle = color;
        ctx.fillRect(this.x - this.w / 2, this.y - this.h / 2, this.w, this.h);
        ctx.fillStyle = "white";
        ctx.font = 'bold 48px "Roboto Mono", sans-serif';
        ctx.textBaseline = 'middle';
        ctx.textAlign = 'center';
        ctx.fillText(this.str, this.x, this.y);
        ctx.restore();
    }

    onSubscribe(ctx, isSelected = false) {
        console.log("subscribed " + isSelected);
        this.draw(ctx, this.color, isSelected);
    }

    onClick(ctx, timeout = 300) {
        this.draw(ctx, "red", true);
        setTimeout(() => {
            this.draw(ctx, this.color, false);
        }, timeout);
    }

    checkIfClicked(point) {
        return (this.x - this.w / 2 <= point.x && point.x <= this.x + this.w / 2) &&
            (this.y - this.h / 2 <= point.y && point.y <= this.y + this.h / 2);
    }
}

const PoleTypes = [];
PoleTypes.push(10, 10, 10, 30, 30, 70, 30, 30, 25, 25, 25);
const PoleCoordinates = [];
PoleCoordinates.push([300, 940], [550, 940], [800, 940], [425, 740], [675, 740], [550, 540], [425, 340], [675, 340], [300, 140], [550, 140], [800, 140]);
const PoleTargetAngles = [];
PoleTargetAngles.push(degToRad(-45), degToRad(0), degToRad(45), degToRad(-30), degToRad(30), degToRad(0), degToRad(-15), degToRad(15), degToRad(-20), degToRad(0), degToRad(20));//パラメータ：各ポールのプリセットangle
const PoleTargetDuties = [];
// PoleTargetDuties.push(470, 370, 470, 530, 530, 640, 620, 620, 620, 620, 620); //パラメータ：各ポールのプリセットduty。自陣Type1左, 中, 右, 自陣Type2左, 右, Type3, 敵陣Type2左, 右, 敵陣Type1左, 中, 右


class Pole extends Rectangle {
    constructor(x, y, no) {
        super(x, y, 125, 125, PoleTypes[no], "black");
        this.type = PoleTypes[no];
        this.no = no;
    }
    onUnselected(ctx) {
        this.draw(ctx, this.color, false);
    }
    onClick(ctx, timeout = 300) {
        console.info("Aiming at Type " + this.type + " pole.");
        const aimingPole = new ROSLIB.Message({
            data: this.no
        })
        cmdAimingPole.publish(aimingPole);
        targetAngle = PoleTargetAngles[this.no];
        updateAngle(targetAngle, ctx);
        targetDuty = PoleTargetDuties[this.no];
        updateDuty(targetDuty, ctx);
    }
}

class Shoot extends Rectangle {
    constructor(x, y) {
        super(x, y, 300, 100, 'Shoot', 'blue');
    }
    onSubscribe(ctx, timeout = 300) {
        this.draw(ctx, "red", true);
        setTimeout(() => {
            this.draw(ctx, this.color, false);
        }, timeout);
    }
    onClick() {
        console.info("Shooting");
        const toggle = new ROSLIB.Message({
            data: true
        })
        cmdToggleShoot.publish(toggle);
    }
}

class Receive extends Rectangle {
    constructor(x, y, direction) {
        super(x, y, 130, 100, '受取', 'blue');
        this.direction = direction;
    }

    onSubscribe(ctx, isSelected = false, poles = []) {
        super.onSubscribe(ctx, isSelected);
        if (isSelected) {
            poles.forEach(pole => {
                pole.onUnselected(ctx);
            });
        }
    }
    onUnselected(ctx) {
        this.draw(ctx, this.color, false);
    }
    onClick(ctx) {
        console.info("Receive in " + (this.direction ? "Right" : "Left"));
        // targetAngle = this.direction ? degToRad(-90) : degToRad(90);
        // updateAngle(targetAngle, ctx);
        const pub = new ROSLIB.Message({
            data: this.direction
        })
        cmdReceive.publish(pub);
    }
}

class Emergency extends Rectangle {
    constructor(x, y) {
        super(x, y, 300, 100, 'Emergency', 'gray');
    }

    onSubscribe(ctx, isSelected = false) {
        super.onSubscribe(ctx, isSelected);
    }

    onClick(ctx) {
        // super.onClick(ctx);
        console.info("Emergency stop");
        const toggle = new ROSLIB.Message({
            data: true
        })
        cmdEmergencyStop.publish(toggle);
    }
}

class StatusButton extends Rectangle {
    constructor(x, y, w, h, str, defaultColor, changedColor) {
        super(x, y, w, h, str, defaultColor);
        this.changedColor = changedColor;
        this.status = false;
    }

    onClick(ctx) {
        // super.onClick(ctx, 0);
        this.status = !this.status;
    }

    onSubscribe(ctx, isSelected = false) {
        // super.onSubscribe(ctx, isSelected);
        this.status = isSelected;
        this.draw(ctx);
    }

    draw(ctx) {
        super.draw(ctx, this.status ? this.changedColor : this.defaultColor);
    }
}

class Belt extends StatusButton {
    constructor(x, y) {
        super(x, y, 300, 100, 'Belt', 'blue', 'green');
    }

    onClick(ctx) {
        super.onClick(ctx);
        console.info("Belt " + (this.status ? "on" : "off"));
        const toggle = new ROSLIB.Message({
            data: this.status
        })
        cmdToggleBelt.publish(toggle);
    }

}

class Lidar extends StatusButton {
    constructor(x, y) {
        super(x, y, 300, 100, 'Lidar', 'blue', 'green');
    }

    onClick(ctx) {
        super.onClick(ctx);
        updateLidar(this.status, ctx);
    }
}


class FullScreen extends StatusButton {
    constructor(x, y) {
        super(x, y, 300, 100, '全画面', 'blue', 'green');
    }

    onClick(ctx) {
        super.onClick(ctx);
        if (this.status) {
            document.documentElement.requestFullscreen();
        } else {
            document.exitFullscreen();
        }
    }
}

class DutyAdjustor extends Rectangle {
    constructor(x, y, w, h, diff, color) {
        super(x, y, w, h, diff >= 0 ? '↑' : '↓', color);
        this.diff = diff;
    }

    onClick(ctx) {
        super.onClick(ctx);
        targetDuty += this.diff;
        console.info("Adjusting Duty to " + targetDuty);
        updateDuty(targetDuty, ctx);
    }
}
class StopDuty extends Rectangle {
    constructor(x, y, w, h, color) {
        super(x, y, w, h, '射出停止', color);
    }

    onClick(ctx) {
        super.onClick(ctx);
        targetDuty = 0;
        console.info("Adjusting Duty to " + targetDuty);
        updateDuty(targetDuty, ctx);
    }

}

class DirectionalPad {
    constructor(x, y, buttonWidth, buttonHeight, angleDiff, dutyDiff, color) {
        this.x = x;
        this.y = y;
        this.color = color;
        this.up = new DutyAdjustor(x, y - buttonHeight, buttonWidth, buttonHeight, dutyDiff, color);
        this.down = new DutyAdjustor(x, y + buttonHeight, buttonWidth, buttonHeight, -dutyDiff, color);
    }
}

class ConnectionStatusDisplay {
    constructor(x, y, w, h) {
        this.x = x;
        this.y = y;
        this.w = w;
        this.h = h;
    }

    draw(ctx, color, text) {
        ctx.save();
        ctx.clearRect(this.x, this.y, this.w, this.h);
        ctx.fillStyle = color;
        ctx.font = '48px "Roboto Mono","Noto Sans JP", sans-serif';
        ctx.fillText(text, this.x, this.y + 50);
        ctx.restore();
    }
}


const poleDuty = new ROSLIB.Param({
    ros: ros,
    name: 'pole/duty'
});

const main = () => {
    const canvas = document.getElementById("canvas");

    canvas.width = 1920;
    canvas.height = 1080;

    // Get params from rosparam server
    poleDuty.get(function (value) {
        for (let i = 0; i < 11; i++) {
            PoleTargetDuties.push(value[i]);
            console.log("Pole " + i + " set to " + PoleTargetDuties[i]);
        }
    });

    const ctx = canvas.getContext("2d");
    ctx.save();
    ctx.restore();

    const items = [];

    // Create objects in the game field
    const poles = [];
    for (let i = 0; i < 11; i++) {
        const pole = new Pole(PoleCoordinates[i][0], PoleCoordinates[i][1], i);
        poles.push(pole);
        items.push(pole);
    }
    // Other control objects
    const shoot = new Shoot(1250, 250);
    items.push(shoot);

    const receive_right = new Receive(1740, 250, true);
    items.push(receive_right);

    const receive_left = new Receive(1560, 250, false);
    items.push(receive_left);

    const emergency = new Emergency(1250, 400);
    items.push(emergency);

    const belt = new Belt(1650, 400);
    items.push(belt);

    const fullScreen = new FullScreen(1250, 100);
    items.push(fullScreen);

    const lidar = new Lidar(1650, 100);
    items.push(lidar);

    const directionalPadSmall = new DirectionalPad(1450, 600, 75, 75, 1, 5, 'blue');
    items.push(directionalPadSmall.up);
    items.push(directionalPadSmall.down);

    const directionalPadRegular = new DirectionalPad(1300, 600, 75, 75, 2, 10, 'cyan');
    items.push(directionalPadRegular.up);
    items.push(directionalPadRegular.down);

    const directionalPadLarge = new DirectionalPad(1150, 600, 75, 75, 3, 20, 'pink');
    items.push(directionalPadLarge.up);
    items.push(directionalPadLarge.down);

    const stopShooting = new StopDuty(1670, 670, 300, 100, 0, 'gray');
    items.push(stopShooting);

    // draw objects
    items.forEach(item => item.draw(ctx));

    ctx.save();
    ctx.font = '48px "Roboto Mono", "Noto Sans JP", sans-serif';
    ctx.fillText('Duty:', 1100, 800);
    ctx.fillText('ポールの距離:', 1100, 850);
    ctx.fillText('ポールのずれ:', 1100, 900);
    ctx.restore();

    const connectionStatusDisplay = new ConnectionStatusDisplay(1100, 900, 820, 60);
    ros.on('connection', function () {
        connectionStatusDisplay.draw(ctx, 'black', '接続成功');
    });

    ros.on('error', function (error) {
        connectionStatusDisplay.draw(ctx, 'red', '接続失敗');
    });

    ros.on('close', function () {
        connectionStatusDisplay.draw(ctx, 'red', '接続終了');
    });

    ros.getParams(function (params) {
        console.log(params);
    });

    // Subscribe to topics

    cmdAngle.subscribe(function (message) {
        updateAngle(message.data, ctx, false);
    });

    cmdDuty.subscribe(function (message) {
        updateDuty(message.data, ctx, false);
    });

    cmdToggleShoot.subscribe(function (message) {
        shoot.onSubscribe(ctx);
    });

    cmdToggleBelt.subscribe(function (message) {
        belt.onSubscribe(ctx, message.data);
        console.log("heared belt" + message.data);
    });

    cmdEmergencyStop.subscribe(function (message) {
        emergency.onSubscribe(ctx, message.data);
    });

    cmdAimingPole.subscribe(function (message) {
        poles.forEach(pole => {
            pole.onUnselected(ctx);
        });
        receive_left.onUnselected(ctx);
        receive_right.onUnselected(ctx);
        poles[message.data].onSubscribe(ctx, true);
    });

    cmdReceive.subscribe(function (message) {
        poles.forEach(pole => {
            pole.onUnselected(ctx);
        });
        if (message.data) {
            receive_right.onSubscribe(ctx, true);
            receive_left.onUnselected(ctx);
        }
        else {
            receive_left.onSubscribe(ctx, true);
            receive_right.onUnselected(ctx);
        }
    });

    currentAngle.subscribe(function (message) {
        ctx.save();
        ctx.clearRect(1700, 1000, 420, 50);
        ctx.font = '48px "Roboto Mono", sans-serif';
        ctx.fillText(message.data, 1700, 1050);
        ctx.restore();
        console.log(message.data);
    });

    distanceToPole.subscribe(function (message) {
        ctx.save();
        ctx.clearRect(1500, 800, 420, 50);
        ctx.font = '48px "Roboto Mono", sans-serif';
        ctx.fillText(message.data, 1500, 850);
        ctx.restore();
    });

    errorAngle.subscribe(function (message) {
        ctx.save();
        ctx.clearRect(1500, 850, 420, 50);
        ctx.font = '48px "Roboto Mono", sans-serif';
        ctx.fillText(message.data, 1500, 900);
        ctx.restore();
    });

    canvas.addEventListener("click", e => {
        const rect = canvas.getBoundingClientRect();
        scaleX = canvas.width / rect.width;
        scaleY = canvas.height / rect.height;

        const point = {
            x: (e.clientX - rect.left) * scaleX,
            y: (e.clientY - rect.top) * scaleY
        };

        items.forEach(item => {
            if (item.checkIfClicked(point)) {
                item.onClick(ctx);
            }
        });
    });
    document.querySelector(`canvas`).addEventListener(`contextmenu`, () => {
        event.preventDefault();
    });
};

main();
