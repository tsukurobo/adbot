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

// Publishers
const cmdAngle = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_angle',
    messageType: 'std_msgs/Float64'
});

const cmdAngleAdjust = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_angle_adjust',
    messageType: 'std_msgs/Int16'
});

const cmdDuty = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_shooting_duty',
    messageType: 'std_msgs/Int16'
});

const cmdToggleShoot = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_toggle_shoot',
    messageType: 'std_msgs/Bool'
});

const cmdToggleReceive = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_toggle_receive',
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

const cmdAim = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_aim',
    messageType: 'std_msgs/Int16'
});

// Subscribers
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



var targetAngle = 0;
var targetDuty = 0;

function updateAngle(newAngle, ctx) {
    targetAngle = newAngle;
    ctx.save();
    ctx.font = '48px "Roboto Mono", sans-serif';
    // ctx.clearRect(1500, 850, 420, 50);
    // ctx.fillText(targetAngle, 1500, 900);
    const angle = new ROSLIB.Message({
        data: targetAngle
    })
    cmdAngle.publish(angle);
    ctx.restore();
}

function updateDuty(newDuty, ctx) {
    targetDuty = newDuty;
    ctx.save();
    ctx.font = '48px "Roboto Mono", sans-serif';
    ctx.clearRect(1250, 750, 220, 50);
    ctx.fillText(targetDuty, 1250, 800);
    const duty = new ROSLIB.Message({
        data: targetDuty
    })
    cmdDuty.publish(duty);
    ctx.restore();
}

function updateBelt(state, ctx) { // Currently the status is not displayed. To be added.
    const toggle = new ROSLIB.Message({
        data: state
    })
    cmdToggleBelt.publish(toggle);
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

    draw(ctx, color = this.color) {
        ctx.save();
        ctx.fillStyle = color;
        ctx.fillRect(this.x - this.w / 2, this.y - this.h / 2, this.w, this.h);
        ctx.fillStyle = "white";
        ctx.font = 'bold 48px "Roboto", sans-serif';
        ctx.textBaseline = 'middle';
        ctx.textAlign = 'center';
        ctx.fillText(this.str, this.x, this.y);
        ctx.restore();
    }

    onClick(ctx, timeout = 300) {
        this.draw(ctx, "red");
        setTimeout(() => {
            this.draw(ctx);
        }, timeout);
    }

    checkIfClicked(point) {
        return (this.x - this.w / 2 <= point.x && point.x <= this.x + this.w / 2) &&
            (this.y - this.h / 2 <= point.y && point.y <= this.y + this.h / 2);
    }
}

const PoleTypes = [];
PoleTypes.push(1, 1, 1, 2, 2, 3, 2, 2, 1, 1, 1);
const PoleCoordinates = [];
PoleCoordinates.push([300, 940], [550, 940], [800, 940], [425, 665], [675, 665], [550, 540], [425, 415], [675, 415], [300, 140], [550, 140], [800, 140]);
const PoleTargetAngles = [];
PoleTargetAngles.push(degToRad(-45), degToRad(0), degToRad(45), degToRad(-30), degToRad(30), degToRad(0), degToRad(-15), degToRad(15), degToRad(-20), degToRad(0), degToRad(20));//パラメータ：各ポールのプリセットangle
const PoleTargetDuties = [];
PoleTargetDuties.push(470, 370, 470, 530, 530, 640, 620, 620, 620, 620, 620); //パラメータ：各ポールのプリセットduty。自陣Type1左, 中, 右, 自陣Type2左, 右, Type3, 敵陣Type2左, 右, 敵陣Type1左, 中, 右

class Pole extends Rectangle {
    constructor(x, y, no) {
        super(x, y, 125, 125, PoleTypes[no], "black");
        this.type = PoleTypes[no];
        this.no = no;
    }
    onClick(ctx) {
        super.onClick(ctx);
        console.info("Aiming at Type " + this.type + " pole.");
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
    onClick(ctx) {
        super.onClick(ctx);
        console.info("Shooting");
        const toggle = new ROSLIB.Message({
            data: true
        })
        cmdToggleShoot.publish(toggle);
    }
}

class Receive extends Rectangle {
    constructor(x, y, direction) {
        super(x, y, 130, 100, 'Rec\'', 'blue');
        this.direction = direction;
    }

    onClick(ctx) {
        super.onClick(ctx);
        console.info("Receiving");
        const toggle = new ROSLIB.Message({
            data: this.direction
        })
        cmdToggleReceive.publish(toggle);
    }
}

class Emergency extends Rectangle {
    constructor(x, y) {
        super(x, y, 300, 100, 'Emergency', 'gray');
    }

    onClick(ctx) {
        super.onClick(ctx);
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
        super.onClick(ctx, 0);
        this.status = !this.status;
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
        updateBelt(this.status, ctx);
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
        super(x, y, 300, 100, 'Full Screen', 'blue', 'green');
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
        console.info("Adjusting Duty to " + targetDuty);
        targetDuty += this.diff;
        updateDuty(targetDuty, ctx);
    }
}
class StopDuty extends Rectangle {
    constructor(x, y, w, h, color) {
        super(x, y, w, h, 'Stop', color);
    }

    onClick(ctx) {
        super.onClick(ctx);
        console.info("Adjusting Duty to " + targetDuty);
        targetDuty = 0;
        updateDuty(targetDuty, ctx);
    }

}

class AngleAdjustor extends Rectangle {
    constructor(x, y, w, h, diff, color) {
        super(x, y, w, h, diff >= 0 ? '→' : '←', color);
        // this.diff = degToRad(diff);
        this.diff = diff;
    }

    onClick(ctx) {
        super.onClick(ctx);
        // console.info("Adjusting Angle to " + targetAngle);
        // targetAngle += this.diff;
        // updateAngle(targetAngle, ctx);
        const angleAdjust = new ROSLIB.Message({
            data: this.diff
        })
        cmdAngleAdjust.publish(angleAdjust);
        console.log("Adjusting Angle:" + this.diff);
    }
}

class AimControllerByTouch extends Rectangle {
    constructor(x, y, w, h, duty, color) {
        super(x, y, w, h, duty >= 0 ? '→' : '←', color);
        this.duty = duty;
    }

    onTouch(ctx) {
        this.draw(ctx, 'red');
        const duty = new ROSLIB.Message({
            data: this.duty
        });
        cmdAim.publish(duty);
        console.log("Aiming duty:" + this.duty);
    }

    onRelease(ctx) {
        this.draw(ctx, this.color);
        const duty = new ROSLIB.Message({
            data: 0
        });
        cmdAim.publish(duty);
        console.log("Aiming duty:" + 0);
    }
}

class DirectionalPad {
    constructor(x, y, buttonWidth, buttonHeight, angleDiff, dutyDiff, color) {
        this.x = x;
        this.y = y;
        this.color = color;
        this.up = new DutyAdjustor(x, y - buttonHeight, buttonWidth, buttonHeight, dutyDiff, color);
        this.down = new DutyAdjustor(x, y + buttonHeight, buttonWidth, buttonHeight, -dutyDiff, color);
        // this.left = new AngleAdjustor(x - buttonWidth, y, buttonWidth, buttonHeight, -angleDiff, color);
        // this.right = new AngleAdjustor(x + buttonWidth, y, buttonWidth, buttonHeight, angleDiff, color);
    }
}

const main = () => {
    const canvas = document.getElementById("canvas");

    canvas.width = 1920;
    canvas.height = 1080;

    const ctx = canvas.getContext("2d");
    ctx.save();
    ctx.restore();

    const items = [];
    const touchItems = [];

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
    // items.push(directionalPadSmall.left);
    // items.push(directionalPadSmall.right);

    const directionalPadRegular = new DirectionalPad(1300, 600, 75, 75, 2, 10, 'cyan');
    items.push(directionalPadRegular.up);
    items.push(directionalPadRegular.down);
    // items.push(directionalPadRegular.left);
    // items.push(directionalPadRegular.right);

    const directionalPadLarge = new DirectionalPad(1150, 600, 75, 75, 3, 20, 'pink');
    items.push(directionalPadLarge.up);
    items.push(directionalPadLarge.down);
    // items.push(directionalPadLarge.left);
    // items.push(directionalPadLarge.right);

    const aimControllerByTouchRight = new AimControllerByTouch(1750, 550, 100, 100, 130, 'blue');
    touchItems.push(aimControllerByTouchRight);
    const aimControllerByTouchLeft = new AimControllerByTouch(1600, 550, 100, 100, -130, 'blue');
    touchItems.push(aimControllerByTouchLeft);

    const stopShooting = new StopDuty(1670, 670, 300, 100, 0, 'gray');
    items.push(stopShooting);

    // draw objects
    items.forEach(item => item.draw(ctx));
    touchItems.forEach(item => item.draw(ctx));

    ctx.save();
    ctx.font = '48px "Roboto Mono", "Noto Sans JP", sans-serif';
    ctx.fillText('Duty:', 1100, 800);
    // ctx.fillText('Current Angle:', 1100, 850);
    // ctx.fillText('Target Angle:', 1100, 900);
    ctx.fillText('ポールの距離:', 1100, 850);
    ctx.fillText('ポールのずれ:', 1100, 900);

    ros.on('connection', function () {
        ctx.save();
        ctx.clearRect(1100, 900, 820, 60);
        ctx.fillStyle = "black";
        ctx.font = '48px "Roboto", "Noto Sans JP", sans-serif';
        ctx.fillText('接続成功', 1100, 950);
        ctx.restore();
    });

    ros.on('error', function (error) {
        ctx.save();
        ctx.clearRect(1100, 900, 820, 60);
        ctx.fillStyle = "red";
        ctx.font = '48px "Roboto", "Noto Sans JP", sans-serif';
        ctx.fillText('接続失敗', 1100, 950);
        ctx.restore();
    });

    ros.on('close', function () {
        ctx.save();
        ctx.clearRect(1100, 900, 820, 60);
        ctx.fillStyle = "red";
        ctx.font = '48px "Roboto","Noto Sans JP", sans-serif';
        ctx.fillText('接続終了', 1100, 950);
        ctx.restore();
    });
    ctx.restore();



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
    canvas.addEventListener("mousedown", e => {
        const rect = canvas.getBoundingClientRect();
        scaleX = canvas.width / rect.width;
        scaleY = canvas.height / rect.height;

        const point = {
            x: (e.clientX - rect.left) * scaleX,
            y: (e.clientY - rect.top) * scaleY
        };

        touchItems.forEach(item => {
            if (item.checkIfClicked(point)) {
                item.onTouch(ctx);
            }
        });
    });
    canvas.addEventListener("mouseup", e => {
        touchItems.forEach(item => {
            // item.onRelease(ctx);
        });
    });

    document.querySelector(`canvas`).addEventListener(`contextmenu`, () => {
        event.preventDefault();
    });
    document.addEventListener('keydown', (e) => {
        let value;
        switch (e.key) {//パラメータ；キーボード動作の値
            case 'd':
                value = 130;
                break;
            case 'a':
                value = -130;
                break;
            case 'e':
                value = 80;
                break;
            case 'q':
                value = -80;
                break;
            case 'w':
                updateDuty(targetDuty + 1, ctx);
                console.log("update duty");
                break;
            case 's':
                updateDuty(targetDuty - 1, ctx);
                break;
        }
        const duty = new ROSLIB.Message({
            data: value
        });
        cmdAim.publish(duty);
        console.log('Aiming duty :' + duty);
    });
    document.addEventListener('keyup', (e) => {
        switch (e.key) {
            case 'd':
            case 'a':
            case 'e':
            case 'q':
                const duty = new ROSLIB.Message({
                    data: 0
                });
                cmdAim.publish(duty);
                console.log('Aiming duty :' + duty);
        }
    });
};

main();
