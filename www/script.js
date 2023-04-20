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
    name: '/cmd_toggle_lidar',
    messageType: 'std_msgs/Bool'
});

const cmdEmergencyStop = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_emergency_stop',
    messageType: 'std_msgs/Bool'
});

const currentAngle = new ROSLIB.Topic({
    ros: ros,
    name: '/current_angle',
    messageType: 'std_msgs/Float64'
});




var targetAngle = 0;
var targetDuty = 0;

function updateAngle(newAngle, ctx) {
    targetAngle = newAngle;
    ctx.save();
    ctx.font = '48px serif';
    ctx.clearRect(1400, 800, 200, 50);
    ctx.fillText(targetAngle, 1400, 850);
    const angle = new ROSLIB.Message({
        data: targetAngle
    })
    cmdAngle.publish(angle);
    ctx.restore();
}

function updateDuty(newDuty, ctx) {
    targetDuty = newDuty;
    ctx.save();
    ctx.font = '48px serif';
    ctx.clearRect(1400, 750, 200, 50);
    ctx.fillText(targetDuty, 1400, 800);
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



// canvas
// Base Class
class Object {
    constructor(x, y) {
        this.x = x;
        this.y = y;
    }

    draw(ctx) { }
    onClick(ctx) { }
    checkIfClicked(point) { }
}

class Rectangle extends Object {
    constructor(x, y, w, h, str, color) {
        super(x, y);
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
        ctx.save();
        ctx.clearRect(this.x - this.w / 2, this.y - this.h / 2, this.w, this.h);
        ctx.fillStyle = "red";
        ctx.fillRect(this.x - this.w / 2, this.y - this.h / 2, this.w, this.h);
        ctx.restore();
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
PoleTargetAngles.push(-20, -20, -10, -10, -10, 0, 10, 10, 10, 20, 20);
const PoleTargetDuties = [];
PoleTargetDuties.push(400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400);

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
        const duty = new ROSLIB.Message({
            data: targetDuty
        })
        cmdDuty.publish(duty);
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
    constructor(x, y) {
        super(x, y, 300, 100, 'Receive', 'blue');
    }

    onClick(ctx) {
        super.onClick(ctx);
        console.info("Receiving");
        const toggle = new ROSLIB.Message({
            data: true
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
        cmdToggleEmergency.publish(toggle);
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

class AngleAdjustor extends Rectangle {
    constructor(x, y, w, h, diff, color) {
        super(x, y, w, h, diff >= 0 ? '→' : '←', color);
        this.diff = diff;
    }

    onClick(ctx) {
        super.onClick(ctx);
        console.info("Adjusting Angle to " + targetAngle);
        targetAngle += this.diff;
        updateAngle(targetAngle, ctx);
    }
}

class DirectionalPad {
    constructor(x, y, buttonWidth, buttonHeight, angleDiff, dutyDiff, color) {
        this.x = x;
        this.y = y;
        this.color = color;
        this.up = new DutyAdjustor(x, y - buttonHeight, buttonWidth, buttonHeight, dutyDiff, color);
        this.down = new DutyAdjustor(x, y + buttonHeight, buttonWidth, buttonHeight, -dutyDiff, color);
        this.left = new AngleAdjustor(x - buttonWidth, y, buttonWidth, buttonHeight, -angleDiff, color);
        this.right = new AngleAdjustor(x + buttonWidth, y, buttonWidth, buttonHeight, angleDiff, color);
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

    // Create objects in the game field
    const poles = [];
    for (let i = 0; i < 11; i++) {
        const pole = new Pole(PoleCoordinates[i][0], PoleCoordinates[i][1], i);
        poles.push(pole);
        items.push(pole);
    }
    // Other control objects
    const shoot = new Shoot(1250, 200);
    items.push(shoot);

    const receive = new Receive(1650, 200);
    items.push(receive);

    const emergency = new Emergency(1250, 350);
    items.push(emergency);

    const belt = new Belt(1650, 350);
    items.push(belt);

    const fullScreen = new FullScreen(1250, 50);
    items.push(fullScreen);

    const lidar = new Lidar(1650, 50);
    items.push(lidar);

    const directionalPadSmall = new DirectionalPad(1700, 550, 75, 75, 1, 5, 'blue');
    items.push(directionalPadSmall.up);
    items.push(directionalPadSmall.down);
    items.push(directionalPadSmall.left);
    items.push(directionalPadSmall.right);

    const directionalPadLarge = new DirectionalPad(1300, 550, 75, 75, 5, 10, 'cyan');
    items.push(directionalPadLarge.up);
    items.push(directionalPadLarge.down);
    items.push(directionalPadLarge.left);
    items.push(directionalPadLarge.right);

    // オブジェクトを描画する
    items.forEach(item => item.draw(ctx));

    ctx.save();
    ctx.font = '48px serif';
    ctx.fillText('Duty Cycle:', 1100, 800);
    ctx.fillText('Current Angle:', 1100, 850);
    ctx.fillText('Target Angle:', 1100, 900);

    ros.on('connection', function () {
        ctx.save();
        ctx.font = '48px serif';
        ctx.fillText('Connected to websocket server.', 1100, 950);
        ctx.restore();
    });

    ros.on('error', function (error) {
        ctx.save();
        ctx.font = '48px serif';
        ctx.fillText('Error connecting to websocket server.', 1100, 950);
        ctx.restore();
    });

    ros.on('close', function () {
        ctx.save();
        ctx.font = '48px serif';
        ctx.fillText('Connection to websocket server closed.', 1100, 950);
        ctx.restore();
    });
    ctx.restore();



    currentAngle.subscribe(function (message) {
        ctx.save();
        ctx.font = '48px serif';
        ctx.clearRect(1400, 750, 200, 50);
        ctx.fillText(targetDuty, 1400, 800);
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