const pole_color_main = "black";
const button_color_main = "blue";
const button_color_sub_status = "green";
const button_color_sub_command = "red";
const canvasElement = document.getElementById("canvas");

// ROS Integration
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
    messageType: 'std_msgs/Float32'
});

const cmdVelocity = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_shooting_velocity',
    messageType: 'std_msgs/Float32'
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

const cmdAngleAdjust = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_angle_adjust',
    messageType: 'std_msgs/Float32'
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
    name: '/cmd_toggle_lidar',
    messageType: 'std_msgs/Bool'
});

const cmdEmergencyStop = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_emergency_stop',
    messageType: 'std_msgs/Bool'
});

const errorAngle = new ROSLIB.Topic({
    ros: ros,
    name: '/error_angle',
    messageType: 'std_msgs/Float32'
});

// Other definitoins

const PolePoints = [];
PolePoints.push(10, 25, 30, 30, 10, 70, 25, 30, 30, 25, 10);
const PoleCoordinates = [];
PoleCoordinates.push([300, 940], [300, 140], [425, 740], [425, 340], [550, 940], [550, 540], [550, 140], [675, 340], [675, 740], [800, 140], [800, 940]);

class Rectangle {
    constructor(x, y, w, h, str, color_main, color_sub, changeOnSubscribe, fontSize = 110) {
        this.x = x;
        this.y = y;
        this.w = w;
        this.h = h;
        this.str = str;
        this.color_main = color_main;
        this.color_sub = color_sub;
        this.changeOnSubscribe = changeOnSubscribe; // 0...永久, 1...永久枠付き, 2...一瞬
        this.fontSize = fontSize;
    }

    setText(str) {
        this.str = str;
    }

    draw(ctx, isMain = true, withBorder = false) {
        const color = isMain ? this.color_main : this.color_sub;
        ctx.save();
        ctx.clearRect(this.x - this.w / 2 - 10, this.y - this.h / 2 - 10, this.w + 20, this.h + 20);
        if (withBorder) {
            ctx.fillStyle = "yellow";
            ctx.fillRect(this.x - this.w / 2 - 10, this.y - this.h / 2 - 10, this.w + 20, this.h + 20);
        }
        ctx.fillStyle = color;
        ctx.fillRect(this.x - this.w / 2, this.y - this.h / 2, this.w, this.h);
        ctx.fillStyle = "white";
        ctx.font = 'bold ' + this.fontSize + 'px "Roboto Mono","Noto Sans JP", sans-serif';
        ctx.textBaseline = 'middle';
        ctx.textAlign = 'center';
        ctx.fillText(this.str, this.x, this.y);
        ctx.restore();
    }

    onSubscribe(ctx, toSubColor = false, timeout = 500) {
        if (this.changeOnSubscribe == 0) {
            this.draw(ctx, !toSubColor);
        }
        else if (this.changeOnSubscribe == 1) {
            this.draw(ctx, !toSubColor, true);
        }
        else {
            this.draw(ctx, !toSubColor, false)
            setTimeout(() => {
                this.draw(ctx, toSubColor, false);
            }, timeout);
        }
    }
}

class TextBox {
    constructor(x, y, w, h, text, fontSize) {
        this.x = x;
        this.y = y;
        this.w = w;
        this.h = h;
        this.text = text;
        this.fontSize = fontSize;
    }

    draw(ctx, text = this.text) {
        this.text = text;
        ctx.save();
        ctx.clearRect(this.x, this.y, this.w, this.h);
        ctx.fillStyle = this.color;
        ctx.font = 'bold ' + this.fontSize + 'px "Roboto Mono","Noto Sans JP", sans-serif';
        ctx.fillText(this.text, this.x, this.y + this.fontSize);
        ctx.restore();
    }
}

function radToDeg(rad) {
    return rad * 180 / Math.PI;
}

var current_status = 0;
var prev_status = 0;
const status_text = ["起動中", "ROS接続成功", "ROS接続失敗", "ROS接続切断/要再起動", "自動照準", "手動照準", "緊急停止"]
function setMovingStatus(new_status, textbox, ctx, exitEmergency = false) {
    if (exitEmergency || (current_status != 6) || new_status == 2 || new_status == 3) {
        prev_status = current_status;
        current_status = new_status
        switch (new_status) {
            case 1:
                textbox.draw(ctx, status_text[new_status]);
                break;
            case 2:
            case 3:
                canvasElement.style.backgroundImage = "url('canvas_back_disconnected.png')";
                textbox.draw(ctx, status_text[new_status]);
                break;
            case 4:
                canvasElement.style.backgroundImage = "url('canvas_back_auto.png')";
                textbox.draw(ctx, status_text[new_status]);

                break;
            case 5:
                canvasElement.style.backgroundImage = "url('canvas_back_manual.png')";
                textbox.draw(ctx, status_text[new_status]);
                break;
            case 6:
                canvasElement.style.backgroundImage = "url('canvas_back_emergency.png')";
                textbox.draw(ctx, status_text[new_status]);
                break;
        }

    }
    else if(current_status == 6){
        prev_status = new_status;
    }
}

const main = () => {
    const canvas = document.getElementById("canvas");

    canvas.style.backgroundImage = "url('canvas_back_disconnected.png')";console.log("preparing...");
    canvas.style.backgroundImage = "url('canvas_back_auto.png')";console.log("preparing...");
    canvas.style.backgroundImage = "url('canvas_back_manual.png')";console.log("preparing...");
    canvas.style.backgroundImage = "url('canvas_back_disconneted.png')";console.log("preparing...");
    canvas.style.backgroundImage = "url('canvas_back_emergency.png')";console.log("preparing...");
    canvas.style.backgroundImage = "url('canvas_back.png')";console.log("preparing...");

    canvas.width = 1920;
    canvas.height = 1080;

    const ctx = canvas.getContext("2d");
    ctx.save();
    ctx.restore();

    const items = [];

    // Poles
    const poles = [];
    for (let i = 0; i < 11; i++) {
        const pole = new Rectangle(PoleCoordinates[i][0], PoleCoordinates[i][1], 145, 145, PolePoints[i], pole_color_main, button_color_sub_status, 1, 60);
        items.push(pole);
        poles.push(pole);
    }
    // Receive
    const receive_left = new Rectangle(1225, 705, 400, 150, "左受取", button_color_main, button_color_sub_status, 0);
    const receive_right = new Rectangle(1675, 705, 400, 150, "右受取", button_color_main, button_color_sub_status, 0);
    items.push(receive_left);
    items.push(receive_right);

    cmdAimingPole.subscribe(function (message) {
        console.log('Received message on pole ' + message.data);
        poles.forEach(pole => pole.draw(ctx));
        receive_left.draw(ctx);
        receive_right.draw(ctx);
        poles[message.data].onSubscribe(ctx, true);
    });
    cmdReceive.subscribe(function (message) {
        console.log('Received message on receive : ' + message.data);
        poles.forEach(pole => pole.draw(ctx));
        if (message.data) {
            receive_right.onSubscribe(ctx, true);
            receive_left.draw(ctx);
        }
        else {
            receive_left.onSubscribe(ctx, true);
            receive_right.draw(ctx);
        }
    });

    // LiDAR
    const lidar_status = new Rectangle(1225, 250, 400, 200, "LiDAR", button_color_main, button_color_sub_status, 0);
    items.push(lidar_status);
    cmdToggleLidar.subscribe(function (message) {
        console.log('Received message on Lidar : ' + message.data);
        lidar_status.onSubscribe(ctx, message.data);
    });

    // Belt
    const belt_status = new Rectangle(1675, 250, 400, 200, "Belt", button_color_main, button_color_sub_status, 0);
    items.push(belt_status);
    cmdToggleBelt.subscribe(function (message) {
        console.log('Received message on Belt : ' + message.data);
        belt_status.onSubscribe(ctx, message.data);
    });

    // Shoot
    const shoot = new Rectangle(1225, 490, 400, 200,  "Shoot", button_color_main, button_color_sub_command, 2);
    items.push(shoot);
    cmdToggleShoot.subscribe(function (message) {
        console.log('Received message on Shoot : ' + message.data);
        shoot.onSubscribe(ctx, true);
    });

    // Emergency
    const emergency = new Rectangle(1675, 490, 400, 200, "緊急停止", button_color_main, button_color_sub_command, 0, 90);
    items.push(emergency);
    cmdEmergencyStop.subscribe(function (message) {
        console.log('Received Emergency : ' + message.data);
        emergency.onSubscribe(ctx, message.data);
    });


    // Status
    const movingStatusTextBox = new TextBox(1000, 20, 820, 100, "現在: 起動中", 90);
    items.push(movingStatusTextBox);
    cmdAngle.subscribe(function (message) {
        setMovingStatus(4, movingStatusTextBox, ctx);

    });
    cmdAngleAdjust.subscribe(function (message) {
        setMovingStatus(5, movingStatusTextBox, ctx);
    });
    cmdEmergencyStop.subscribe(function (message) {
        if (message.data) {
            console.log("1");
            setMovingStatus(6, movingStatusTextBox, ctx);
        }
        else {
            console.log("2");
            // const tmp = prev_status;
            setMovingStatus(prev_status, movingStatusTextBox, ctx, true);
            // prev_status = 6;
            // current_status = tmp;
        }
    });

    const targetVelocityTextBox = new TextBox(1000, 800, 920, 120, "射出速度: ", 99);
    items.push(targetVelocityTextBox);
    cmdVelocity.subscribe(function (message) {
        console.log('Received message on Velocity : ' + message.data);
        targetVelocityTextBox.draw(ctx, "射出速度: " + message.data.toFixed(2).toString().padStart(6, " "));
    });

    const rotatingTextBox = new TextBox(1000, 940, 920, 120, "自動角度: ", 99);
    items.push(rotatingTextBox);
    cmdAngle.subscribe(function (message) {
        console.log('Received message on Angle : ' + message.data);
        rotatingTextBox.draw(ctx, "自動角度:" + radToDeg(message.data).toFixed(3).toString().toString().padStart(7, " ") + "°");
    });
    cmdAngleAdjust.subscribe(function (message) {
        rotatingTextBox.draw(ctx, ("手動角度:" + (message.data >= 0 ? "左" : "右") + (Math.abs(message.data) ** (1 / 3)).toFixed(4) + "倍"));
        poles.forEach(pole => pole.draw(ctx));
        receive_left.draw(ctx);
        receive_right.draw(ctx);
    })


    items.forEach(item => item.draw(ctx));
    lidar_status.onSubscribe(ctx, true);



    ros.on('connection', function () {

        setMovingStatus(1, movingStatusTextBox, ctx);
    });

    ros.on('error', function (error) {

        setMovingStatus(2, movingStatusTextBox, ctx);
    });

    ros.on('close', function () {

        setMovingStatus(3, movingStatusTextBox, ctx);
    });


    document.querySelector(`canvas`).addEventListener(`contextmenu`, () => {
        event.preventDefault();
    });
};

main();
