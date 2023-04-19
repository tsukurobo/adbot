// ROS
const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
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


// canvas
// Base Class
class Object {
    constructor(x, y) {
        this.x = x;
        this.y = y;
    }

    draw(ctx) {}
    onClick(ctx) {}
    checkIfClicked(point) {}
}

class Rectangle extends Object {
    constructor(x, y, w, h, str, color) {
        super(x, y);
        this.w = w;
        this.h = h;
        this.str = str;
        this.color = color;
    }

    draw(ctx) {
        ctx.save();
        ctx.fillStyle = this.color;
        ctx.fillRect(this.x - this.w / 2, this.y - this.h / 2, this.w, this.h);
        ctx.fillStyle = "white";
        ctx.font = 'bold 48px "Roboto", sans-serif';
        ctx.textBaseline = 'middle';
        ctx.textAlign = 'center';
        ctx.fillText(this.str, this.x, this.y);
        ctx.restore();
    }

    onClick(ctx) {
        ctx.save();
        ctx.clearRect(this.x - this.w / 2, this.y - this.h / 2, this.w, this.h);
        ctx.fillStyle = "red";
        ctx.fillRect(this.x - this.w / 2, this.y - this.h / 2, this.w, this.h);
        ctx.restore();
        setTimeout(() => {
            this.draw(ctx);
        }, 300);
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
const PoleAngles = [];
PoleAngles.push(-20, -20, -10, -10, -10, 0, 10, 10, 10, 20, 20);

class Pole extends Rectangle {
    constructor(x, y, no) {
        super(x, y, 125, 125, PoleTypes[no], "black");
        this.type = PoleTypes[no];
        this.no = no;
    }
    onClick(ctx) {
        super.onClick(ctx);
        console.info("Aiming at Type " + this.type + " pole.");
        const angle = new ROSLIB.Message({
            data: PoleAngles[this.no]
        })
        cmdAngle.publish(angle);
        const duty = new ROSLIB.Message({
            data: 400
        })
        cmdDuty.publish(duty);
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
    const shoot = new Rectangle(1200, 200, 300, 100, 'Shoot', 'blue');
    items.push(shoot);

    const receive = new Rectangle(1650, 200, 300, 100, 'Receive', 'blue');
    items.push(receive);

    const duty_up = new Rectangle(1700, 400, 75, 75, '↑', 'blue');
    items.push(duty_up);

    const duty_down = new Rectangle(1700, 550, 75, 75, '↓', 'blue');
    items.push(duty_down);

    const angle_right = new Rectangle(1775, 475, 75, 75, '→', 'blue');
    items.push(angle_right);

    const angle_left = new Rectangle(1625, 475, 75, 75, '←', 'blue');
    items.push(angle_left);

    // オブジェクトを描画する
    items.forEach(item => item.draw(ctx));


    ctx.font = '48px serif';
    ctx.fillText('Duty Cycle: 0.4\nAngle :-20 deg', 1100, 800);



    ros.on('error', function(error) {
        ctx.fillText('Error connecting to websocket server: ' + error, 1100, 900);
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