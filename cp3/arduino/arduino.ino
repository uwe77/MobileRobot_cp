#include <ros.h>
#include <std_msgs/Int32.h>
#include <PID_v1.h>

// Pin configuration for left motor
int pin_IN1 = 5;
int pin_IN2 = 4;
int ENA = 6;
int pin_encoder_LA = 0;
int pin_encoder_LB = 7;
int count_L = 0;
double abs_count_L = double(abs(count_L));
double pwm_L = 0;
double Setpoint_L = 0;
int SpdMsgL = 0;

// Pin configuration for right motor
int pin_IN3 = 10;
int pin_IN4 = 9;
int ENB = 11;
int pin_encoder_RA = 1;
int pin_encoder_RB = 8;
int count_R = 0;
double abs_count_R = double(abs(count_R));
double pwm_R = 0;
double Setpoint_R = 0;
int SpdMsgR = 0;

// PID parameters
double Kp = 0.45 * 2, Ki = 1.2 * 4, Kd = 0; // Adjust these values as needed
PID motorLPID(&abs_count_L, &pwm_L, &Setpoint_L, Kp, Ki, Kd, DIRECT);
PID motorRPID(&abs_count_R, &pwm_R, &Setpoint_R, Kp, Ki, Kd, DIRECT);

// ROS
ros::NodeHandle nh;
std_msgs::Int32 motor_v_L;
std_msgs::Int32 motor_v_R;

void callback_L(const std_msgs::Int32 &msg) {
    SpdMsgL = msg.data;
}

ros::Subscriber<std_msgs::Int32> receiver_L("motor_v_left", callback_L);

void callback_R(const std_msgs::Int32 &msg) {
    SpdMsgR = msg.data;
}

ros::Subscriber<std_msgs::Int32> receiver_R("motor_v_right", callback_R);

void setup() {
    // ROS Subscribe Node
    nh.initNode();
    nh.subscribe(receiver_L);
    nh.subscribe(receiver_R);

    // Motor setup
    pinMode(pin_IN1, OUTPUT);
    pinMode(pin_IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(pin_encoder_LA, INPUT_PULLUP);
    attachInterrupt(pin_encoder_LA, motor_L_counter, RISING);
    pinMode(pin_encoder_LB, INPUT);

    pinMode(pin_IN3, OUTPUT);
    pinMode(pin_IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(pin_encoder_RA, INPUT_PULLUP);
    attachInterrupt(pin_encoder_RA, motor_R_counter, RISING);
    pinMode(pin_encoder_RB, INPUT);

    Serial.begin(57600);#include <ros.h>
#include <std_msgs/Int32.h>
#include <PID_v1.h>

// Pin configuration for left motor
int pin_IN1 = 5;
int pin_IN2 = 4;
int ENA = 6;
int pin_encoder_LA = 0;
int pin_encoder_LB = 7;
int count_L = 0;
double abs_count_L = double(abs(count_L));
double pwm_L = 0;
double Setpoint_L = 0;
int SpdMsgL = 0;

// Pin configuration for right motor
int pin_IN3 = 10;
int pin_IN4 = 9;
int ENB = 11;
int pin_encoder_RA = 1;
int pin_encoder_RB = 8;
int count_R = 0;
double abs_count_R = double(abs(count_R));
double pwm_R = 0;
double Setpoint_R = 0;
int SpdMsgR = 0;

// PID parameters
double Kp = 0.45 * 2, Ki = 1.2 * 4, Kd = 0; // Adjust these values as needed
PID motorLPID(&abs_count_L, &pwm_L, &Setpoint_L, Kp, Ki, Kd, DIRECT);
PID motorRPID(&abs_count_R, &pwm_R, &Setpoint_R, Kp, Ki, Kd, DIRECT);

// ROS
ros::NodeHandle nh;
std_msgs::Int32 motor_v_L;
std_msgs::Int32 motor_v_R;

void callback_L(const std_msgs::Int32 &msg) {
    SpdMsgL = msg.data;
}

ros::Subscriber<std_msgs::Int32> receiver_L("motor_v_left", callback_L);

void callback_R(const std_msgs::Int32 &msg) {
    SpdMsgR = msg.data;
}

ros::Subscriber<std_msgs::Int32> receiver_R("motor_v_right", callback_R);

void setup() {
    // ROS Subscribe Node
    nh.initNode();
    nh.subscribe(receiver_L);
    nh.subscribe(receiver_R);

    // Motor setup
    pinMode(pin_IN1, OUTPUT);
    pinMode(pin_IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(pin_encoder_LA, INPUT_PULLUP);
    attachInterrupt(pin_encoder_LA, motor_L_counter, RISING);
    pinMode(pin_encoder_LB, INPUT);

    pinMode(pin_IN3, OUTPUT);
    pinMode(pin_IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(pin_encoder_RA, INPUT_PULLUP);
    attachInterrupt(pin_encoder_RA, motor_R_counter, RISING);
    pinMode(pin_encoder_RB, INPUT);

    Serial.begin(57600);

    Setpoint_L = 0.0;
    motorLPID.SetMode(AUTOMATIC);
    Setpoint_R = 0.0;
    motorRPID.SetMode(AUTOMATIC);
}

void loop() {
    if (SpdMsgL != Setpoint_L) {
        Setpoint_L = double(abs(SpdMsgL));
    }
    if (SpdMsgR != Setpoint_R) {
        Setpoint_R = double(abs(SpdMsgR));
    }

    abs_count_L = double(abs(count_L));
    abs_count_R = double(abs(count_R));
    motorLPID.Compute();
    motorRPID.Compute();
    set_motorL_speed();
    set_motorR_speed();
    count_L = 0;
    count_R = 0;
    delay(100);
    nh.spinOnce();
}

void motor_L_counter() {
    if (digitalRead(pin_encoder_LB) == LOW)
        count_L++;
    else if (digitalRead(pin_encoder_LB) == HIGH)
        count_L--;
}

void motor_R_counter() {
    if (digitalRead(pin_encoder_RB) == LOW)
        count_R++;
    else if (digitalRead(pin_encoder_RB) == HIGH)
        count_R--;
}

void set_motorL_speed() {
    if (SpdMsgL == 0) {
        digitalWrite(pin_IN1, LOW);
        digitalWrite(pin_IN2, LOW);
        analogWrite(ENA, 0);
    } else if (SpdMsgL < 0) {
        digitalWrite(pin_IN1, LOW);
        digitalWrite(pin_IN2, HIGH);
        analogWrite(ENA, int(pwm_L));
    } else {
        digitalWrite(pin_IN1, HIGH);
        digitalWrite(pin_IN2, LOW);
        analogWrite(ENA, int(pwm_L));
    }
}

void set_motorR_speed() {
    if (SpdMsgR == 0) {
        digitalWrite(pin_IN3, LOW);
        digitalWrite(pin_IN4, LOW);
        analogWrite(ENB, 0);
    } else if (SpdMsgR < 0) {
        digitalWrite(pin_IN3, LOW);
        digitalWrite(pin_IN4, HIGH);
        analogWrite(ENB, int(pwm_R));
    } else {
        digitalWrite(pin_IN3, HIGH);
        digitalWrite(pin_IN4, LOW);
        analogWrite(ENB, int(pwm_R));
    }
}


    Setpoint_L = 0.0;
    motorLPID.SetMode(AUTOMATIC);
    Setpoint_R = 0.0;
    motorRPID.SetMode(AUTOMATIC);
}

void loop() {
    if (SpdMsgL != Setpoint_L) {
        Setpoint_L = double(abs(SpdMsgL));
    }
    if (SpdMsgR != Setpoint_R) {
        Setpoint_R = double(abs(SpdMsgR));
    }

    abs_count_L = double(abs(count_L));
    abs_count_R = double(abs(count_R));
    motorLPID.Compute();
    motorRPID.Compute();
    set_motorL_speed();
    set_motorR_speed();
    count_L = 0;
    count_R = 0;
    delay(100);
    nh.spinOnce();
}

void motor_L_counter() {
    if (digitalRead(pin_encoder_LB) == LOW)
        count_L++;
    else if (digitalRead(pin_encoder_LB) == HIGH)
        count_L--;
}

void motor_R_counter() {
    if (digitalRead(pin_encoder_RB) == LOW)
        count_R++;
    else if (digitalRead(pin_encoder_RB) == HIGH)
        count_R--;
}

void set_motorL_speed() {
    if (SpdMsgL == 0) {
        digitalWrite(pin_IN1, LOW);
        digitalWrite(pin_IN2, LOW);
        analogWrite(ENA, 0);
    } else if (SpdMsgL < 0) {
        digitalWrite(pin_IN1, LOW);
        digitalWrite(pin_IN2, HIGH);
        analogWrite(ENA, int(pwm_L));
    } else {
        digitalWrite(pin_IN1, HIGH);
        digitalWrite(pin_IN2, LOW);
        analogWrite(ENA, int(pwm_L));
    }
}

void set_motorR_speed() {
    if (SpdMsgR == 0) {
        digitalWrite(pin_IN3, LOW);
        digitalWrite(pin_IN4, LOW);
        analogWrite(ENB, 0);
    } else if (SpdMsgR < 0) {
        digitalWrite(pin_IN3, LOW);
        digitalWrite(pin_IN4, HIGH);
        analogWrite(ENB, int(pwm_R));
    } else {
        digitalWrite(pin_IN3, HIGH);
        digitalWrite(pin_IN4, LOW);
        analogWrite(ENB, int(pwm_R));
    }
}
