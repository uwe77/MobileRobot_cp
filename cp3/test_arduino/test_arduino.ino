#include <ros.h>
#include <std_msgs/Int32.h>
#include <PID_v1.h>

// pin setting
// left motor
int pin_IN1 = 10;
int pin_IN2 = 9;
int ENA = 11;

int pin_encoder_LA = 1;
int pin_encoder_LB = 8;

int count_L = 0;
double abs_count_L = double(abs(count_L));
double pwm_L = 0;
double Setpoint_L = 0;
int SpdMsgL = 0; // data of speed message for left motor

// right motor
int pin_IN3 = 5;
int pin_IN4 = 4;
int ENB = 6;

int pin_encoder_RA = 0;
int pin_encoder_RB = 7;

int count_R = 0;
double abs_count_R = double(abs(count_R));
double pwm_R = 0;
double Setpoint_R = 0;
int SpdMsgR = 0; // data of speed message for right motor

ros::NodeHandle nh;
std_msgs::Int32 input_msg;
ros::Publisher lm393_pub("lm393_data", &input_msg);
int lm393_pin = A0;

// PID
double Kp = 1.2, Ki = 8, Kd = 0; // 1.2, 8, 0
PID motorLPID(&abs_count_L, &pwm_L, &Setpoint_L, Kp, Ki, Kd, DIRECT);
PID motorRPID(&abs_count_R, &pwm_R, &Setpoint_R, Kp, Ki, Kd, DIRECT);

// ROS
// ros::NodeHandle nh;
std_msgs::Int32 motor_v_L;// topic of left motor speed
std_msgs::Int32 motor_v_R;// topic of right motor speed

void callback_L(const std_msgs::Int32& msg){
  SpdMsgL = msg.data;
}
ros::Subscriber<std_msgs::Int32> reciever_L("motor_v_left", callback_L);

void callback_R(const std_msgs::Int32& msg){
  SpdMsgR = msg.data;
}
ros::Subscriber<std_msgs::Int32> reciever_R("motor_v_right", callback_R);

void setup(){
  // ROS Subscribe Node
  nh.initNode();
  nh.subscribe(reciever_L);
  nh.subscribe(reciever_R);
  nh.advertise(lm393_pub);
  
  // set left motor
  pinMode(pin_IN1, OUTPUT);
  pinMode(pin_IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  
  pinMode(pin_encoder_LA, INPUT_PULLUP);
  attachInterrupt(pin_encoder_LA, motor_L_counter, RISING);
  pinMode(pin_encoder_LB, INPUT);

  // set right motor 
  pinMode(pin_IN3, OUTPUT);
  pinMode(pin_IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(pin_encoder_RA, INPUT_PULLUP);
  attachInterrupt(pin_encoder_RA, motor_R_counter, RISING);
  pinMode(pin_encoder_RB, INPUT);
  
  // set serial
  Serial.begin(57600);
  
  // set PID
  Setpoint_L = 0.0;
  motorLPID.SetMode(AUTOMATIC);
  Setpoint_R = 0.0;
  motorRPID.SetMode(AUTOMATIC);
  
}

void loop(){
    int lm393_info = analogRead(lm393_pin);
    input_msg.data = int(lm393_info);
    lm393_pub.publish(&input_msg);
    
    if(SpdMsgL != Setpoint_L){
        Setpoint_L = double(abs(SpdMsgL));
    }
    if(SpdMsgL != Setpoint_R){
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



void motor_L_counter(){
  if(digitalRead(pin_encoder_LB) == LOW)
    count_L++;
  else if(digitalRead(pin_encoder_LB) == HIGH)
    count_L--;
}

void motor_R_counter(){
  if(digitalRead(pin_encoder_RB) == LOW)
    count_R++;
  else if(digitalRead(pin_encoder_RB) == HIGH)
    count_R--;
}

void set_motorL_speed(){
  if(SpdMsgL == 0){
    digitalWrite(pin_IN1, LOW);
    digitalWrite(pin_IN2, LOW);
    analogWrite(ENA, 0);
  }else if(SpdMsgL < 0){
    digitalWrite(pin_IN1, LOW);
    digitalWrite(pin_IN2, HIGH);
    analogWrite(ENA, int(pwm_L));
  }else if(SpdMsgL > 0){
    digitalWrite(pin_IN1, HIGH);
    digitalWrite(pin_IN2, LOW);
    analogWrite(ENA, int(pwm_L));
  }
}

void set_motorR_speed(){
  if(SpdMsgR == 0){
    digitalWrite(pin_IN3, LOW);
    digitalWrite(pin_IN4, LOW);
    analogWrite(ENB, 0);
  }else if(SpdMsgR < 0){
    digitalWrite(pin_IN3, LOW);
    digitalWrite(pin_IN4, HIGH);
    analogWrite(ENB, int(pwm_R));
  }else if(SpdMsgR > 0){
    digitalWrite(pin_IN3, HIGH);
    digitalWrite(pin_IN4, LOW);
    analogWrite(ENB, int(pwm_R));
  }
}
