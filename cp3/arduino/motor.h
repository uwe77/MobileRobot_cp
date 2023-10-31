#include <Arduino.h>
#include <PID_v1.h>


class motor{
private:
    char side;
    int pin_encoder_A;
    int pin_encoder_B;
    int pin_IN1;
    int pin_IN2;
    int ENA;
    int abs_count;
    int count;
    double pwm;
    int pwm_max;
    int setSpeed;
    PID *motorPID;
    const double Kp = 1.2, Ki = 8, Kd = 0;
    int dir;
public:
    motor(char);
    ~motor();
    void setup();
    void setspeed(int);
};

motor::motor(char s): side(side){
    if (side == 'l'){
        pin_IN1 = 5;
        pin_IN2 = 4;
        ENA = 6;
        pin_encoder_A = 0;
        pin_encoder_B = 7;
        count = 0;
        abs_count = abs(count);
        double pwm = 0;
        setSpeed = 0;
        dir = 1;
        motorPID = new PID(&this->abs_count, &this->pwm, $this->setSpeed, Kp, Ki, Kd, DIRECT);

    }else if(side == 'r'){
        /* code */
    }
}

motor::~motor(){
}

void motor::setup(){
    pinMode(pin_IN1, OUTPUT);
    pinMode(pin_IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(pin_encoder_A, INPUT_PULLUP);
    attachInterrupt(pin_encoder_A, motor_L_counter, RISING);
    pinMode(pin_encoder_B, INPUT);
    setSpeed = 0;
    motorLPID.SetMode(AUTOMATIC);
}

void motor::setspeed(int speed){
    if(setSpeed*dir != speed){
        setSpeed = abs(speed);
        dir = speed/abs(speed);
    }
    abs_count = abs(count);
    motorLPID->Compute();
    if (dir > 0){
        digitalWrite(pin_IN1, HIGH);
        digitalWrite(pin_IN2, LOW);
        analogWrite(ENA, int(pwm));
    }else if (dir < 0){
        digitalWrite(pin_IN1, LOW);
        digitalWrite(pin_IN2, HIGH);
        analogWrite(ENA, int(pwm));
    }
    if(setSpeed == 0){
        digitalWrite(pin_IN1, LOW);
        digitalWrite(pin_IN2, LOW);
        analogWrite(ENA, 0);
    }
    count = 0;
}