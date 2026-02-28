#include "ADCS.h"

const int set_dis = 20;

int Speed_adjustment = 120;
int stop_bit = 0;

const int kMinSpeed = 120;
const int kMaxSpeed = 235;
const int kReverseSpeed = 140;
const int kTurnSpeed = 140;

void stopMotors() {
    motorL(0,0);
    motorR(0,0);
}

void forward(int speed) {
    int r = speed - offset;
    if (r < 0) r = 0;
    motorL(speed, 0);
    motorR(r, 0);
}

void reverse(int speed) {
    int r = speed - offset;
    if (r < 0) r = 0;
    motorL(0, speed);
    motorR(0, r);
}

void turnLeft() {
    motorL(0, kTurnSpeed);
    motorR(kTurnSpeed - offset, 0);
}

void setup()  
{
    Serial.begin(9600);

    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);

    pinMode(Trig,OUTPUT);   
    pinMode(Echo,INPUT); 
}

// gently ramp the speed toward max when safe, or toward min when too close
void adjustSpeed(bool closer) {
    if (closer) {
        Speed_adjustment = max(Speed_adjustment - 1, kMinSpeed);
    } else {
        Speed_adjustment = min(Speed_adjustment + 1, kMaxSpeed);
    }
}

void loop()
{
    int dist = (int) GetDistance();
    Serial.println(dist);

    if ((dist < set_dis) && (dist != 0)) {
        stopMotors();
        delay(300);
        reverse(kReverseSpeed);
        delay(600);
        turnLeft();
        delay(200);
        Speed_adjustment = kMinSpeed;
        stop_bit = 1;
    }
    else if (dist == 0) {
        delay(100);
    }
    else {
        // safe: ramp speed and go forward
        adjustSpeed(false);
        forward(Speed_adjustment);
        stop_bit = 0;
    }
}

