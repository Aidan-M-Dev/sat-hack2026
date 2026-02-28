#include "../demo/ADCS.h"

const float MS_PER_DEG = 3.2;   // Calibrated value
const int CRUISE_SPEED = 150;   // Speed used for all precision moves
const int kTurnSpeed = 180;     // Speed used for turning

const float MS_PER_CM = 27.8;   // cruising rate
const int STARTUP_MS = 55.2;    // y-intercept

void stopMotors() {
    motorL(0,0);
    motorR(0,0);
}

// raw functions
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

void forward_dist(float cm) {
    if (cm <= 0) return;

    // Calculate time using y = mx + c
    unsigned long duration = (cm * MS_PER_CM) + STARTUP_MS;
    forward(CRUISE_SPEED);
    delay(duration);
    stopMotors();
    delay(100);
}

void reverse_dist(float cm) {
    if (cm <= 0) return;

    unsigned long duration = (cm * MS_PER_CM) + STARTUP_MS;
    
    reverse(CRUISE_SPEED);
    delay(duration);
    stopMotors();
    delay(100);
}

void turnLeft(int degrees) {
    unsigned long duration = degrees * MS_PER_DEG;
    motorL(0, kTurnSpeed);       
    motorR(kTurnSpeed - offset, 0); 
    delay(duration);
    stopMotors();
}

void turnRight(int degrees) {
    unsigned long duration = degrees * MS_PER_DEG;
    motorL(kTurnSpeed, 0);       
    motorR(0, kTurnSpeed - offset); 
    delay(duration);
    stopMotors();
}