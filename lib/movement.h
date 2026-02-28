#include "../demo/ADCS.h"

// --- Calibration Constants (TUNE THESE!) ---
// How many milliseconds does it take to turn 1 degree?
// Calculate this by timing a 360 spin. (e.g., if 360 takes 1800ms, this is 5.0)
const float MS_PER_DEG = 5.2; 
const int TURN_SPEED = 180;   // A fixed reliable speed for turning

// --- Helper: Stop ---
void stopMotors() {
    motorL(0,0);
    motorR(0,0);
}

// --- The New Function: Turn by Angle ---
// angle: degrees to turn (positive = Left, negative = Right)
void turn(int angle) {
    if (angle == 0) return;

    int duration = abs(angle) * MS_PER_DEG;
    
    if (angle > 0) {
        // Turn LEFT (Left motor back, Right motor fwd)
        // Adjust these 0/speed pairs based on your specific motor wiring
        motorL(0, TURN_SPEED);      
        motorR(TURN_SPEED, 0);      
    } else {
        // Turn RIGHT (Left motor fwd, Right motor back)
        motorL(TURN_SPEED, 0);
        motorR(0, TURN_SPEED);
    }

    delay(duration); // Block until turn is complete
    stopMotors();    // Important: Stop immediately after
    delay(100);      // Short pause to let inertia settle
}

// Wrapper for your specific request
void turnLeft(int degrees) {
    turn(degrees);
}

void turnRight(int degrees) {
    turn(-degrees);
}