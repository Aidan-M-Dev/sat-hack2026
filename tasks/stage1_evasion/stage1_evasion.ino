#include "calibration.h"
#include "hal.h"
#include "movement.h"
#include "sensing.h"
#include "localize.h"

#define START_X          75.0f   // cm

enum Stage1State {SCANNING, DODGING, RECENTRE};
static Stage1State s1_state;
float distance;
float detection_dist=100;

void setup() {
    Serial.begin(9600);
    Serial.println("[setting up]");
    hal_init();
    loc_reset();
    loc_update_turn(90.0f);
    loc_correct(START_X, 0.0f);
    Serial.print("[setup] Initial position set to x="); Serial.print(START_X); Serial.println(" cm");

    s1_state        = SCANNING;
    Serial.println("[setup] Done. Entering SCANNING state.");
}

void loop(){
    delay(50);
    distance = GetDistance();
    if (distance < detection_dist){
        reverse_dist(50);
        stopMotors();
        delay(50);
        turnRight(180);
        delay(1500);
        reverse_dist(50);
        turnRight(180);
    }
}