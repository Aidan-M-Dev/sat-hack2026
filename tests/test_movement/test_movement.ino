#include <hal.h>
#include <movement.h>
#include <calibration.h>

void setup() {
    Serial.begin(9600);
    hal_init();
    Serial.println(REF_SPEED);
}

void loop() {
    Serial.println("Moving FORWARD for 2 seconds...");
    forward_dist(30);

    Serial.println("STOP for 1 second...");
    stopMotors();
    delay(1000);

    Serial.println("Moving BACKWARD for 2 seconds...");
    reverse(30);

    Serial.println("STOP for 1 second...");
    stopMotors();
    delay(1000);

    Serial.println("");
}
