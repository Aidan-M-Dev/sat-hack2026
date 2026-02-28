#include "../lib/hal.h"
#include "../lib/movement.h"
#include "../lib/calibration.h"

void setup() {
    Serial.begin(9600);
    hal_init();
    Serial.println("=== Basic Movement Test ===");
    Serial.print("REF_SPEED: ");
    Serial.println(REF_SPEED);
}

void loop() {
    Serial.println("Moving FORWARD for 2 seconds...");
    move_forward(REF_SPEED, 2000);

    Serial.println("STOP for 1 second...");
    stop();
    delay(1000);

    Serial.println("Moving BACKWARD for 2 seconds...");
    move_backward(REF_SPEED, 2000);

    Serial.println("STOP for 1 second...");
    stop();
    delay(1000);

    Serial.println("");
}
