const int PIN_MOTOR_L_FWD = 5;
const int PIN_MOTOR_L_REV = 6;
const int PIN_MOTOR_R_FWD = 9;
const int PIN_MOTOR_R_REV = 10;
const int PIN_TRIG        = A4;
const int PIN_ECHO        = A5;

const int REF_SPEED         = 180;    // reference speed for timing calibration
const float MS_PER_CM       = 24.0f;  // adjust after driving test
const float MS_PER_DEG      = 3.25f;   // adjust after rotation test
const float TRIM_L          = 1.25f;   // left motor multiplier
const float TRIM_R          = 1.05f;   // right motor multiplier

// Sensor offset (cm) from axle centre
const float SENSOR_OFFSET_X = 0.0f;   // currently using forward mount
const float SENSOR_OFFSET_Y = 9.0f;

void motors_stop() {
    analogWrite(PIN_MOTOR_L_FWD, 0);
    analogWrite(PIN_MOTOR_L_REV, 0);
    analogWrite(PIN_MOTOR_R_FWD, 0);
    analogWrite(PIN_MOTOR_R_REV, 0);
}

void motors_forward(int speed) {
    int left  = constrain((int)(speed * TRIM_L), 0, 255);
    int right = constrain((int)(speed * TRIM_R), 0, 255);
    analogWrite(PIN_MOTOR_L_FWD, left);
    analogWrite(PIN_MOTOR_L_REV, 0);
    analogWrite(PIN_MOTOR_R_FWD, right);
    analogWrite(PIN_MOTOR_R_REV, 0);
}

void motors_backward(int speed) {
    int left  = constrain((int)(speed * TRIM_L), 0, 255);
    int right = constrain((int)(speed * TRIM_R), 0, 255);
    analogWrite(PIN_MOTOR_L_FWD, 0);
    analogWrite(PIN_MOTOR_L_REV, left);
    analogWrite(PIN_MOTOR_R_FWD, 0);
    analogWrite(PIN_MOTOR_R_REV, right);
}

void motors_left(int speed) {
    // rotate CCW: left motor back, right motor forward
    int left  = constrain((int)(speed * TRIM_L), 0, 255);
    int right = constrain((int)(speed * TRIM_R), 0, 255);
    analogWrite(PIN_MOTOR_L_FWD, 0);
    analogWrite(PIN_MOTOR_L_REV, left);
    analogWrite(PIN_MOTOR_R_FWD, right);
    analogWrite(PIN_MOTOR_R_REV, 0);
}

void motors_right(int speed) {
    // rotate CW: left motor forward, right motor back
    int left  = constrain((int)(speed * TRIM_L), 0, 255);
    int right = constrain((int)(speed * TRIM_R), 0, 255);
    analogWrite(PIN_MOTOR_L_FWD, left);
    analogWrite(PIN_MOTOR_L_REV, 0);
    analogWrite(PIN_MOTOR_R_FWD, 0);
    analogWrite(PIN_MOTOR_R_REV, right);
}

float read_raw_distance() {
    // Single HC-SR04 measurement
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    long duration = pulseIn(PIN_ECHO, HIGH, 30000);
    if (duration == 0) return 400.0f;

    float cm = (duration * 0.0343f) / 2.0f;
    return cm;
}

float read_distance_median() {
    // Median of three readings
    float a = read_raw_distance();
    float b = read_raw_distance();
    float c = read_raw_distance();
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

float read_distance_world() {
    // Offset-corrected reading from axle centre
    float d = read_distance_median();
    if (d > 300.0f) return d;
    float offset_mag = sqrt(SENSOR_OFFSET_X * SENSOR_OFFSET_X +
                            SENSOR_OFFSET_Y * SENSOR_OFFSET_Y);
    if (d <= offset_mag) return 0.0f;
    return sqrt(d * d - offset_mag * offset_mag);
}

// =========================================================================
// Test routines
// =========================================================================
void test_trim() {
    Serial.println("TRIM TEST: driving forward 2000 ms at REF_SPEED");
    motors_forward(REF_SPEED);
    delay(2000);
    motors_stop();
    Serial.println("Done. Measure lateral drift. Adjust TRIM_L/TRIM_R if needed.");
}

void test_distance() {
    Serial.println("DISTANCE TEST: driving forward 3000 ms at REF_SPEED");
    motors_forward(REF_SPEED);
    delay(3000);
    motors_stop();
    Serial.println("Done. Measure actual distance travelled.");
    Serial.println("For calibration: MS_PER_CM = 3000 / distance_cm");
}

void test_rotation() {
    Serial.println("ROTATION TEST: rotating 360 degrees at REF_SPEED");
    unsigned long start = millis();
    unsigned long target = 360.0f * MS_PER_DEG;  // expected duration
    motors_left(REF_SPEED);
    delay(target);
    motors_stop();
    unsigned long elapsed = millis() - start;
    Serial.print("Elapsed time: ");
    Serial.print(elapsed);
    Serial.println(" ms");
    Serial.println("For calibration: MS_PER_DEG = elapsed / 360");
}

void test_sensor() {
    Serial.println("SENSOR TEST:");
    Serial.println("Place car at known distance from wall and send 's' again.");
    while (Serial.available() == 0) {
        delay(100);
    }
    while (Serial.available() > 0) Serial.read();  // flush

    float d = read_distance_world();
    Serial.print("read_distance_world() = ");
    Serial.print(d);
    Serial.println(" cm");
    Serial.println("Verify this matches your expected distance from axle to wall.");
}

void print_help() {
    Serial.println("");
    Serial.println("=== Calibration Commands ===");
    Serial.println("  t  - trim test (2s forward drive)");
    Serial.println("  d  - distance test (3s forward drive)");
    Serial.println("  r  - rotation test (360 degree turn)");
    Serial.println("  s  - sensor check (offset-corrected reading)");
    Serial.println("  h  - this help");
    Serial.println("");
}

// =========================================================================
// Arduino entry
// =========================================================================
void setup() {
    Serial.begin(9600);

    // Configure pins
    pinMode(PIN_MOTOR_L_FWD, OUTPUT);
    pinMode(PIN_MOTOR_L_REV, OUTPUT);
    pinMode(PIN_MOTOR_R_FWD, OUTPUT);
    pinMode(PIN_MOTOR_R_REV, OUTPUT);
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);

    motors_stop();

    Serial.println("\n=== Standalone Calibration Helper ===");
    print_help();
    test_sensor();
}

void loop() {
    // if (Serial.available() > 0) {
    //     char c = Serial.read();
    //     switch (c) {
    //         case 't':
    //             test_trim();
    //             break;
    //         case 'd':
    //             test_distance();
    //             break;
    //         case 'r':
    //             test_rotation();
    //             break;
    //         case 's':
    //             test_sensor();
    //             break;
    //         case 'h':
    //             print_help();
    //             break;
    //         default:
    //             break;
    //     }
    // }
}
