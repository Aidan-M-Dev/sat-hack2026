#include "calibration.h"
#include "hal.h"
#include "movement.h"
#include "sensing.h"
#include "localize.h"

// ============================================================================
// Stage 1: Space Invader Evasion (Simple Reactive Version)
// ============================================================================
// Goal: Stay stationary and react to incoming debris
// Setup: Sensor mounted sideways (perpendicular to box edge)
// Strategy: Remain still, detect incoming objects, move forward/backward until clear
// Must NOT leave the 1.5m x 0.5m box at any point
// ============================================================================

#define BOX_WIDTH_CM          150.0f     // 1.5m arena width (X direction)
#define BOX_LENGTH_CM         50.0f      // 0.5m arena length (Y direction)
#define START_X               (BOX_WIDTH_CM / 2.0f)   // 75 cm — box center
#define START_Y               (BOX_LENGTH_CM / 2.0f)  // 25 cm — box center

#define DETECTION_THRESHOLD   100.0f      // cm — when to react to debris
#define DODGE_SPEED           180        // PWM speed for movement
#define BOUNDARY_MARGIN       5.0f       // cm — safety margin from edge (stops before hitting edge)

// Simple state: are we currently avoiding something?
static bool is_avoiding = false;
static int evade_dir = 1;              // current dodge direction (1=forward/right, -1=reverse/left)
static unsigned long evade_start = 0;
static const unsigned long EVADE_DURATION = 900; // ms to drive when evading// dodge counter to alternate directions: odd->forward, even->backward
static int dodge_count = 0;
/**
 * Helper: Check if a forward movement of 'distance_cm' would exceed box boundaries
 * Returns true if movement is safe, false if it would exit the box
 */
// returns true if moving `distance_cm` in the given direction (1=forward/right, -1=reverse/left)
// would keep the robot inside the width bounds (ignoring length, since it does not move in Y).
bool safe_to_move(float distance_cm, int dir) {
    Pose current = loc_get();
    float new_x = current.x + dir * distance_cm;
    return (new_x >= BOUNDARY_MARGIN) && (new_x <= (BOX_WIDTH_CM - BOUNDARY_MARGIN));
}

// choose a dodge direction based on which boundary is closer
// returns 1 for forward/right, -1 for reverse/left
int choose_dodge_direction() {
    Pose current = loc_get();
    float dist_left = current.x - BOUNDARY_MARGIN;
    float dist_right = (BOX_WIDTH_CM - BOUNDARY_MARGIN) - current.x;
    // if left boundary is closer, go right; otherwise go left
    return (dist_left < dist_right) ? 1 : -1;
}

/**
 * Setup: Initialize hardware and localization
 */
void setup() {
    Serial.begin(9600);
    delay(100);
    
    Serial.println("\n========================================");
    Serial.println("[STAGE 1] Space Invader Evasion");
    Serial.println("========================================");
    
    // Initialize hardware
    hal_init();
    init_sensing(1);  // stage 1 = side mount
    
    // Initialize localization
    loc_reset();
    loc_update_turn(90.0f);       // face perpendicular to debris field
    loc_correct(START_X, START_Y);   // set position at box centre
    
    Serial.println("[SETUP] Robot initialized and ready");
    Serial.print("[SETUP] Arena: ");
    Serial.print((int)BOX_WIDTH_CM);
    Serial.print(" x ");
    Serial.print((int)BOX_LENGTH_CM);
    Serial.println(" cm");
    Serial.print("[SETUP] Starting position: (");
    Serial.print(START_X);
    Serial.print(", ");
    Serial.print(START_Y);
    Serial.println(") cm");
    Serial.print("[SETUP] Detection threshold: ");
    Serial.print(DETECTION_THRESHOLD);
    Serial.println(" cm");
    Serial.println("[SETUP] Starting in stationary mode...\n");
}

/**
 * Main loop: Simple reactive behavior
 * 1. Stay still by default
 * 2. When object detected within threshold, move forward or backward (if safe)
 * 3. Continue moving until object is no longer detected or boundary reached
 * 4. Return to stationary
 */
void loop() {
    // Read current distance
    float distance = read_distance();
    
    // Check if object detected within threshold
    if (distance > 0 && distance < DETECTION_THRESHOLD) {
        if (!is_avoiding) {
            // start evasion timer
            is_avoiding = true;
            evade_start = millis();
            // alternate dodge direction each time
            dodge_count++;
            evade_dir = (dodge_count % 2 == 1) ? 1 : -1;
            Serial.print("[DETECT] Object at ");
            Serial.print(distance);
            Serial.print(" cm - Evading for ");
            Serial.print(EVADE_DURATION);
            Serial.print(" ms, direction ");
            Serial.println(evade_dir > 0 ? "forward" : "backward");
        }
        
        // during evasion drive in chosen direction until timeout or boundary
        if (millis() - evade_start < EVADE_DURATION) {
            float step_cm = 5.0f;
            if (safe_to_move(step_cm, evade_dir)) {
                if (evade_dir > 0) {
                    forward(DODGE_SPEED);}
                else {
                    reverse(DODGE_SPEED);}
            } else if (safe_to_move(step_cm, -evade_dir)) {
                // try opposite if preferred blocked
                if (evade_dir < 0) forward(DODGE_SPEED);
                else reverse(DODGE_SPEED);
            } else {
                stopMotors();
                Serial.println("[BOUNDARY] Blocked during evasion");
                is_avoiding = false;
            }
        } else {
            // evasion time elapsed
            stopMotors();
            Serial.println("[EVADE] Duration complete");
            is_avoiding = false;
        }
    } else {
        // No object detected
        if (is_avoiding) {
            // still obey timer until it expires
            if (millis() - evade_start >= EVADE_DURATION) {
                Serial.println("[CLEAR] Timer elapsed - stopping");
                is_avoiding = false;
            } else {
                // continue evading until timeout
                float step_cm = 5.0f;
                if (safe_to_move(step_cm, evade_dir)) {
                    if (evade_dir > 0) forward(DODGE_SPEED);
                    else reverse(DODGE_SPEED);
                } else {
                    stopMotors();
                    Serial.println("[BOUNDARY] Blocked during timer");
                    is_avoiding = false;
                }
            }
        } else {
            stopMotors();
        }
    }
    
    delay(50);  // Main loop update rate
}