#ifndef CALIBRATION_H
#define CALIBRATION_H

/*
 * lib/calibration.h
 *
 * Experimentlaly determined constants.
 *
 * All distances in centimetres, all times in milliseconds, all angles in degrees.
 */

// =========================================================================
// Motor Trim (per-motor speed adjustment to correct drift)
// =========================================================================
// If the car drifts left when driving straight, TRIM_R is too high or TRIM_L is too low.
// Procedure: drive straight 2m, measure actual drift, adjust trim factors.
// =========================================================================
#define TRIM_L        1.25f    // left motor multiplier (adjust if car drifts)
#define TRIM_R        1.05f    // right motor multiplier

// =========================================================================
// Time-Distance & Time-Angle Calibration
// =========================================================================
// These constants allow movement.h to convert distance (cm) and angle
// (degrees) into durations (ms) at a reference speed.
//
// REF_SPEED: the PWM value at which MS_PER_CM and MS_PER_DEG were measured.
//
// MS_PER_CM: milliseconds of forward driving = 1 cm at REF_SPEED
// MS_PER_DEG: milliseconds of in-place rotation = 1 degree at REF_SPEED
//
// Procedure:
//   1. Drive forward at speed REF_SPEED for 3 seconds; measure actual distance (e.g., 45 cm)
//      → MS_PER_CM = 3000 / 45 = 66.7 ms/cm
//   2. Rotate in place at REF_SPEED for 360 degrees; measure actual time (e.g., 1600 ms)
//      → MS_PER_DEG = 1600 / 360 = 4.44 ms/degree
//
// movement.h scales these by the ratio of desired speed to REF_SPEED when
// moving at a different speed.
// =========================================================================
#define REF_SPEED     180     // reference speed (PWM 0–255) used for calibration
#define MS_PER_CM     24.0f   // milliseconds per cm forward at REF_SPEED (adjust after test drive)
#define MS_PER_DEG    3.25f    // milliseconds per degree rotation at REF_SPEED (adjust after test drive)

// =========================================================================
// Sensor Offset from Wheel Axle Centre
// =========================================================================
// The HC-SR04 transducer is mounted off-centre from the steering/wheel axle.
// These offsets (in cm) are critical for:
//   - sensing.h: to project sensor readings into world coordinates
//   - mapping.h: to know where the sensor was when taking a reading
//   - localization correction: to map sensor distance back to axle position
//
// Coordinate frame (viewed from above, car facing forward):
//   +X = right of the axle centre
//   +Y = forward (away from rear axle)
//
// Measured by subtracting the measured distance from sensor to wall from ruler measured distance from wall to axle
//    Measured as 15.5 - 6.5 = 9cm for forward mount
// =========================================================================

// Side-mount (stage 1): ultrasonic faces perpendicular to car axis
#define SENSOR_SIDE_OFFSET_X    6.5f   // left-right offset (cm); adjust if mounted off-centre. known distance - measured distance 15.5-9 = 6.5
#define SENSOR_SIDE_OFFSET_Y    7.5f   // forward offset (cm); adjust if not on axle line

// Front-mount (stages 2 & 3): ultrasonic faces forward
#define SENSOR_FRONT_OFFSET_X   0.0f   // left-right offset (cm) 0 = centred
#define SENSOR_FRONT_OFFSET_Y   9.0f   // forward offset (cm) from axle

// =========================================================================
// Active Sensor Configuration for current stage
// =========================================================================
// Set SENSOR_MOUNT to MOUNT_FRONT (stages 2 & 3) or MOUNT_SIDE (stage 1) to select the applicable offsets
// =========================================================================
#define MOUNT_FRONT   0
#define MOUNT_SIDE    1

#define SENSOR_MOUNT  MOUNT_SIDE // current mount

#if SENSOR_MOUNT == MOUNT_FRONT
  #define SENSOR_OFFSET_X  SENSOR_FRONT_OFFSET_X
  #define SENSOR_OFFSET_Y  SENSOR_FRONT_OFFSET_Y
#else  // MOUNT_SIDE
  #define SENSOR_OFFSET_X  SENSOR_SIDE_OFFSET_X
  #define SENSOR_OFFSET_Y  SENSOR_SIDE_OFFSET_Y
#endif

// =========================================================================
// Arena Dimensions (from task spec)
// =========================================================================
#define COURSE_W_CM   300    // 3m arena width
#define COURSE_H_CM   600    // 6m arena length
#define EVASION_W_CM  150    // 1.5m stage 1 box width (lateral)
#define EVASION_H_CM  50     // 0.5m stage 1 box length (front-to-back)

// =========================================================================
// Safety and Behaviour Thresholds
// =========================================================================

// Emergency stop distance: hard-stop if anything is closer than this.
// Should be larger than HC-SR04 minimum range (~2cm) and smaller than
// desired safety margin (~15cm).
// note readings can be unreliable for < 5cm
#define EMERGENCY_STOP_CM   5

// Stage 1 debris detection range: trigger evasion if reading is < this.
// Should be large enough to give reaction time (~50–70cm from 2m away).
#define INCOMING_THRESHOLD  65   // cm (stage 1 debris detection)

// Wall-follow target distance: maintain this distance from walls in stages 2 & 3.
#define WALL_FOLLOW_CM      15   // cm

// =========================================================================
// Calibration Procedure
// =========================================================================
// 
// 1. MOTOR TRIM (fix drift)
// ────────────────────────────
//    - Clear a straight 2–3 meter path
//    - Upload a simple sketch that calls move_forward(REF_SPEED, ~2000ms)
//    - Measure how much the car drifts left or right
//    - If drift is left: car turned left → right motor is faster
//      → decrease TRIM_R or increase TRIM_L
//    - If drift is right: car turned right → left motor is faster
//      → increase TRIM_R or decrease TRIM_L
//    - Repeat until drift is < 5 cm over 2 m
//
// 2. MS_PER_CM (time-to-distance scale)
// ──────────────────────────────────────
//    - Set up a clear ~1.5 m straight path
//    - Call move_forward(REF_SPEED, 3000) in a sketch
//    - Measure the actual distance the car travels (e.g., 45 cm)
//    - MS_PER_CM = 3000 / 45 = 66.7
//    - Update #define MS_PER_CM and retest until accurate
//
// 3. MS_PER_DEG (time-to-angle scale)
// ───────────────────────────────────
//    - Mark a spot; place car on it facing a fixed direction (e.g., a wall)
//    - Call turn_degrees(REF_SPEED, 360.0) in a sketch
//    - Car should complete a full rotation and return to start heading
//    - Measure actual rotation time needed (e.g., 1600 ms for 360°)
//    - MS_PER_DEG = 1600 / 360 = 4.44
//    - Update #define MS_PER_DEG and retest
//
// 4. SENSOR OFFSET (physical measurement)
// ────────────────────────────────────
//    - For each mount config (side and front):
//      - Place sensor on the car in that orientation
//      - Measure distance (X) from wheel axle centre to sensor transducer
//        (positive = to the right)
//      - Measure distance (Y) from wheel axle to sensor transducer
//        (positive = forward)
//      - Update SENSOR_SIDE_OFFSET_X/Y or SENSOR_FRONT_OFFSET_X/Y
//
// 5. SENSOR SANITY CHECK (verify offset-corrected readings)
// ──────────────────────────────────────────────────────────
//    - Place car at a known distance from a wall (e.g., 50 cm from axle to wall)
//    - Upload a sensor test sketch: read_distance_world() should report ~50 cm
//    - If off by more than a couple cm, double-check offset measurements
//
// =========================================================================

#endif