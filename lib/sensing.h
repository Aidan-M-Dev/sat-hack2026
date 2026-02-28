#ifndef SENSING_H
#define SENSING_H

/*
 * lib/sensing.h
 *
 * Ultrasonic sensor helpers for the SatHack2026 RC car.
 *
 * Provides filtered distance readings, simple obstacle checks, sweep
 * scanning and conversion of sensor measurements into world coordinates.
 * All routines account for the fact that the HC-SR04 transducer is not
 * mounted at the wheel axle centre; the X/Y offset is applied when
 * computing world distances.
 *
 * This implementation follows the architecture described in
 * architecture.md (see 1b. Sensing section).
 */

#include "ADCS.h"     // provides GetDistance()
#include <math.h>

// ---------------------------------------------------------------------------
// Sensor offset configuration
// ---------------------------------------------------------------------------
// Distance in centimetres from the wheel axle centre to the sensor
//   offset_x: positive to the right of the axle centre (looking forward)
//   offset_y: positive forward of the axle centre
// These should be calibrated per stage (side mount vs forward mount).
// ---------------------------------------------------------------------------
static float sensor_offset_x = 0.0f;
static float sensor_offset_y = 0.0f;

/**
 * set_sensor_offset(x, y)
 * @param x  left/right offset (cm, positive = right)
 * @param y  forward/back offset (cm, positive = forward)
 */
inline void set_sensor_offset(float x, float y) {
    sensor_offset_x = x;
    sensor_offset_y = y;
}

/**
 * convenience initialiser for stage-specific mounts
 * stage1 = sideways (evasion), stage2/3 = forward (navigation)
 */
inline void init_sensing(int stage) {
    if (stage == 1) {
        // side mount; assume centred on axle line by default
        sensor_offset_x = 0.0f;
        sensor_offset_y = 0.0f;
    } else {
        // forward mount; typical ~5cm ahead of axle
        sensor_offset_x = 0.0f;
        sensor_offset_y = 5.0f;
    }
}

static float median_of_three(float a, float b, float c) {
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

/**
 * read_distance()
 * Median-filtered raw sensor distance (cm).
 */
inline float read_distance() {
    float a = GetDistance();
    float b = GetDistance();
    float c = GetDistance();
    return median_of_three(a, b, c);
}

/**
 * read_distance_world()
 * Distance (cm) from the wheel axle centre to the nearest obstacle.
 * The raw sensor reading is adjusted to account for the X/Y offset.
 */
inline float read_distance_world() {
    float d = read_distance();
    // do not adjust large "no echo" values
    if (d > 300.0f) return d;
    if (d < 0.0f) return 400; // sometimes returns negative if distance too large

    // effective offset magnitude in direction of measurement
    float offset_mag = sqrt(sensor_offset_x*sensor_offset_x +
                            sensor_offset_y*sensor_offset_y);
    if (d <= offset_mag) return 0.0f;
    return sqrt(d*d - offset_mag*offset_mag);
}

/**
 * obstacle_in_range(threshold_cm)
 * Quick boolean check against a threshold.
 */
inline bool obstacle_in_range(float threshold_cm) {
    float d = read_distance();
    return (d > 0 && d < threshold_cm);
}

/**
 * scan_sweep(dists, angles, steps, total_deg)
 *
 * Rotate the vehicle through total_deg in `steps` increments taking a
 * distance reading at each step. The arrays `dists` and `angles` must be
 * large enough to hold `steps` entries.  After the call the car should
 * be back at the starting heading.
 */
inline void scan_sweep(float* dists, float* angles, int steps, float total_deg) {
    if (steps <= 0 || !dists || !angles) return;
    float delta = total_deg / steps;
    for (int i = 0; i < steps; ++i) {
        // caller should rotate here by delta degrees
        dists[i] = read_distance();
        angles[i] = i * delta;
        delay(50);
    }
    // caller should rotate back by -total_deg after loop
}

/**
 * sensor_to_world(sensor_dist, car_heading, wx, wy)
 *
 * Converts a sensor reading (cm) and the car's heading (degrees, 0=North,
 * positive clockwise) into world-frame X/Y coordinates (cm).  The origin is
 * assumed to be the axle centre at the time of the reading.  The sensor's
 * offset is applied prior to rotation.
 */
inline void sensor_to_world(float sensor_dist, float car_heading,
                            float* wx, float* wy) {
    if (!wx || !wy) return;
    float heading_rad = car_heading * (M_PI / 180.0f);

    // point in sensor frame (sensor facing +Y)
    float sx = 0.0f;
    float sy = sensor_dist;

    // shift to axle frame
    float ax = sx - sensor_offset_x;
    float ay = sy - sensor_offset_y;

    float cosh = cos(heading_rad);
    float sinh = sin(heading_rad);
    *wx = ax * cosh - ay * sinh;
    *wy = ax * sinh + ay * cosh;
}

#endif
