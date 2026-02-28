# Architecture

## Context

SatHack 2026 embedded systems task: program an RC car (Arduino Nano, 2 DC motors, 1 HC-SR04 ultrasonic sensor) to autonomously navigate 3 obstacle course stages representing space debris scenarios.

| Stage | Name | Arena | Obstacles | Time |
|-------|------|-------|-----------|------|
| 1 | Space Invader Evasion | 1.5m x 0.5m box | Debris launched at car every ~5s | Survive 30s |
| 2 | Static Debris Field | 3m x 6m course | Cardboard walls (fixed) | Reach end, no collisions |
| 3 | Moving Debris Field | 3m x 6m course | Stage 2 walls + moving RC cars + rolling debris from ramp | Reach end, no collisions |

**Scoring**: Stage 1 = seconds survived (max 50 pts). Stages 2&3 = checkpoints * 10 + 50 bonus for completion (max 100 pts each). Total max 250 pts.

---

## Layer Diagram

```
 ┌──────────────────────────────────────────────────────┐
 │               Stage Implementations                  │
 │    Stage 1 (evasion) │ Stage 2 (static) │ Stage 3    │
 ├──────────────────────────────────────────────────────┤
 │                  Base Libraries                      │
 │  Movement │ Sensing │ Mapping │ Localize │ Obstacle  │
 ├──────────────────────────────────────────────────────┤
 │             Hardware Abstraction (HAL)               │
 │           Motors │ Ultrasonic │ Pins │ Timer         │
 └──────────────────────────────────────────────────────┘
```

---

## Layer 0: Hardware Abstraction (`lib/hal.h`)

Thin wrappers over raw hardware. Nothing else touches pins directly.

```c
void hal_init();                          // pinMode setup for all pins
void hal_motor_left(int fwd, int rev);    // PWM to motor 1 (pins 5, 6)
void hal_motor_right(int fwd, int rev);   // PWM to motor 2 (pins 9, 10)
float hal_ultrasonic_cm();                // single HC-SR04 reading in cm
unsigned long hal_millis();               // wrapper around millis()
```

Pin map:

| Function        | Pin |
|-----------------|-----|
| Motor L fwd     | 5   |
| Motor L rev     | 6   |
| Motor R fwd     | 9   |
| Motor R rev     | 10  |
| Ultrasonic trig | A4  |
| Ultrasonic echo | A5  |

---

## Layer 1: Base Libraries

### 1a. Movement (`lib/movement.h`)

Motion primitives with drift compensation. Both blocking and non-blocking variants.

```c
// Configuration
void  set_trim(float left_factor, float right_factor);

// Blocking (return when done)
void  move_forward(int speed, unsigned long ms);
void  move_backward(int speed, unsigned long ms);
void  turn_left(int speed, unsigned long ms);
void  turn_right(int speed, unsigned long ms);
void  stop();

// Calibrated blocking
void  move_forward_cm(int speed, float cm);
void  turn_degrees(int speed, float degrees);  // positive=left, negative=right

// Non-blocking (for reactive control in stage 1)
void  drive(int left_speed, int right_speed);  // raw differential, trim applied
```

Calibration constants (experimentally determined, stored in `lib/calibration.h`):
- `MS_PER_CM` — ms of forward driving per cm at reference speed
- `MS_PER_DEG` — ms of in-place rotation per degree at reference speed
- `TRIM_L`, `TRIM_R` — per-motor speed multipliers to correct drift

### 1b. Sensing (`lib/sensing.h`)

Filtered readings and sweep scans using the single ultrasonic sensor.

```c
float read_distance();                          // median-of-3 filtered reading (cm)
float read_distance_world();                    // reading adjusted for sensor offset from axle centre
bool  obstacle_in_range(float threshold_cm);    // quick boolean check
void  scan_sweep(float* dists, float* angles, int steps, float total_deg);
void  sensor_to_world(float sensor_dist, float car_heading, float* wx, float* wy);
```

`read_distance_world()` returns the distance from the **wheel axle centre** (not the sensor) to the obstacle, accounting for the sensor's X/Y offset. Raw `read_distance()` returns the distance from the sensor itself.

`sensor_to_world()` converts a sensor reading at a given car heading into world-frame X/Y coordinates of the detected point, applying the sensor offset. This is used by mapping to place walls correctly on the grid.

`scan_sweep` rotates the car through `total_deg` in `steps` increments, storing a distance and its corresponding angle at each step. The car ends facing its original heading. This is how we **produce wall/obstacle location data** from a single forward-facing sensor. All distances stored are raw sensor values; the offset is applied when projecting into the map.

For stage 1 the sensor is mounted sideways facing the debris field, so `read_distance()` directly detects incoming debris without needing to rotate. For stages 2 and 3 the sensor is remounted forward.

**Sensor mounting per stage**:

| Stage | Sensor faces | Why |
|-------|-------------|-----|
| 1 (evasion) | Sideways toward debris launch line | Direct detection of incoming debris, no rotation delay |
| 2 & 3 (navigation) | Forward | Detect walls and obstacles in path of travel |

The sensor will be physically moved between stages. Each mounting has its own offset calibration (see Calibration section).

**Sensor offset**: The HC-SR04 is not at the centre of the wheel axle. It sits offset in both X (left/right of centre) and Y (forward/back of axle). All sensing and mapping functions account for this offset so that distance readings are correctly projected into world coordinates. The offset values change between side-mount and front-mount configurations.

### 1c. Localization (`lib/localize.h`)

Dead-reckoning pose tracker. Movement functions update it automatically.

```c
struct Pose { float x; float y; float heading; };

void  loc_reset();
Pose  loc_get();
void  loc_update_forward(float cm);
void  loc_update_turn(float degrees);
void  loc_correct(float x, float y);  // manual correction from known landmark
```

Units: centimetres, degrees (0 = initial forward, positive = CCW).

Dead reckoning drifts over time. The course boundaries and known wall positions can be used to correct: e.g. if sensor reads 30 cm to a wall you know is at x=300, you know your x position. When using sensor distance for correction, always subtract the sensor offset to get the axle-centre position.

### 1d. Mapping (`lib/mapping.h`)

Grid-based occupancy map for stages 2 and 3. Records where walls and free space are.

```c
#define MAP_W    60   // cells wide  (60 * 5cm = 300cm = 3m course width)
#define MAP_H   120   // cells tall  (120 * 5cm = 600cm = 6m course length)
#define CELL_CM   5   // resolution

enum Cell : uint8_t { UNKNOWN = 0, FREE = 1, WALL = 2, DYNAMIC = 3 };

void  map_init();
Cell  map_get(int gx, int gy);
void  map_set(int gx, int gy, Cell val);
void  map_update_from_scan(Pose pose, float* dists, float* angles, int steps);
void  map_mark_boundary();              // pre-fill course edges as WALL
void  map_print_serial();              // dump over serial for debugging
```

`map_update_from_scan` ray-casts each scan reading from the car's current pose into the grid. The ray origin is the **sensor's world position** (car pose + sensor offset), not the axle centre. Cells along the ray become `FREE`, the endpoint cell becomes `WALL`.

**Memory**: 60 x 120 = 7200 cells. At 2 bits per cell = 1800 bytes. Arduino Nano has 2 KB SRAM. This is tight but feasible if we keep other stack/heap usage minimal. Can reduce to `CELL_CM=10` (900 bytes) if needed.

Since we know the course dimensions upfront, `map_mark_boundary()` pre-fills the outer edges as `WALL` so the car never tries to leave the arena.

### 1e. Obstacle Detection (`lib/obstacle.h`)

Wraps sensing for real-time obstacle classification. Distinguishes static from moving obstacles for stage 3.

```c
struct Obstacle {
  float distance;    // cm from car
  float angle;       // relative to car heading
  bool  approaching; // true if distance is shrinking (moving toward us)
};

Obstacle check_forward();                    // single quick check ahead
bool     is_path_clear(float distance_cm);   // can we drive forward this far?
bool     detect_moving(float* prev_dist, float* curr_dist, int n);  // compare two scans
```

For stage 3, `detect_moving` compares consecutive scan readings to flag obstacles whose distance is changing — these are moving debris that need different avoidance strategy (wait or dodge) vs static walls (navigate around).

---

## Layer 2: Stage Implementations

Each stage is a separate `.ino` sketch using the shared base libraries.

### Stage 1: Space Invader Evasion (`tasks/stage1_evasion/stage1_evasion.ino`)

**Goal**: Dodge debris launched every ~5s for 30 seconds. Stay within 1.5m x 0.5m box.

**Setup**: Sensor mounted sideways, facing the debris launch line (2m away). Car drives laterally (forward/backward relative to wheels) to dodge.

**Scanning**: The HC-SR04 has a ~15-degree cone — too narrow to cover the full 1.5m-wide debris field from 2m away. The car must **sweep scan left and right** while idle to search for incoming debris. This is done by making small oscillating turns (e.g. +/- 20 degrees) so the side-mounted sensor sweeps across the field. When a close reading is detected during the sweep, we know debris is incoming from that angle.

```
loop:
  // 1. Scan: oscillate heading to sweep sensor across debris field
  turn small increment (alternating left/right)
  dist = read_distance()

  if dist < INCOMING_THRESHOLD:
    // 2. Debris detected — determine dodge direction
    //    If debris is to our left (sensor angled left), dodge right, and vice versa
    //    "left/right" here means forward/backward on wheels since car is sideways
    straighten heading (face perpendicular to field)
    dodge in chosen direction

  // 3. Boundary check
  if loc_get().x near box edge:
    bias dodge direction toward centre

  // 4. After dodge, return to scanning
```

**Boundary tracking**: `loc_get()` tracks lateral position. Each drive updates the estimate. If near the 0.5m box edge on either side, force dodge direction toward centre. The sensor offset is accounted for when estimating lateral position.

**State machine**:
```
        ┌───────────┐
        │ SCANNING  │◄──── no detection / debris passed
        │ (sweep    │
        │  sensor)  │
        └─────┬─────┘
              │ dist < threshold
              ▼
        ┌───────────┐
        │ DODGING   │──── straighten + drive laterally
        │           │
        └─────┬─────┘
              │ dodge complete
              ▼
        ┌───────────┐
        │ RECENTRE  │──── bias back toward box centre
        └───────────┘
```

### Stage 2: Static Debris Field (`tasks/stage2_static/stage2_static.ino`)

**Goal**: Navigate 3m x 6m course with cardboard wall segments from start to end. No collisions.

**Approach**: Scan-and-advance. Build up wall data, plan short segments.

```
setup:
  map_init()
  map_mark_boundary()   // pre-fill 3m x 6m edges
  loc_reset()           // car starts at bottom centre

loop:
  // 1. Sense
  scan_sweep(dists, angles, 9, 180)     // 9 readings over 180 degrees
  map_update_from_scan(loc_get(), dists, angles, 9)

  // 2. Decide
  find best heading toward goal (forward/upward) that is FREE on map
  if direct path clear:
    advance forward a fixed step (e.g. 20 cm)
  else:
    pick left or right to go around wall
    turn and advance along wall edge until a forward gap appears

  // 3. Update position
  (localize auto-updates from movement calls)

  // 4. Check completion
  if loc_get().y >= COURSE_LENGTH - MARGIN:
    stop()  // reached end
```

**Wall-following fallback**: When a wall blocks the forward path, hug it on one side until a gap appears. This handles the maze-like segments in the course diagram.

**Corridor detection**: The course has long straight corridors with gaps between wall segments. After scanning, identify the corridor direction and drive along it. No need for full A* — greedy forward + wall-following is sufficient for this layout.

### Stage 3: Moving Debris Field (`tasks/stage3_dynamic/stage3_dynamic.ino`)

**Goal**: Same course as stage 2, but with moving RC cars and rolling debris.

**Approach**: Extends stage 2 logic with dynamic obstacle handling.

```
loop:
  // Same scan-and-plan as stage 2, plus:

  // Before each advance, quick forward check
  obs = check_forward()
  if obs.approaching:
    stop()
    wait_for_clear(TIMEOUT)  // let moving debris pass
    if still blocked after timeout:
      dodge sideways and retry

  // During advance, continuous monitoring
  while moving forward:
    if obstacle_in_range(EMERGENCY_STOP_CM):
      stop() immediately
      re-scan and re-plan
```

**Dynamic obstacle strategy**:

| Obstacle type | Detection | Response |
|---------------|-----------|----------|
| Static wall (from stage 2) | Same position across scans | Navigate around (map it) |
| Linear moving RC car | Distance changing between consecutive reads | Stop and wait for it to pass |
| Looping RC car | Periodic pattern in readings | Time the gap, then advance |
| Rolling debris from ramp | Sudden close reading from right side | Emergency stop, wait for it to roll past |

The key insight: **moving obstacles are transient — you can wait them out.** Static walls must be navigated around. `detect_moving()` differentiates these by comparing two rapid readings.

---

## File Layout

```
sat-hack2026/
├── README.md
├── architecture.md
├── demo/                           # original demo code (unchanged)
│   ├── ADCS.h
│   └── demo.ino
├── lib/                            # shared base libraries (header-only)
│   ├── hal.h                       # hardware abstraction
│   ├── movement.h                  # motion primitives
│   ├── sensing.h                   # ultrasonic reads and sweep scans
│   ├── localize.h                  # dead-reckoning pose tracker
│   ├── mapping.h                   # grid occupancy map
│   ├── obstacle.h                  # obstacle detection and classification
│   └── calibration.h               # per-car tuning constants
└── tasks/
    ├── stage1_evasion/
    │   └── stage1_evasion.ino      # space invader dodge
    ├── stage2_static/
    │   └── stage2_static.ino       # static debris field navigation
    └── stage3_dynamic/
        └── stage3_dynamic.ino      # moving debris field navigation
```

Each `.ino` includes the shared libs via `#include "../../lib/hal.h"` etc. All libraries are header-only — standard Arduino IDE workflow, no build system changes.

---

## Data Flow

```
HC-SR04 Ultrasonic Sensor
         │
         ▼
    hal_ultrasonic_cm()                    ← HAL
         │
         ▼
    read_distance() / scan_sweep()         ← Sensing
         │
    ┌────┴──────────────┐
    │                   │
    ▼                   ▼
 map_update_          check_forward()      ← Mapping / Obstacle
 from_scan()          detect_moving()
    │                   │
    └────┬──────────────┘
         │
         ▼
    Stage logic                            ← Task (decide next action)
    (wall follow, dodge, wait, advance)
         │
         ▼
    drive() / move_forward_cm()            ← Movement
    turn_degrees()
         │
    ┌────┴────┐
    │         │
    ▼         ▼
 hal_motor  loc_update                     ← HAL / Localize
 _left/right  _forward/_turn
```

---

## Calibration (`lib/calibration.h`)

Tuned per car before competition. Run each test, measure, update constants.

```c
// Motor trim (fix drift)
#define TRIM_L        1.0
#define TRIM_R        1.15   // example: right motor slightly weaker

// Time-distance calibration at reference speed 180
#define REF_SPEED     180
#define MS_PER_CM     12.0   // example: 12ms of driving = 1cm forward
#define MS_PER_DEG    4.5    // example: 4.5ms of rotation = 1 degree

// Sensor offset from wheel axle centre (cm)
// Measured with tape measure on the physical car. These change when
// the sensor is remounted between stages.
//
//   Coordinate frame (from axle centre, looking forward):
//     +X = right
//     +Y = forward (away from axle, toward front of car)
//
//   Front-mount (stages 2 & 3): sensor faces forward, mounted ahead of axle
#define SENSOR_FRONT_OFFSET_X   0.0   // centred left-right (adjust if not)
#define SENSOR_FRONT_OFFSET_Y   7.5   // example: 7.5cm forward of axle
//
//   Side-mount (stage 1): sensor faces left/right, mounted on side of chassis
#define SENSOR_SIDE_OFFSET_X    6.0   // example: 6cm to the right of axle centre
#define SENSOR_SIDE_OFFSET_Y    2.0   // example: 2cm forward of axle

// Active sensor config — set before compiling for each stage
#define SENSOR_MOUNT  MOUNT_SIDE      // MOUNT_FRONT or MOUNT_SIDE
#define MOUNT_FRONT   0
#define MOUNT_SIDE    1

// Derived: active offsets (used by sensing/mapping/localize)
#if SENSOR_MOUNT == MOUNT_FRONT
  #define SENSOR_OFFSET_X  SENSOR_FRONT_OFFSET_X
  #define SENSOR_OFFSET_Y  SENSOR_FRONT_OFFSET_Y
#else
  #define SENSOR_OFFSET_X  SENSOR_SIDE_OFFSET_X
  #define SENSOR_OFFSET_Y  SENSOR_SIDE_OFFSET_Y
#endif

// Course dimensions (known from task sheet)
#define COURSE_W_CM   300    // 3m
#define COURSE_H_CM   600    // 6m
#define EVASION_W_CM  150    // 1.5m (stage 1 box width)
#define EVASION_H_CM  50     // 0.5m (stage 1 box depth)

// Safety thresholds
#define EMERGENCY_STOP_CM   8    // hard stop if anything this close
#define INCOMING_THRESHOLD  60   // stage 1: debris detection range
#define WALL_FOLLOW_CM      15   // target distance from walls in stages 2/3
```

**How sensor offset is used**:
- **Sensing**: `sensor_to_world()` adds the rotated offset to the car's axle-centre pose before projecting the distance reading into world coordinates.
- **Mapping**: `map_update_from_scan()` ray-casts from the sensor's world position, not the axle centre.
- **Localization correction**: when `loc_correct()` uses a known wall distance, it subtracts the offset to get the axle position.
- **Stage 1 boundary tracking**: lateral position of the axle centre = sensor position minus offset — prevents the car body from leaving the box even though the sensor is off-centre.

**Calibration procedure**:
1. **Trim** — drive straight 2m, measure drift, adjust `TRIM_L`/`TRIM_R`
2. **MS_PER_CM** — drive forward 3s at `REF_SPEED`, measure actual distance, divide
3. **MS_PER_DEG** — command 360-degree spin, measure actual rotation, divide
4. **Sensor offset** — with sensor mounted, measure X and Y distance from axle centre to the sensor face with a ruler. Record for both mount configs.
5. **Sensor sanity** — point at wall at known distance, verify `read_distance_world()` (offset-corrected) matches expected distance from axle centre to wall

---

## Constraints and Tradeoffs

| Constraint | Impact | Mitigation |
|------------|--------|------------|
| Single ultrasonic sensor | Can only see one direction at a time | Sweep scans; side-mount option for stage 1 |
| No wheel encoders | Dead reckoning is time-based, drifts | Correct using known wall positions and course boundaries |
| Arduino Nano 2 KB SRAM | Map must be compact | 2-bit cells, reduce resolution if needed |
| No FPU | Float math is slow | Use integer math in hot loops; floats ok in scan/plan phases |
| HC-SR04 ~15 degree cone | Poor angular resolution; can't cover wide area without rotating | Sweep-scan by oscillating heading; in stage 1, scan left/right to cover 1.5m field from 2m away |
| HC-SR04 min range ~2 cm | Blind spot up close | Use emergency stop threshold larger than min range |
| Transparent perspex in debris field | Ultrasonic may pass through or reflect unpredictably | Treat unexpected max-range readings with caution; slow approach speed |
| Course setup may change day-of | Can't hard-code wall positions | Build map dynamically from sensor data, only pre-fill outer boundary |
