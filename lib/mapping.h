#ifndef MAPPING_H
#define MAPPING_H

// Grid-based occupancy map for maze navigation (stages 2 & 3).
// Stores a 2-bit packed occupancy grid.  Integrates with localize.h for pose
// and exposes the raw grid pointer for particle_filter.h.
//
// Cell values (2 bits each):
//   0 = UNKNOWN  — not yet observed (treated as passable by particle filter)
//   1 = FREE     — confirmed clear
//   2 = WALL     — observed or pre-filled obstacle
//   3 = DYNAMIC  — moving obstacle (overrides WALL during ray-cast)
//
// Grid dimensions:
//   MAP_W = 60 cells × 5 cm = 300 cm (3 m course width)
//   MAP_H = 120 cells × 5 cm = 600 cm (6 m course length)
//
// Memory: 60×120 cells at 2 bits = 1800 bytes + 32 bytes for up to 4 openings.
// Timing: map_add_wall O(cells along segment); map_update_from_scan O(steps × dist/step).
//
// Setup (in setup()):
//
//   #include "localize.h"
//   #include "mapping.h"
//
//   map_init();
//   map_mark_boundary();             // pre-fill 3m × 6m outer walls
//
//   // Punch gaps where the course boundary is open (start/end openings, etc.)
//   // Call AFTER map_mark_boundary().  Clears those grid cells and registers
//   // a dead zone so sensor rays pointing through the gap are ignored.
//   map_add_opening(100, 0,  200, 0);   // start opening on south wall
//   map_add_opening(100, 600, 200, 600); // exit opening on north wall
//
//   map_set_sensor_offset(0.0, 7.5); // sensor 7.5 cm ahead of axle (front-mount)
//
//   // Optional: add known walls from course layout before driving.
//   map_add_wall(100, 200,  200, 200);  // horizontal wall, y=200cm
//   map_add_wall(150, 300,  150, 500);  // vertical wall
//
//   // Feed grid to particle filter:
//   pf_set_map(map_get_grid(), MAP_W, MAP_H, CELL_CM);
//
// Drive loop:
//
//   // After scan_sweep fills dists[] and angles[]:
//   map_update_from_scan(loc_get(), dists, angles, steps);
//
// Notes:
//   - Coordinate frame matches localize.h: +X = right, +Y = forward.
//   - HC-SR04 reliable range: MAP_SENSOR_MIN_CM (3 cm) to MAP_SENSOR_MAX_CM
//     (350 cm).  map_update_from_scan silently drops readings outside this
//     window — they are either echo noise (too close) or beyond the sensor's
//     effective range (too far).
//   - map_add_wall uses Bresenham rasterisation.  Walls are ~1 cm thick —
//     thinner than one CELL_CM=5 cm cell — so the centre-line trace is exact.
//   - map_add_opening clears those boundary cells back to UNKNOWN and registers
//     a dead zone.  Scan rays whose WALL endpoint falls inside a dead zone are
//     silently dropped — the robot will not mark phantom walls at openings.
//     FREE cells along the ray toward an opening are still marked correctly.
//   - Out-of-bounds WALL endpoints are already no-ops via _map_set bounds check.
//   - map_update_from_scan does not overwrite existing WALL cells with FREE,
//     preventing scan noise from erasing known walls.
//   - map_init() resets the grid AND all sensor/opening state so tests can
//     call it between cases without side effects.
//   - map_print_serial outputs a top-down ASCII view for serial debugging.

#include "localize.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

#define MAP_W   60    // grid columns (x)
#define MAP_H   120   // grid rows    (y)
#define CELL_CM  5    // centimetres per cell

#define MAP_UNKNOWN  0
#define MAP_FREE     1
#define MAP_WALL     2
#define MAP_DYNAMIC  3

// HC-SR04 reliable sensing window.
// Below MAP_SENSOR_MIN_CM: echo returns before the pulse has fully left the
//   transducer — reading is noise, not a real obstacle.
// Above MAP_SENSOR_MAX_CM: signal too weak to echo reliably; reading may be
//   a false max-range return or a missed echo.
// map_update_from_scan silently drops any reading outside [MIN, MAX].
#define MAP_SENSOR_MIN_CM  3.0f
#define MAP_SENSOR_MAX_CM  350.0f

// 2 bits per cell; 60*120 / 4 = 1800 bytes
static uint8_t _map_grid[MAP_W * MAP_H / 4];

static float _map_sensor_ox = 0.0f;  // sensor offset from axle, body +X (cm)
static float _map_sensor_oy = 0.0f;  // sensor offset from axle, body +Y (cm)

// Dead zones — open sections of the boundary where scan WALL endpoints are dropped.
#define MAP_MAX_OPENINGS 4
struct _MapOpening { float x0, y0, x1, y1; };  // world cm, axis-aligned bounding box
static struct _MapOpening _map_openings[MAP_MAX_OPENINGS];
static int _map_nopen = 0;

// ---------- internal 2-bit helpers ----------

static uint8_t _map_get(int gx, int gy) {
  if (gx < 0 || gx >= MAP_W || gy < 0 || gy >= MAP_H) return MAP_WALL;
  int idx   = gy * MAP_W + gx;
  int shift = (idx % 4) * 2;
  return (_map_grid[idx / 4] >> shift) & 0x03;
}

static void _map_set(int gx, int gy, uint8_t val) {
  if (gx < 0 || gx >= MAP_W || gy < 0 || gy >= MAP_H) return;
  int idx   = gy * MAP_W + gx;
  int shift = (idx % 4) * 2;
  _map_grid[idx / 4] = (_map_grid[idx / 4] & ~(0x03 << shift))
                       | ((val & 0x03) << shift);
}

// ---------- public API ----------

static void map_init() {
  memset(_map_grid, 0, sizeof(_map_grid)); // all UNKNOWN
  _map_sensor_ox = 0.0f;
  _map_sensor_oy = 0.0f;
  _map_nopen     = 0;
}

static uint8_t map_get(int gx, int gy) { return _map_get(gx, gy); }
static void    map_set(int gx, int gy, uint8_t val) { _map_set(gx, gy, val); }

// Sensor body-frame offset from wheel-axle centre (cm).
// Body frame: +X = right, +Y = forward.
static void map_set_sensor_offset(float ox, float oy) {
  _map_sensor_ox = ox;
  _map_sensor_oy = oy;
}

// Return raw packed grid pointer (pass to pf_set_map).
static uint8_t* map_get_grid() { return _map_grid; }

// ---------- opening / dead-zone helpers ----------

// Returns true if world point (wx, wy) falls within any registered opening zone.
// Uses a one-cell buffer so endpoints just outside the gap edge are also caught.
static bool _map_in_opening(float wx, float wy) {
  for (int i = 0; i < _map_nopen; i++) {
    float x0 = _map_openings[i].x0, x1 = _map_openings[i].x1;
    float y0 = _map_openings[i].y0, y1 = _map_openings[i].y1;
    if (x0 > x1) { float t = x0; x0 = x1; x1 = t; }
    if (y0 > y1) { float t = y0; y0 = y1; y1 = t; }
    // Expand by one cell to catch endpoints that land just past the gap
    x0 -= CELL_CM;  x1 += CELL_CM;
    y0 -= CELL_CM;  y1 += CELL_CM;
    if (wx >= x0 && wx <= x1 && wy >= y0 && wy <= y1) return true;
  }
  return false;
}

// ---------- public API ----------

// Pre-fill the course perimeter as WALL.
static void map_mark_boundary() {
  for (int x = 0; x < MAP_W; x++) {
    _map_set(x, 0,        MAP_WALL);
    _map_set(x, MAP_H-1,  MAP_WALL);
  }
  for (int y = 0; y < MAP_H; y++) {
    _map_set(0,       y, MAP_WALL);
    _map_set(MAP_W-1, y, MAP_WALL);
  }
}

// Register an open section of the boundary (gap, entry/exit point, etc.).
// Call after map_mark_boundary().  Does two things:
//   1. Clears (Bresenham) those grid cells back to UNKNOWN, undoing the
//      pre-filled WALL so the robot won't treat the gap as solid.
//   2. Registers a dead zone so that scan rays whose WALL endpoint falls
//      inside the gap region are silently ignored (no phantom WALL created).
// Up to MAP_MAX_OPENINGS (4) openings can be registered.
static void map_add_opening(float x1, float y1, float x2, float y2) {
  // Store dead zone
  if (_map_nopen < MAP_MAX_OPENINGS) {
    _map_openings[_map_nopen].x0 = x1;
    _map_openings[_map_nopen].y0 = y1;
    _map_openings[_map_nopen].x1 = x2;
    _map_openings[_map_nopen].y1 = y2;
    _map_nopen++;
  }

  // Clear grid cells along the segment (Bresenham, set to UNKNOWN)
  int gx  = (int)(x1 / CELL_CM),  gy  = (int)(y1 / CELL_CM);
  int gx2 = (int)(x2 / CELL_CM),  gy2 = (int)(y2 / CELL_CM);
  int dx  = gx2 - gx;  if (dx < 0) dx = -dx;
  int dy  = gy2 - gy;  if (dy < 0) dy = -dy;
  int sx  = gx < gx2 ? 1 : -1;
  int sy  = gy < gy2 ? 1 : -1;
  int err = dx - dy;

  while (1) {
    _map_set(gx, gy, MAP_UNKNOWN);
    if (gx == gx2 && gy == gy2) break;
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; gx += sx; }
    if (e2 <  dx) { err += dx; gy += sy; }
  }
}

// Rasterise a straight wall from world point (x1,y1) to (x2,y2) in cm.
// Uses Bresenham's line algorithm — marks every grid cell the segment crosses.
// Physical walls are ~1 cm thick, thinner than one 5 cm cell, so the
// centre-line trace covers exactly the right set of cells.
static void map_add_wall(float x1, float y1, float x2, float y2) {
  int gx = (int)(x1 / CELL_CM);
  int gy = (int)(y1 / CELL_CM);
  int gx1 = (int)(x2 / CELL_CM);
  int gy1 = (int)(y2 / CELL_CM);

  int dx  = gx1 - gx;  if (dx < 0) dx = -dx;
  int dy  = gy1 - gy;  if (dy < 0) dy = -dy;
  int sx  = gx < gx1 ? 1 : -1;
  int sy  = gy < gy1 ? 1 : -1;
  int err = dx - dy;

  while (1) {
    _map_set(gx, gy, MAP_WALL);
    if (gx == gx1 && gy == gy1) break;
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; gx += sx; }
    if (e2 <  dx) { err += dx; gy += sy; }
  }
}

// Mark cells along a single sensor ray.
// Cells before the endpoint → FREE (if not already WALL).
// Endpoint cell → WALL (if not DYNAMIC).
static void _map_mark_ray(float sx, float sy, float ray_rad, float dist_cm) {
  float ddx  = sinf(ray_rad);
  float ddy  = cosf(ray_rad);
  float step = CELL_CM * 0.5f;  // half-cell steps for sub-cell accuracy
  float t    = step;
  int last_gx = -1, last_gy = -1;

  // Free space along ray (stop one cell before endpoint to avoid clobbering)
  while (t < dist_cm - CELL_CM) {
    int gx = (int)((sx + ddx * t) / CELL_CM);
    int gy = (int)((sy + ddy * t) / CELL_CM);
    if (gx != last_gx || gy != last_gy) {
      if (_map_get(gx, gy) == MAP_UNKNOWN)
        _map_set(gx, gy, MAP_FREE);
      last_gx = gx;
      last_gy = gy;
    }
    t += step;
  }

  // Endpoint: mark WALL unless it lands in a registered opening dead zone
  // (readings pointing through a gap in the boundary are meaningless).
  float epx = sx + ddx * dist_cm;
  float epy = sy + ddy * dist_cm;
  if (!_map_in_opening(epx, epy)) {
    int wx = (int)(epx / CELL_CM);
    int wy = (int)(epy / CELL_CM);
    if (_map_get(wx, wy) != MAP_DYNAMIC)
      _map_set(wx, wy, MAP_WALL);
  }
}

// Update the map from a sweep-scan result.
// pose:   car axle-centre pose from loc_get().
// dists:  sensor distances in cm (from scan_sweep).
// angles: scan angles in degrees relative to car heading (from scan_sweep).
// steps:  number of readings.
//
// Ray origin is the sensor's world position (pose + rotated sensor offset).
// Readings outside [MAP_SENSOR_MIN_CM, MAP_SENSOR_MAX_CM] are silently dropped:
//   < 3 cm  — echo noise before pulse clears transducer face.
//   > 350 cm — signal too weak; reading is unreliable or a missed echo.
static void map_update_from_scan(Pose pose, float* dists, float* angles, int steps) {
  float h_rad = pose.heading * (float)M_PI / 180.0f;

  // Rotate sensor offset into world frame (body +X=right, +Y=forward)
  float sx = pose.x + _map_sensor_ox * cosf(h_rad) + _map_sensor_oy * sinf(h_rad);
  float sy = pose.y - _map_sensor_ox * sinf(h_rad) + _map_sensor_oy * cosf(h_rad);

  for (int i = 0; i < steps; i++) {
    if (dists[i] < MAP_SENSOR_MIN_CM || dists[i] > MAP_SENSOR_MAX_CM) continue;
    float ray_rad = h_rad + angles[i] * (float)M_PI / 180.0f;
    _map_mark_ray(sx, sy, ray_rad, dists[i]);
  }
}

// ASCII debug dump (top of course at top of output).
// '#'=WALL  '.'=FREE  'D'=DYNAMIC  '?'=UNKNOWN
#ifdef ARDUINO
static void map_print_serial() {
  for (int y = MAP_H - 1; y >= 0; y--) {
    for (int x = 0; x < MAP_W; x++) {
      uint8_t c = _map_get(x, y);
      Serial.print(c == MAP_WALL    ? '#' :
                   c == MAP_FREE    ? '.' :
                   c == MAP_DYNAMIC ? 'D' : '?');
    }
    Serial.println();
  }
}
#else
#include <stdio.h>
static void map_print_serial() {
  for (int y = MAP_H - 1; y >= 0; y--) {
    for (int x = 0; x < MAP_W; x++) {
      uint8_t c = _map_get(x, y);
      putchar(c == MAP_WALL    ? '#' :
              c == MAP_FREE    ? '.' :
              c == MAP_DYNAMIC ? 'D' : '?');
    }
    putchar('\n');
  }
}
#endif

#endif // MAPPING_H
