// Desktop tests for mapping.h
// Compile: g++ -std=c++11 -o test_mapping tests/test_mapping.cpp -lm

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#include "../lib/mapping.h"

static int tests_passed = 0;
static int tests_failed = 0;

#define CHECK(cond, msg) do { \
  if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
  else      { tests_failed++; printf("  FAIL: %s\n", msg); } \
} while(0)

// ------------------------------------------------------------------ helpers

static bool all_cells(uint8_t expected) {
  for (int y = 0; y < MAP_H; y++)
    for (int x = 0; x < MAP_W; x++)
      if (_map_get(x, y) != expected) return false;
  return true;
}

// count cells with a given value
static int count_cells(uint8_t val) {
  int n = 0;
  for (int y = 0; y < MAP_H; y++)
    for (int x = 0; x < MAP_W; x++)
      if (_map_get(x, y) == val) n++;
  return n;
}

// ------------------------------------------------------------------ tests

void test_init_clears_grid() {
  printf("Test: map_init sets every cell to UNKNOWN\n");
  // Dirty the grid first
  _map_set(0, 0, MAP_WALL);
  _map_set(30, 60, MAP_FREE);
  map_init();
  CHECK(all_cells(MAP_UNKNOWN), "all cells UNKNOWN after map_init");
  CHECK(_map_nopen == 0, "opening count reset to 0 by map_init");
}

void test_get_set_roundtrip() {
  printf("Test: map_get / map_set round-trip for all cell values\n");
  map_init();
  _map_set(5,  10, MAP_UNKNOWN); CHECK(_map_get(5,  10) == MAP_UNKNOWN, "set/get UNKNOWN");
  _map_set(5,  10, MAP_FREE);    CHECK(_map_get(5,  10) == MAP_FREE,    "set/get FREE");
  _map_set(5,  10, MAP_WALL);    CHECK(_map_get(5,  10) == MAP_WALL,    "set/get WALL");
  _map_set(5,  10, MAP_DYNAMIC); CHECK(_map_get(5,  10) == MAP_DYNAMIC, "set/get DYNAMIC");

  // Confirm neighbours are unaffected
  CHECK(_map_get(4, 10) == MAP_UNKNOWN, "left neighbour unaffected");
  CHECK(_map_get(6, 10) == MAP_UNKNOWN, "right neighbour unaffected");
  CHECK(_map_get(5,  9) == MAP_UNKNOWN, "bottom neighbour unaffected");
  CHECK(_map_get(5, 11) == MAP_UNKNOWN, "top neighbour unaffected");
}

void test_oob_get_returns_wall() {
  printf("Test: out-of-bounds _map_get returns WALL\n");
  map_init();
  CHECK(_map_get(-1,  0)     == MAP_WALL, "negative x → WALL");
  CHECK(_map_get( 0, -1)     == MAP_WALL, "negative y → WALL");
  CHECK(_map_get(MAP_W, 0)   == MAP_WALL, "x == MAP_W → WALL");
  CHECK(_map_get(0, MAP_H)   == MAP_WALL, "y == MAP_H → WALL");
  CHECK(_map_get(MAP_W+10, MAP_H+10) == MAP_WALL, "far OOB → WALL");
}

void test_oob_set_is_noop() {
  printf("Test: out-of-bounds _map_set does not corrupt grid\n");
  map_init();
  int before = count_cells(MAP_UNKNOWN);
  _map_set(-1,  0, MAP_WALL);
  _map_set( 0, -1, MAP_WALL);
  _map_set(MAP_W, 0, MAP_WALL);
  _map_set(0, MAP_H, MAP_WALL);
  CHECK(count_cells(MAP_UNKNOWN) == before, "OOB writes don't change grid");
}

void test_mark_boundary_fills_edges() {
  printf("Test: map_mark_boundary fills all four outer edges as WALL\n");
  map_init();
  map_mark_boundary();

  bool south_ok = true, north_ok = true, west_ok = true, east_ok = true;
  for (int x = 0; x < MAP_W; x++) {
    if (_map_get(x, 0)       != MAP_WALL) south_ok = false;
    if (_map_get(x, MAP_H-1) != MAP_WALL) north_ok = false;
  }
  for (int y = 0; y < MAP_H; y++) {
    if (_map_get(0,       y) != MAP_WALL) west_ok = false;
    if (_map_get(MAP_W-1, y) != MAP_WALL) east_ok = false;
  }
  CHECK(south_ok, "south edge (y=0) fully WALL");
  CHECK(north_ok, "north edge (y=MAP_H-1) fully WALL");
  CHECK(west_ok,  "west edge  (x=0) fully WALL");
  CHECK(east_ok,  "east edge  (x=MAP_W-1) fully WALL");

  // Interior cells untouched
  CHECK(_map_get(1, 1) == MAP_UNKNOWN, "interior cell (1,1) still UNKNOWN");
  CHECK(_map_get(30, 60) == MAP_UNKNOWN, "interior cell (30,60) still UNKNOWN");
}

void test_mark_boundary_interior_is_unknown() {
  printf("Test: map_mark_boundary leaves interior cells UNKNOWN\n");
  map_init();
  map_mark_boundary();
  bool interior_clean = true;
  for (int y = 1; y < MAP_H-1; y++)
    for (int x = 1; x < MAP_W-1; x++)
      if (_map_get(x, y) != MAP_UNKNOWN) interior_clean = false;
  CHECK(interior_clean, "all interior cells remain UNKNOWN");
}

void test_add_opening_clears_boundary_cells() {
  printf("Test: map_add_opening clears boundary cells back to UNKNOWN\n");
  map_init();
  map_mark_boundary();

  // Opening from x=100cm to x=200cm on south wall (y=0)
  map_add_opening(100, 0, 200, 0);

  // Grid cells: x=20..40, y=0 should be UNKNOWN
  bool cleared = true;
  for (int gx = 20; gx <= 40; gx++)
    if (_map_get(gx, 0) != MAP_UNKNOWN) cleared = false;
  CHECK(cleared, "opening cells on south wall cleared to UNKNOWN");

  // Adjacent boundary cells outside the gap remain WALL
  CHECK(_map_get(10, 0) == MAP_WALL, "cell left of opening still WALL");
  CHECK(_map_get(50, 0) == MAP_WALL, "cell right of opening still WALL");
}

void test_add_opening_registers_dead_zone() {
  printf("Test: map_add_opening registers a dead zone\n");
  map_init();
  map_add_opening(100, 0, 200, 0);

  // Centre of opening
  CHECK(_map_in_opening(150, 0),  "centre of opening is in dead zone");
  // Buffer zone (one cell outside each end)
  CHECK(_map_in_opening(95, 0),   "within buffer on left of opening");
  CHECK(_map_in_opening(205, 0),  "within buffer on right of opening");
  // Clearly outside
  CHECK(!_map_in_opening(50, 0),  "far left of opening NOT in dead zone");
  CHECK(!_map_in_opening(260, 0), "far right of opening NOT in dead zone");
  // Wrong wall
  CHECK(!_map_in_opening(150, 300), "midcourse point NOT in dead zone");
}

void test_add_wall_horizontal() {
  printf("Test: map_add_wall marks correct cells for horizontal wall\n");
  map_init();
  // Horizontal wall y=100cm, x=50..150cm
  map_add_wall(50, 100, 150, 100);

  // Grid row gy=20 (100/5), gx=10..30 should be WALL
  bool all_wall = true;
  for (int gx = 10; gx <= 30; gx++)
    if (_map_get(gx, 20) != MAP_WALL) all_wall = false;
  CHECK(all_wall, "horizontal wall: all expected cells are WALL");
  CHECK(_map_get(9,  20) == MAP_UNKNOWN, "cell before wall start is UNKNOWN");
  CHECK(_map_get(31, 20) == MAP_UNKNOWN, "cell after wall end is UNKNOWN");
  CHECK(_map_get(20, 19) == MAP_UNKNOWN, "cell south of wall is UNKNOWN");
  CHECK(_map_get(20, 21) == MAP_UNKNOWN, "cell north of wall is UNKNOWN");
}

void test_add_wall_vertical() {
  printf("Test: map_add_wall marks correct cells for vertical wall\n");
  map_init();
  // Vertical wall x=150cm, y=50..200cm
  map_add_wall(150, 50, 150, 200);

  bool all_wall = true;
  for (int gy = 10; gy <= 40; gy++)
    if (_map_get(30, gy) != MAP_WALL) all_wall = false;
  CHECK(all_wall, "vertical wall: all expected cells are WALL");
  CHECK(_map_get(30,  9) == MAP_UNKNOWN, "cell below wall start is UNKNOWN");
  CHECK(_map_get(30, 41) == MAP_UNKNOWN, "cell above wall end is UNKNOWN");
}

void test_add_wall_single_cell() {
  printf("Test: map_add_wall with endpoints in the same cell marks that cell\n");
  map_init();
  // Wall only 2cm long — both endpoints in same 5cm cell
  map_add_wall(52, 102, 54, 102);
  CHECK(_map_get(10, 20) == MAP_WALL, "same-cell wall marks that cell WALL");
}

void test_sensor_range_too_close() {
  printf("Test: readings below MAP_SENSOR_MIN_CM (%g cm) are ignored\n",
         (double)MAP_SENSOR_MIN_CM);
  map_init();
  Pose p = {150, 300, 0};
  float dists[]  = {0.0f, 1.0f, 2.9f};   // all below 3 cm
  float angles[] = {0.0f, 0.0f, 0.0f};
  map_update_from_scan(p, dists, angles, 3);
  CHECK(all_cells(MAP_UNKNOWN), "sub-minimum readings produce no map changes");
}

void test_sensor_range_too_far() {
  printf("Test: readings above MAP_SENSOR_MAX_CM (%g cm) are ignored\n",
         (double)MAP_SENSOR_MAX_CM);
  map_init();
  Pose p = {150, 300, 0};
  float dists[]  = {350.1f, 400.0f, 999.0f};   // all above 350 cm
  float angles[] = {0.0f,   0.0f,   0.0f};
  map_update_from_scan(p, dists, angles, 3);
  CHECK(all_cells(MAP_UNKNOWN), "over-maximum readings produce no map changes");
}

void test_sensor_range_boundary_values() {
  printf("Test: readings exactly at range limits are handled correctly\n");
  map_init();
  Pose p = {150, 50, 0};   // near south of course
  float dists[]  = {MAP_SENSOR_MIN_CM, MAP_SENSOR_MAX_CM};
  float angles[] = {0.0f, 0.0f};

  // MIN is valid — should mark a WALL at y=50+3=53cm
  // MAX is valid — endpoint at y=50+350=400cm → inside grid, marks WALL
  map_update_from_scan(p, dists, angles, 2);

  int walls = count_cells(MAP_WALL);
  CHECK(walls >= 1, "at least one WALL marked for readings at range limits");
}

void test_scan_marks_free_and_wall() {
  printf("Test: map_update_from_scan marks FREE along ray and WALL at endpoint\n");
  map_init();
  map_set_sensor_offset(0, 0);

  // Robot at (150, 50), heading 0 (facing +Y). Reading: 100 cm.
  // Sensor at (150, 50). Wall endpoint at (150, 150).
  Pose p = {150, 50, 0};
  float dists[]  = {100.0f};
  float angles[] = {0.0f};
  map_update_from_scan(p, dists, angles, 1);

  // Cells from (150,50) to (150,140) should be FREE (gx=30, gy=10..28)
  bool free_ok = true;
  for (int gy = 10; gy <= 27; gy++)   // up to but not including endpoint cell
    if (_map_get(30, gy) != MAP_FREE) free_ok = false;
  CHECK(free_ok, "cells along ray are FREE");

  // Endpoint cell (150,150) → gx=30, gy=30 should be WALL
  CHECK(_map_get(30, 30) == MAP_WALL, "endpoint cell is WALL");
}

void test_scan_ignores_opening_endpoint() {
  printf("Test: ray endpoint in dead zone does NOT mark WALL\n");
  map_init();
  map_mark_boundary();
  // Opening on south wall: x=100..200cm, y=0
  map_add_opening(100, 0, 200, 0);
  map_set_sensor_offset(0, 0);

  // Robot at (150, 20), heading 180° (facing south toward opening).
  // Reading 20 cm → endpoint at (150, 0) — dead centre of opening.
  Pose p = {150, 20, 180};
  float dists[]  = {20.0f};
  float angles[] = {0.0f};
  map_update_from_scan(p, dists, angles, 1);

  // Opening cells should remain UNKNOWN, not WALL
  CHECK(_map_get(30, 0) == MAP_UNKNOWN,
        "opening cell not marked WALL by scan toward gap");
}

void test_scan_no_overwrite_wall_with_free() {
  printf("Test: FREE ray trace does not overwrite existing WALL cells\n");
  map_init();
  map_set_sensor_offset(0, 0);

  // Pre-place a wall at gx=30, gy=20 (x=150, y=100)
  _map_set(30, 20, MAP_WALL);

  // Robot at (150, 50) heading 0, reading 150cm → ray passes through (150,100)
  Pose p = {150, 50, 0};
  float dists[]  = {150.0f};
  float angles[] = {0.0f};
  map_update_from_scan(p, dists, angles, 1);

  CHECK(_map_get(30, 20) == MAP_WALL,
        "pre-existing WALL not overwritten by FREE ray trace");
}

void test_scan_dynamic_not_overwritten() {
  printf("Test: DYNAMIC cell at endpoint is not overwritten with WALL\n");
  map_init();
  map_set_sensor_offset(0, 0);

  // Mark endpoint cell as DYNAMIC before the scan
  Pose p = {150, 50, 0};
  int ep_gx = 30, ep_gy = 30;  // endpoint at (150, 150)
  _map_set(ep_gx, ep_gy, MAP_DYNAMIC);

  float dists[]  = {100.0f};
  float angles[] = {0.0f};
  map_update_from_scan(p, dists, angles, 1);

  CHECK(_map_get(ep_gx, ep_gy) == MAP_DYNAMIC,
        "DYNAMIC cell at endpoint not overwritten with WALL");
}

void test_get_grid_returns_data() {
  printf("Test: map_get_grid returns non-null pointer to grid data\n");
  map_init();
  uint8_t* g = map_get_grid();
  CHECK(g != NULL, "map_get_grid is non-null");
  CHECK(g == _map_grid, "map_get_grid points to internal grid");

  // Verify it reflects live changes
  _map_set(1, 1, MAP_WALL);
  CHECK(map_get(1, 1) == MAP_WALL, "map_get reflects _map_set");
}

// ------------------------------------------------------------------ main

int main() {
  printf("=== Mapping Tests ===\n\n");

  test_init_clears_grid();
  test_get_set_roundtrip();
  test_oob_get_returns_wall();
  test_oob_set_is_noop();
  test_mark_boundary_fills_edges();
  test_mark_boundary_interior_is_unknown();
  test_add_opening_clears_boundary_cells();
  test_add_opening_registers_dead_zone();
  test_add_wall_horizontal();
  test_add_wall_vertical();
  test_add_wall_single_cell();
  test_sensor_range_too_close();
  test_sensor_range_too_far();
  test_sensor_range_boundary_values();
  test_scan_marks_free_and_wall();
  test_scan_ignores_opening_endpoint();
  test_scan_no_overwrite_wall_with_free();
  test_scan_dynamic_not_overwritten();
  test_get_grid_returns_data();

  printf("\n=== Results: %d passed, %d failed ===\n",
         tests_passed, tests_failed);
  return tests_failed > 0 ? 1 : 0;
}
