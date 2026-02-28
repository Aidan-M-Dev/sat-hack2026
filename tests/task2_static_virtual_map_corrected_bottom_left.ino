#include <Arduino.h>

// Task 2 static obstacle map (corrected orientation)
// --------------------------------------------------
// Each grid cell = 5 cm.
// Real walls are 10 cm thick = 2 cells thick.
// Dashed construction lines are NOT included as walls.
//
// FINAL coordinate system used by this sketch:
//   (0,0) = bottom-left corner of the arena
//   x     = left -> right
//   y     = bottom -> top
//
// Important:
// The original wall measurements were entered with the origin effectively at
// the TOP-RIGHT corner. To correct that without re-measuring everything,
// this sketch flips those wall rectangles by 180 degrees before drawing them.
// So the printed map now matches a true bottom-left origin.
//
// Map size from your measured drawing:
//   width  = 56 cells
//   height = 121 cells

#define MAP_W   56
#define MAP_H   121
#define CELL_CM 5

enum Cell : uint8_t { UNKNOWN = 0, FREE = 1, WALL = 2, DYNAMIC = 3 };

// 2 bits per cell -> 4 cells per byte
static uint8_t map_data[(MAP_W * MAP_H + 3) / 4];

static inline int cell_index(int gx, int gy) {
  return gy * MAP_W + gx;
}

Cell map_get(int gx, int gy) {
  if (gx < 0 || gx >= MAP_W || gy < 0 || gy >= MAP_H) return WALL;

  int idx = cell_index(gx, gy);
  int byteIndex = idx >> 2;          // divide by 4
  int shift = (idx & 0x03) << 1;     // (idx % 4) * 2
  return (Cell)((map_data[byteIndex] >> shift) & 0x03);
}

void map_set(int gx, int gy, Cell val) {
  if (gx < 0 || gx >= MAP_W || gy < 0 || gy >= MAP_H) return;

  int idx = cell_index(gx, gy);
  int byteIndex = idx >> 2;
  int shift = (idx & 0x03) << 1;

  map_data[byteIndex] &= ~(0x03 << shift);
  map_data[byteIndex] |= (((uint8_t)val & 0x03) << shift);
}

void map_init(Cell fill = FREE) {
  // Fill all cells with one repeated 2-bit value.
  uint8_t packed = ((uint8_t)fill & 0x03);
  packed |= packed << 2;
  packed |= packed << 4;

  for (unsigned int i = 0; i < sizeof(map_data); i++) {
    map_data[i] = packed;
  }
}

void fill_rect(int x0, int y0, int x1, int y1, Cell val = WALL) {
  if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
  if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }

  if (x1 < 0 || y1 < 0 || x0 >= MAP_W || y0 >= MAP_H) return;

  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 >= MAP_W) x1 = MAP_W - 1;
  if (y1 >= MAP_H) y1 = MAP_H - 1;

  for (int y = y0; y <= y1; y++) {
    for (int x = x0; x <= x1; x++) {
      map_set(x, y, val);
    }
  }
}

// Use this for wall measurements that were originally entered from a TOP-RIGHT origin.
// It rotates them into the correct BOTTOM-LEFT coordinate system.
void fill_rect_reversed(int x0, int y0, int x1, int y1, Cell val = WALL) {
  fill_rect(
    MAP_W - 1 - x1,
    MAP_H - 1 - y1,
    MAP_W - 1 - x0,
    MAP_H - 1 - y0,
    val
  );
}

void map_mark_boundary() {
  // Outer walls are 2 cells thick.
  fill_rect(0, 0, MAP_W - 1, 1, WALL);                     // bottom
  fill_rect(0, MAP_H - 2, MAP_W - 1, MAP_H - 1, WALL);     // top
  fill_rect(0, 0, 1, MAP_H - 1, WALL);                     // left
  fill_rect(MAP_W - 2, 0, MAP_W - 1, MAP_H - 1, WALL);     // right
}

void map_mark_static_walls() {
  // These wall rectangles came from your measured drawing,
  // but the original measurements were entered in reverse orientation.
  // So every wall is drawn with fill_rect_reversed(...).
  //
  // Because the drawing shows MIDLINES and the real walls are 2 cells thick,
  // each wall below is drawn as a 2-cell-thick rectangle.

  // 1) Top inner horizontal wall (original entry)
  fill_rect_reversed(16, 99, 37, 100, WALL);

  // 2) Upper centre vertical wall
  fill_rect_reversed(27, 80, 28, 91, WALL);

  // 3) Upper left horizontal wall
  fill_rect_reversed(0, 79, 10, 80, WALL);

  // 4) Upper right horizontal wall
  fill_rect_reversed(46, 79, 55, 80, WALL);

  // 5) Mid-left vertical wall
  fill_rect_reversed(7, 46, 8, 63, WALL);

  // 6) Mid-left horizontal wall
  fill_rect_reversed(8, 45, 33, 46, WALL);

  // 7) Mid-right short horizontal wall
  fill_rect_reversed(35, 57, 45, 58, WALL);

  // 8) Lower left horizontal wall
  fill_rect_reversed(0, 29, 21, 30, WALL);

  // 9) Lower right horizontal wall
  fill_rect_reversed(34, 29, 55, 30, WALL);

  // 10) Bottom-centre horizontal wall
  fill_rect_reversed(17, 14, 38, 15, WALL);
}

void map_print_serial() {
  Serial.println();
  Serial.println(F("Map printout (top row first). Coordinate origin is STILL (0,0) at bottom-left."));
  Serial.println(F("# = WALL, . = FREE, ? = UNKNOWN, D = DYNAMIC"));

  for (int y = MAP_H - 1; y >= 0; y--) {
    for (int x = 0; x < MAP_W; x++) {
      Cell c = map_get(x, y);
      char ch = '?';
      if (c == UNKNOWN) ch = '?';
      else if (c == FREE) ch = '.';
      else if (c == WALL) ch = '#';
      else if (c == DYNAMIC) ch = 'D';
      Serial.print(ch);
    }
    Serial.println();
  }

  Serial.println();
  Serial.println(F("Bottom-left map cell is (0,0)."));
  Serial.print(F("Map size: "));
  Serial.print(MAP_W);
  Serial.print(F(" x "));
  Serial.print(MAP_H);
  Serial.println(F(" cells"));
}

void setup() {
  Serial.begin(115200);
  delay(800);

  map_init(FREE);
  map_mark_boundary();
  map_mark_static_walls();
  map_print_serial();
}

void loop() {
  // Nothing here yet.
}
