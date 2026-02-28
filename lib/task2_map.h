#ifndef TASK2_MAP_H
#define TASK2_MAP_H

#include <Arduino.h>

#define MAP_W   56
#define MAP_H   121
#define CELL_CM 5

enum Cell : uint8_t { FREE = 0, UNKNOWN = 1, WALL = 2, DYNAMIC = 3 };

// 2 bits per cell -> 4 cells per byte
static uint8_t map_data[(MAP_W * MAP_H + 3) / 4];

static inline int cell_index(int gx, int gy) {
  return gy * MAP_W + gx;
}

Cell map_get(int gx, int gy) {
  if (gx < 0 || gx >= MAP_W || gy < 0 || gy >= MAP_H) return WALL;

  int idx = cell_index(gx, gy);
  int byteIndex = idx >> 2;
  int shift = (idx & 0x03) << 1;
  return (Cell)((map_data[byteIndex] >> shift) & 0x03);
}

void map_set(int gx, int gy, Cell val) {
  if (gx < 0 || gx >= MAP_W || gy < 0 || gy >= MAP_H) return;

  int idx = cell_index(gx, gy);
  int byteIndex = idx >> 2;
  int shift = (idx & 0x03) << 1;

  map_data[byteIndex] &= ~(0x03 << shift);
  map_data[byteIndex] |= ((uint8_t)val & 0x03) << shift;
}

uint8_t* map_get_grid() {
  return map_data;
}

void map_init(Cell fill = FREE) {
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

// reverse helper so (0,0) is bottom-left
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
  // outer walls are 2 cells thick
  fill_rect(0, 0, MAP_W - 1, 1, WALL);
  fill_rect(0, MAP_H - 2, MAP_W - 1, MAP_H - 1, WALL);
  fill_rect(0, 0, 1, MAP_H - 1, WALL);
  fill_rect(MAP_W - 2, 0, MAP_W - 1, MAP_H - 1, WALL);
}

void map_mark_static_walls() {
  // Corrected so code uses bottom-left as (0,0)

  fill_rect_reversed(16, 99, 37, 100, WALL); // top inner horizontal
  fill_rect_reversed(27, 80, 28, 91, WALL);  // upper centre vertical
  fill_rect_reversed(0, 79, 10, 80, WALL);   // upper left horizontal
  fill_rect_reversed(46, 79, 55, 80, WALL);  // upper right horizontal
  fill_rect_reversed(7, 46, 8, 63, WALL);    // mid-left vertical
  fill_rect_reversed(8, 45, 33, 46, WALL);   // mid-left horizontal
  fill_rect_reversed(35, 57, 45, 58, WALL);  // mid-right short horizontal
  fill_rect_reversed(0, 29, 21, 30, WALL);   // lower left horizontal
  fill_rect_reversed(34, 29, 55, 30, WALL);  // lower right horizontal
  fill_rect_reversed(17, 14, 38, 15, WALL);  // bottom-centre horizontal
}

void map_print_serial() {
  Serial.println();
  for (int y = MAP_H - 1; y >= 0; y--) {
    for (int x = 0; x < MAP_W; x++) {
      Cell c = map_get(x, y);
      char ch = '?';
      if (c == FREE) ch = '.';
      else if (c == UNKNOWN) ch = '?';
      else if (c == WALL) ch = '#';
      else if (c == DYNAMIC) ch = 'D';
      Serial.print(ch);
    }
    Serial.println();
  }
}

#endif
