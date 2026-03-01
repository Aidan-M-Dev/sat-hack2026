#include <Arduino.h>
#include "ADCS.h"
#include "calibration.h"
#include "sensing.h"
#include "movement.h"
#include "localize.h"

// ============================================================
// Nano-safe reactive obstacle solver for the 3m x 6m course
// Uses the uploaded SatHack headers, but deliberately avoids
// mapping.h and particle_filter.h because they exceed Nano SRAM.
// ============================================================

enum Mode {
  MODE_DRIVE = 0,
  MODE_BYPASS_LEFT,
  MODE_BYPASS_RIGHT,
  MODE_RECOVER
};

static Mode mode = MODE_DRIVE;

// ---------- tuneable behaviour constants ----------
static const int   BASE_SPEED            = REF_SPEED;
static const float STEP_CM               = 30.0f;
static const float BACKUP_CM             = 30.0f;
static const int   PEEK_DEG              = 20;
static const int   BYPASS_TURN_DEG       = 16;
static const int   UNWIND_TURN_DEG       = 8;
static const int   MAX_STEER             = 28;
static const int   BYPASS_STEER          = 18;
static const int   MAX_TURN_BUDGET_DEG   = 80;
static const int   COMMIT_STEPS          = 5;

static const float BLOCKED_CM            = 22.0f;
static const float OPEN_CM               = 30.0f;
static const float EXIT_CM               = 38.0f;
static const float SIDE_DIFF_CM          = 6.0f;
static const unsigned long SENSOR_SETTLE_MS = 70;

// ---------- controller memory ----------
static int steeringBias = 0;       // +ve = steer right, -ve = steer left
static int cumulativeTurn = 0;     // +ve = net right turn, -ve = net left turn
static int bypassCountdown = 0;
static int repeatedBlockCount = 0;
static unsigned long loopCount = 0;

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------
static float median5(float a, float b, float c, float d, float e) {
  float v[5] = {a, b, c, d, e};
  for (int i = 1; i < 5; i++) {
    float key = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > key) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = key;
  }
  return v[2];
}

static float robustDistanceCm() {
  float a = read_distance_world();
  delay(8);
  float b = read_distance_world();
  delay(8);
  float c = read_distance_world();
  delay(8);
  float d = read_distance_world();
  delay(8);
  float e = read_distance_world();
  return median5(a, b, c, d, e);
}

static bool isBlocked(float d) {
  return (d > 0.0f && d < BLOCKED_CM);
}

static bool isOpen(float d) {
  return (d >= OPEN_CM || d > 300.0f);
}

static void hardStop() {
  stopMotors();
  delay(80);
}

static void driveSteeredCm(float cm, int steer) {
  if (cm <= 0.0f) return;

  steer = constrain(steer, -MAX_STEER, MAX_STEER);

  unsigned long duration = (unsigned long)(cm * MS_PER_CM) + STARTUP_MS;

  int leftFwd  = constrain(BASE_SPEED + steer, 0, 255);
  int rightFwd = constrain((BASE_SPEED - offset) - steer, 0, 255);

  Serial.print("[drive] cm="); Serial.print(cm);
  Serial.print(" steer="); Serial.print(steer);
  Serial.print(" left="); Serial.print(leftFwd);
  Serial.print(" right="); Serial.println(rightFwd);

  motorL(leftFwd, 0);
  motorR(rightFwd, 0);
  delay(duration);
  hardStop();
  loc_update_forward(cm);
}

static void pivotLeftTracked(int deg) {
  if (deg <= 0) return;
  turnLeft(deg);
  loc_update_turn(-deg);  // left turn = anticlockwise = negative in localize.h
  cumulativeTurn -= deg;
  cumulativeTurn = constrain(cumulativeTurn, -MAX_TURN_BUDGET_DEG, MAX_TURN_BUDGET_DEG);
}

static void pivotRightTracked(int deg) {
  if (deg <= 0) return;
  turnRight(deg);
  loc_update_turn(deg);   // right turn = clockwise = positive
  cumulativeTurn += deg;
  cumulativeTurn = constrain(cumulativeTurn, -MAX_TURN_BUDGET_DEG, MAX_TURN_BUDGET_DEG);
}

static void backupTracked(float cm) {
  if (cm <= 0.0f) return;
  reverse_dist(cm);
  loc_update_forward(-cm);
}

static float peekLeftCm() {
  pivotLeftTracked(PEEK_DEG);
  delay(SENSOR_SETTLE_MS);
  float d = robustDistanceCm();
  pivotRightTracked(PEEK_DEG);
  delay(SENSOR_SETTLE_MS);
  return d;
}

static float peekRightCm() {
  pivotRightTracked(PEEK_DEG);
  delay(SENSOR_SETTLE_MS);
  float d = robustDistanceCm();
  pivotLeftTracked(PEEK_DEG);
  delay(SENSOR_SETTLE_MS);
  return d;
}

static void updateSteeringBias() {
  float left = peekLeftCm();
  float right = peekRightCm();

  if (left > 300.0f) left = 300.0f;
  if (right > 300.0f) right = 300.0f;

  float diff = right - left;   // more open on right => positive => steer right
  int newBias = (int)(diff * 1.5f);
  steeringBias = constrain(newBias, -MAX_STEER, MAX_STEER);

  Serial.print("[bias] left="); Serial.print(left);
  Serial.print(" right="); Serial.print(right);
  Serial.print(" diff="); Serial.print(diff);
  Serial.print(" steeringBias="); Serial.println(steeringBias);
}

static int chooseBypassSide() {
  float left = peekLeftCm();
  float right = peekRightCm();

  Serial.print("[choose] left="); Serial.print(left);
  Serial.print(" right="); Serial.println(right);

  if (right > left + SIDE_DIFF_CM) return +1;
  if (left > right + SIDE_DIFF_CM) return -1;

  // Tie-breaker: prefer whichever direction reduces net accumulated turning.
  if (cumulativeTurn > 0) return -1;
  if (cumulativeTurn < 0) return +1;

  // Final fallback: default right.
  return +1;
}

static void unwindHeadingIfSafe(float center) {
  if (center < EXIT_CM) return;

  if (cumulativeTurn >= UNWIND_TURN_DEG) {
    Serial.println("[unwind] left");
    pivotLeftTracked(UNWIND_TURN_DEG);
  } else if (cumulativeTurn <= -UNWIND_TURN_DEG) {
    Serial.println("[unwind] right");
    pivotRightTracked(UNWIND_TURN_DEG);
  }
}

static void printPose() {
  Pose p = loc_get();
  Serial.print("[pose] x="); Serial.print(p.x);
  Serial.print(" y="); Serial.print(p.y);
  Serial.print(" heading="); Serial.print(p.heading);
  Serial.print(" cumTurn="); Serial.println(cumulativeTurn);
}

// ------------------------------------------------------------
// Setup / loop
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Hardware init matching ADCS.h / movement.h pin usage
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  hardStop();

  // For this course solver the ultrasonic must face forward.
  init_sensing(2);   // stage 2/3 => front mount offsets from calibration.h
  loc_reset();

  Serial.println("=== nano_course_solver starting ===");
  Serial.println("Using ADCS.h + calibration.h + sensing.h + movement.h + localize.h");
  Serial.println("Mapping / particle filter intentionally disabled on Nano SRAM budget");
  delay(1000);
}

void loop() {
  float center = robustDistanceCm();

  Serial.print("\n[loop] #"); Serial.print(loopCount++);
  Serial.print(" mode="); Serial.print((int)mode);
  Serial.print(" center="); Serial.println(center);
  printPose();

  // Emergency stop layer
  if (center > 0.0f && center < EMERGENCY_STOP_CM) {
    Serial.println("[safety] emergency stop");
    hardStop();
    mode = MODE_RECOVER;
  }

  switch (mode) {
    case MODE_DRIVE:
      if (isBlocked(center)) {
        repeatedBlockCount++;
        int side = chooseBypassSide();
        bypassCountdown = COMMIT_STEPS;

        if (side > 0) {
          Serial.println("[mode] -> BYPASS_RIGHT");
          mode = MODE_BYPASS_RIGHT;
        } else {
          Serial.println("[mode] -> BYPASS_LEFT");
          mode = MODE_BYPASS_LEFT;
        }
        break;
      }

      repeatedBlockCount = 0;

      // Refresh wall-centering estimate every few steps while still moving mainly forward.
      if ((loopCount % 3UL) == 0UL) {
        updateSteeringBias();
      }

      // If the robot has accumulated too much heading change, gently unwind when safe.
      unwindHeadingIfSafe(center);

      driveSteeredCm(STEP_CM, steeringBias);
      break;

    case MODE_BYPASS_RIGHT:
      if (isBlocked(center)) {
        Serial.println("[bypass right] turning right around obstacle");
        pivotRightTracked(BYPASS_TURN_DEG);
      } else {
        Serial.println("[bypass right] stepping forward-right");
        driveSteeredCm(STEP_CM, +BYPASS_STEER);
        bypassCountdown--;
      }

      if (bypassCountdown <= 0 && center >= EXIT_CM) {
        steeringBias = +8;
        Serial.println("[mode] right bypass complete -> DRIVE");
        mode = MODE_DRIVE;
      }
      break;

    case MODE_BYPASS_LEFT:
      if (isBlocked(center)) {
        Serial.println("[bypass left] turning left around obstacle");
        pivotLeftTracked(BYPASS_TURN_DEG);
      } else {
        Serial.println("[bypass left] stepping forward-left");
        driveSteeredCm(STEP_CM, -BYPASS_STEER);
        bypassCountdown--;
      }

      if (bypassCountdown <= 0 && center >= EXIT_CM) {
        steeringBias = -8;
        Serial.println("[mode] left bypass complete -> DRIVE");
        mode = MODE_DRIVE;
      }
      break;

    case MODE_RECOVER: {
      Serial.println("[recover] backing up and re-choosing side");
      backupTracked(BACKUP_CM);
      int side = chooseBypassSide();
      bypassCountdown = COMMIT_STEPS + 2;
      repeatedBlockCount = 0;

      if (side > 0) {
        mode = MODE_BYPASS_RIGHT;
      } else {
        mode = MODE_BYPASS_LEFT;
      }
      break;
    }
  }
}
