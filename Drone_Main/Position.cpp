#include "Position.h"
#include <math.h>

// Focal length of the PMW3901 lens in effective pixels.
// Bench-calibrated 2026-07-19 via two known-distance slide tests (3 ft and
// 5 ft) with Position_Test.ino -- the old value of 30.0f was off by ~20.5x
// (a genuine unit conversion error, not just an uncalibrated guess). New
// value derived from x-axis scale factor, consistent to within ~4.5% across
// both tests. Re-tune the same way if the flow sensor or lens is ever
// replaced: move the drone a known distance and compare reported X/Y against
// reality. Increase if reported distance reads too small, decrease if it
// reads too large.
static constexpr float FOCAL_LEN    = 618.0f;

// Altitude low-pass weight. Higher = smoother but slower to respond.
static constexpr float ALT_ALPHA    = 0.85f;

// Valid altitude window from the rangefinder.
static constexpr float ALT_MIN      = 0.05f;  // m
static constexpr float ALT_MAX      = 3.50f;  // m

// Optical flow noise gate: counts at or below this are treated as noise
// and do not update position. Prevents drift while hovering stationary.
static constexpr int16_t FLOW_GATE  = 1;

Position::Position() {}

void Position::setup(IMU &imu, FlowSensor &flow, Rangefinder &rangefinder) {
  // Sensors are already initialized in their own setup() calls.
  // Nothing extra needed here — just capture the start time.
  _lastMicros = micros();
}

void Position::update(IMU &imu, FlowSensor &flow, Rangefinder &rangefinder) {
  unsigned long now = micros();
  float dt = (now - _lastMicros) * 1.0e-6f;
  _lastMicros = now;
  if (dt <= 0.0f || dt > 0.1f) return;

  // ── Altitude (Z) ─────────────────────────────────────────────────────────
  // Rangefinder measures slant distance along its axis. When the drone is
  // tilted, correct back to vertical height using roll and pitch.
  rangefinder.update();
  if (rangefinder.hasData()) {
    float h = rangefinder.getDistance() * cosf(imu.getRoll()) * cosf(imu.getPitch());
    if (h >= ALT_MIN && h <= ALT_MAX)
      _z = ALT_ALPHA * _z + (1.0f - ALT_ALPHA) * h;
  }

  // Refuse to update X/Y without a valid altitude — altitude is required to
  // scale the optical flow counts into real-world meters.
  if (_z < ALT_MIN) return;

  // ── X/Y from optical flow ─────────────────────────────────────────────────
  // Bench-verified 2026-07-18: the PMW3901's raw axes are swapped AND
  // inverted relative to body forward/left -- moving the drone +X (forward)
  // showed up as -Y, and +Y (left) showed up as -X. This corrects it in
  // software, the same way IMU.cpp's FLOW_X_SIGN/FLOW_Y_SIGN already correct
  // for the (separate) 90 deg IMU-vs-flow-sensor mounting relationship.
  int16_t dxRaw = 0, dyRaw = 0;
  flow.read(dxRaw, dyRaw);
  int16_t dx = -dyRaw;
  int16_t dy = -dxRaw;

  if (abs(dx) <= FLOW_GATE && abs(dy) <= FLOW_GATE) return;

  // Remove the apparent motion caused by drone rotation. When the drone yaws
  // (GZ) or rolls (GX), the sensor sees ground features sweep across its
  // field of view even with no translational movement. Subtracting the gyro
  // contribution isolates true translation.
  float dx_f = (float)dx - imu.getGZ() * FOCAL_LEN * dt;
  float dy_f = (float)dy - imu.getGX() * FOCAL_LEN * dt;

  // Convert flow counts to body-frame displacement in meters.
  // displacement = counts * height / focal_length
  float disp_x = dx_f * _z / FOCAL_LEN;
  float disp_y = dy_f * _z / FOCAL_LEN;

  // Rotate body-frame displacement into world frame before accumulating.
  // _x/_y must stay in a fixed world frame to match the (fixed) waypoint
  // targets in Controller/PositionPID -- without this rotation the estimate
  // silently drifts relative to the mission whenever the vehicle isn't
  // facing yaw=0 (any keep_heading=false leg).
  float yawNow = imu.getYaw();
  float cosY = cosf(yawNow), sinY = sinf(yawNow);
  float disp_x_world = cosY * disp_x - sinY * disp_y;
  float disp_y_world = sinY * disp_x + cosY * disp_y;

  _x += disp_x_world;
  _y += disp_y_world;
}

float Position::getX() const { return _x; }
float Position::getY() const { return _y; }
float Position::getZ() const { return _z; }

void Position::reset() {
  _x = _y = 0.0f;
  // Z is intentionally not reset — it reflects real altitude from the sensor.
}
