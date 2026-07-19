#pragma once

// World frame: origin = takeoff point, +x = forward (nose direction at
// takeoff), +y = left, +z = up. Units are meters.
//
// keepHeading:
//   true  -> nose stays pointed at the takeoff heading (0°) for this leg
//   false -> nose turns to face this waypoint before/while flying to it
//
// A waypoint with z = 0 is treated as landing: once reached, motors cut
// and the mission ends. Put it last.

struct Waypoint { float x, y, z; bool keepHeading; };

// Hover altitude capped at 0.4 m -- Rangefinder.cpp's VL53L1X reads
// accurately near 0.19 m but showed a growing undershoot at 0.58 m+ in
// bench testing (uncalibrated/default distance mode), so this mission stays
// well clear of that degraded range rather than trying to compensate for it.
static const Waypoint FLIGHT_PATH[] = {
  { 0.0f, 0.0f, 0.4f, true  },   // takeoff, rise to 0.4 m
  { 1.0f, 0.0f, 0.4f, false },   // point A
  { 1.0f, 1.0f, 0.4f, false },   // point B
  { 0.0f, 0.0f, 0.4f, true  },   // back to origin
  { 0.0f, 0.0f, 0.0f, true  },   // land
};

static constexpr int   NUM_WAYPOINTS = sizeof(FLIGHT_PATH) / sizeof(FLIGHT_PATH[0]);
static constexpr float WP_RADIUS     = 0.15f;   // arrival radius, meters
