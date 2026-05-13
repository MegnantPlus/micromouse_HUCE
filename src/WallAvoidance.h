#pragma once

#include "Globals.h"

struct WallAvoidanceResult {
  bool active;
  int steer;
  int leftError;
  int rightError;
  int leftThreshold;
  int rightThreshold;
};

WallAvoidanceResult computeStraightWallAvoidance(const IrSnapshot &ir,
                                                 const RuntimeParams &cfg);
int wallAvoidanceKick(const RuntimeParams &cfg);
