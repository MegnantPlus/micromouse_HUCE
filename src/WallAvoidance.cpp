#include "WallAvoidance.h"
#include <Arduino.h>

namespace {
constexpr int SENSOR_FL_INDEX = SIDX(S_FL);
constexpr int SENSOR_FR_INDEX = SIDX(S_FR);

bool isWallThresholdReady(int sensorIndex, const RuntimeParams &cfg) {
  int margin = max(50, cfg.ir_deadband / 2);
  return max_IR[sensorIndex] < 4095 - margin;
}

int closeWallThreshold(int sensorIndex, const RuntimeParams &cfg) {
  if (!isWallThresholdReady(sensorIndex, cfg))
    return 4095;
  return constrain(max_IR[sensorIndex] + cfg.offset_upper, 0, 4095);
}
}

int wallAvoidanceKick(const RuntimeParams &cfg) {
  return constrain(cfg.Turn_Min / 3, 6, 15);
}

WallAvoidanceResult computeStraightWallAvoidance(const IrSnapshot &ir,
                                                 const RuntimeParams &cfg) {
  WallAvoidanceResult result = {};
  result.leftThreshold = closeWallThreshold(SENSOR_FL_INDEX, cfg);
  result.rightThreshold = closeWallThreshold(SENSOR_FR_INDEX, cfg);

  int triggerDeadband = max(10, cfg.ir_deadband / 2);
  if (isWallThresholdReady(SENSOR_FL_INDEX, cfg)) {
    result.leftError =
        max(0, ir.frontLeft - result.leftThreshold - triggerDeadband);
  }
  if (isWallThresholdReady(SENSOR_FR_INDEX, cfg)) {
    result.rightError =
        max(0, ir.frontRight - result.rightThreshold - triggerDeadband);
  }

  int rawSteer = result.leftError - result.rightError;
  int steerLimit = max(0, cfg.Turn_Max - cfg.Turn_Min);
  if (rawSteer > 0) {
    result.steer = max((int)(rawSteer * cfg.k_ir), wallAvoidanceKick(cfg));
  } else if (rawSteer < 0) {
    result.steer = min((int)(rawSteer * cfg.k_ir), -wallAvoidanceKick(cfg));
  }

  result.steer = constrain(result.steer, -steerLimit, steerLimit);
  result.active = result.steer != 0;
  return result;
}
