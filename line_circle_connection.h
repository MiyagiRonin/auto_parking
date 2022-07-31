#pragma once

#include "geometry_parking_utils.h"

// Assume goal is 0, 0, 0.5 * M_PI
bool LineCircleLine(const Pose& start, double kappa, LineCirclePath* result);

bool LineCircleOrCircleLine(const Pose& start, const Pose& end, double max_kappa, LineCirclePath* result);