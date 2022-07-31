#include "line_circle_connection.h" 

// Assume goal is 0, 0, 0.5 * M_PI.
bool RunReverseVerticleParking(const std::vector<LineSegment2d>& boundaries, 
const Pose& start, LineCirclePath* result);