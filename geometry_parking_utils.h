#pragma once

#include "apollo_math/box2d.h"
#include "vehicle_conf.h"

using Vec2d = apollo::common::math::Vec2d;
using AABox2d = apollo::common::math::AABox2d;
using LineSegment2d = apollo::common::math::LineSegment2d;
using Box2d = apollo::common::math::Box2d;

struct Pose
{
    Vec2d pos;
    double theta;
    Vec2d dir;
};

struct LineCirclePath
{
    Pose start;
    std::vector<double> lengths;
    std::vector<char> types;
    std::vector<double> kappas;
    std::vector<Pose> ends;
};

Pose GetPoseAlongArc(const Pose &start, double kappa, double length, char type);

std::vector<Pose> ConvertPathToDiscretePoses(const LineCirclePath& path, double step);

bool HasOverlapWithPath(const LineCirclePath& path, const std::vector<LineSegment2d>& boundaries, double resolution);