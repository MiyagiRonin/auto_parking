#include "geometry_parking_utils.h"

#include "apollo_math/math_utils.h"

Pose GetPoseAlongArc(const Pose &start, double kappa, double length, char type)
{
    const double R = 1.0 / kappa;
    const double sign = type == 'L' ? 1.0 : -1.0;
    const double delta_theta = sign * kappa * length;
    const double delta_x = sign * R * std::sin(delta_theta);
    const double delta_y = sign * R * (1.0 - std::cos(delta_theta));
    const double x = start.pos.x() + delta_x * start.dir.x() - delta_y * start.dir.y();
    const double y = start.pos.y() + delta_x * start.dir.y() + delta_y * start.dir.x();
    const double theta = apollo::common::math::NormalizeAngle(start.theta + delta_theta);
    return {Vec2d(x, y), theta, Vec2d::CreateUnitVec2d(theta)};
}

std::vector<Pose> ConvertPathToDiscretePoses(const LineCirclePath &path, double step)
{
    std::vector<Pose> res;
    res.push_back(path.start);
    for (int i = 0; i < path.lengths.size(); ++i)
    {
        const auto cur_start = res.back();
        switch (path.types[i])
        {
        case 'S':
        {
            for (double s = step; s < std::abs(path.lengths[i]) + step; s += step)
            {
                s = std::min(s, std::abs(path.lengths[i]));
                const auto pos = cur_start.pos + std::copysign(s, path.lengths[i]) * cur_start.dir;
                res.push_back({pos, cur_start.theta, cur_start.dir});
            }
            break;
        }
        case 'L':
        case 'R':
        {
            for (double s = step; s < std::abs(path.lengths[i]) + step; s += step)
            {
                s = std::min(s, std::abs(path.lengths[i]));
                res.push_back(GetPoseAlongArc(cur_start, path.kappas[i], std::copysign(s, path.lengths[i]), path.types[i]));
            }
            break;
        }
        }
    }
    return res;
}

bool HasOverlapWithPath(const LineCirclePath &path, const std::vector<LineSegment2d> &boundaries, double resolution)
{
    const auto poses = ConvertPathToDiscretePoses(path, resolution);
    std::vector<Box2d> av_boxes;
    for (const auto &pose : poses)
    {
        av_boxes.emplace_back(pose.pos + 0.5 * (FRONT_TO_RAC - REAR_TO_RAC) * pose.dir, pose.theta, VEHICLE_LENGTH, VEHICLE_WIDTH);
    }
    for (const auto &boundary : boundaries)
    {
        for (const auto &box : av_boxes)
        {
            if (box.HasOverlap(boundary))
                return true;
        }
    }
    return false;
}