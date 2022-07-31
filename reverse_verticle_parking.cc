#include "reverse_verticle_parking.h"

#include "apollo_math/math_utils.h"
#include "vehicle_conf.h"
#include "geometry_parking_utils.h"

using namespace apollo::common::math;

bool RunReverseVerticleParking(const std::vector<LineSegment2d> &boundaries,
                               const Pose &start, LineCirclePath *result)
{
    /*------------PLAN 1: Do not shift gear--------------*/
    if (LineCircleLine(start, MAX_KAPPA, result))
    {
        if (!HasOverlapWithPath(*result, boundaries, /*resolution=*/0.2))
        {
            return true;
        }
    }

    /*------------PLAN 2: Shift gear only once--------------*/
    // Firstly drive to gear shift position.
    const auto get_pose_candidates = [](std::vector<double> xs,
                                        std::vector<double> ys, std::vector<double> thetas)
    {
        std::vector<Pose> res;
        for (const auto x : xs)
        {
            for (const auto y : ys)
            {
                for (const auto theta : thetas)
                {
                    res.push_back({Vec2d(x, y), theta, Vec2d::CreateUnitVec2d(theta)});
                }
            }
        }
        return res;
    };
    std::vector<Pose> pose_candidates;
    std::vector<double> thetas;
    for (double factor : {0.7, 0.6, 0.5, 0.4})
    {
        thetas.push_back(NormalizeAngle(start.theta + factor * (0.5 * M_PI - start.theta)));
    }
    if (start.pos.x() < 0.0)
    {
        pose_candidates = get_pose_candidates({0.2, 0.4, 0.8}, {4.3, 4.6, 4.9}, std::move(thetas));
    }
    else
    {
        pose_candidates = get_pose_candidates({-0.2, -0.4, -0.8}, {4.3, 4.6, 4.9}, std::move(thetas));
    }

    LineCirclePath first_path;
    for (const auto &pose : pose_candidates)
    {
        if (!LineCircleOrCircleLine(start, pose, MAX_KAPPA, &first_path))
        {
            continue;
        }
        if (HasOverlapWithPath(first_path, boundaries, /*resolution=*/0.2))
        {
            continue;
        }

        // Secondly drive forward.
        for (double s = 0.3; s < 3.0; s += 0.3)
        {
            const char type = start.pos.x() < 0.0 ? 'R' : 'L';
            const Pose end = GetPoseAlongArc(first_path.ends.back(), MAX_KAPPA, s, type);
            LineCirclePath forward_path = {{first_path.ends.back()}, {s}, {type}, {MAX_KAPPA}, {end}};
            if (HasOverlapWithPath(forward_path, boundaries, /*resolution=*/0.2))
            {
                break;
            }
            // Thirdly try parking in again.
            LineCirclePath final_path;
            if (!LineCircleLine(end, MAX_KAPPA, &final_path))
            {
                continue;
            }
            if (HasOverlapWithPath(final_path, boundaries, /*resolution=*/0.2))
            {
                continue;
            }
            // Fill results.
            *result = first_path;
            result->lengths.push_back(s);
            result->types.push_back(type);
            result->kappas.push_back(MAX_KAPPA);
            result->ends.push_back(end);
            for (int i = 0; i < final_path.lengths.size(); ++i)
            {
                result->lengths.push_back(final_path.lengths[i]);
                result->types.push_back(final_path.types[i]);
                result->kappas.push_back(final_path.kappas[i]);
                result->ends.push_back(final_path.ends[i]);
            }
            return true;
        }
    }

    /*------------PLAN 3: Shift gear for twice--------------*/
    // Firstly drive to gear shift position.
    for (const auto &pose : pose_candidates)
    {
        if (!LineCircleOrCircleLine(start, pose, MAX_KAPPA, &first_path))
        {
            continue;
        }
        if (HasOverlapWithPath(first_path, boundaries, /*resolution=*/0.2))
        {
            continue;
        }
        // Secondly find the furthest position av can drive forward.
        const auto has_collision = [&boundaries](const Box2d &box)
        {
            for (const auto &boundary : boundaries)
            {
                if (box.HasOverlap(boundary))
                    return true;
            }
            return false;
        };
        double furthest_s = 0.0;
        const char type = start.pos.x() < 0.0 ? 'R' : 'L';
        for (double s = 0.0; s < 3.0; s += 0.3)
        {
            const Pose end = GetPoseAlongArc(first_path.ends.back(), MAX_KAPPA, s, type);
            Box2d av_box(end.pos + 0.5 * (FRONT_TO_RAC - REAR_TO_RAC) * end.dir, end.theta, VEHICLE_LENGTH, VEHICLE_WIDTH);
            furthest_s = s;
            if (has_collision(av_box))
            {
                break;
            }
        }
        for (double s = furthest_s - 0.3; s < furthest_s; s += 0.03)
        {
            const Pose end = GetPoseAlongArc(first_path.ends.back(), MAX_KAPPA, s, type);
            Box2d av_box(end.pos + 0.5 * (FRONT_TO_RAC - REAR_TO_RAC) * end.dir, end.theta, VEHICLE_LENGTH, VEHICLE_WIDTH);
            furthest_s = s;
            if (has_collision(av_box))
            {
                break;
            }
        }
        furthest_s -= 0.03;
        const Pose end1 = GetPoseAlongArc(first_path.ends.back(), MAX_KAPPA, furthest_s, type);

        // Thirdly drive straight backward .
        for (double s1 = -0.3; s1 > -furthest_s - 0.5; s1 -= 0.2)
        {
            const Pose end2 = {end1.pos + s1 * end1.dir, end1.theta, end1.dir};
            Box2d av_box(end2.pos + 0.5 * (FRONT_TO_RAC - REAR_TO_RAC) * end2.dir, end2.theta, VEHICLE_LENGTH, VEHICLE_WIDTH);
            if (has_collision(av_box))
            {
                break;
            }
            // Fourthly drive forward again.
            for (double s2 = 0.4; s2 < 3.0; s2 += 0.2)
            {
                const Pose end3 = GetPoseAlongArc(end2, MAX_KAPPA, s2, type);
                LineCirclePath cur_path = {{end2}, {s2}, {type}, {MAX_KAPPA}, {end3}};
                if (HasOverlapWithPath(cur_path, boundaries, /*resolution=*/0.2))
                {
                    break;
                }
                // Finally try parking in again.
                LineCirclePath final_path;
                if (!LineCircleLine(end3, MAX_KAPPA, &final_path))
                {
                    continue;
                }
                if (HasOverlapWithPath(final_path, boundaries, /*resolution=*/0.2))
                {
                    continue;
                }
                // Fill results.
                *result = first_path;
                result->lengths.push_back(furthest_s);
                result->types.push_back(type);
                result->kappas.push_back(MAX_KAPPA);
                result->ends.push_back(end1);

                result->lengths.push_back(s1);
                result->types.push_back('S');
                result->kappas.push_back(0.0);
                result->ends.push_back(end2);

                result->lengths.push_back(s2);
                result->types.push_back(type);
                result->kappas.push_back(MAX_KAPPA);
                result->ends.push_back(end3);

                for (int i = 0; i < final_path.lengths.size(); ++i)
                {
                    result->lengths.push_back(final_path.lengths[i]);
                    result->types.push_back(final_path.types[i]);
                    result->kappas.push_back(final_path.kappas[i]);
                    result->ends.push_back(final_path.ends[i]);
                }
                return true;
            }
        }
    }
    return false;
}