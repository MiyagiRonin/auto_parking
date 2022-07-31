#include "line_circle_connection.h"

#include "apollo_math/math_utils.h"

using namespace apollo::common::math;

bool LineCircleLine(const Pose& start, double kappa, LineCirclePath* result) {
    double start_x = start.pos.x();
    double start_y = start.pos.y();
    double start_theta = start.theta;
    double start_sin_theta = start.dir.y();
    double start_cos_theta = start.dir.x();

    if(start_y < 0.0) return false;
    if(start_x > 0.0 && start_theta > 0.5 * M_PI) return false;
    if(start_x < 0.0 && start_theta < 0.5 * M_PI) return false;
    //Symmetric transformation.
    bool is_symmetric = false;
    if(start_x < 0.0) {
        start_x = -start_x;
        start_theta = NormalizeAngle(M_PI - start_theta);
        start_cos_theta = -start_cos_theta;
        is_symmetric = true;
    } 

    const double R = 1.0/kappa;
    const double theta = 0.5 * M_PI - start_theta;
    const double c = -theta * R;
    const double delta_x = -2.0 * R * Square(std::sin(theta * 0.5));
    const double delta_y = - R * std::sin(theta);
    const double l1 = -(start_x + delta_x) / start_cos_theta;
    const double l2 = -(start_y + l1 * start_sin_theta + delta_y);
    if(c > 0.0 || l1 > 0.0) return false;

    //Fill results.
    result->start = start;
    result->lengths = {l1, c, l2};
    result->types = {'S', is_symmetric ? 'L' : 'R', 'S'};
    result->kappas = {0.0, kappa, 0.0};
    Pose end1 = {start.pos + l1 * start.dir, start.theta, start.dir};
    Pose end2 = GetPoseAlongArc(end1, kappa, c, result->kappas[1]);
    Pose end3 = {end2.pos + l2 * end2.dir, end2.theta, end2.dir};
    result->ends = {end1, end2, end3};
    return true;
}

bool LineCircleOrCircleLine(const Pose& start, const Pose& end, double max_kappa, LineCirclePath* result) {
    // Transfer end to origin.
    const double delta_x = start.pos.x() - end.pos.x();
    const double delta_y = start.pos.y() - end.pos.y();
    double x = delta_x * end.dir.x() + delta_y * end.dir.y();
    double y = delta_y * end.dir.x() - delta_x * end.dir.y();
    double theta = NormalizeAngle(start.theta - end.theta);

    if (x < 0.0) return false;
    if (y > 0.0 && theta > 0.5 * M_PI) return false;
    if (y < 0.0 && theta < -0.5 * M_PI) return false;

    // Symmetric transformation.
    bool is_symmetric = false;
    if (y < 0.0) {
        y = -y;
        theta = -theta;
        is_symmetric = true;
    }
    
    const double l1 = x - y / std::tan(theta);
    if (l1 < 0.0) return false;
    const double l2 = y / std::sin(theta);

    if (l1 < l2) {
        //Line-Circle.
        const double kappa = std::tan(0.5 * theta) / l1;
        if(kappa > max_kappa) return false;
        //Fill results.
        result->start = start;
        result->lengths = {l1-l2, -theta/kappa};
        result->kappas = {0.0, kappa};
        result->types = {'S', is_symmetric ? 'R' : 'L'};
        Pose end1 = {start.pos + result->lengths[0] * start.dir, start.theta, start.dir};
        Pose end2 = GetPoseAlongArc(end1, result->kappas[1], result->lengths[1], result->types[1]);
        result->ends = {end1, end2};
        return true;
    } else {
        // Circle-Line.
        const double kappa = std::tan(0.5 * theta) / l2;
        if (kappa > max_kappa) return false;
        // Fill results. 
        result->start = start;
        result->lengths = {-theta/kappa, l2 - l1};
        result->kappas = {kappa, 0.0};
        result->types = {is_symmetric ? 'R' : 'L', 'S'};
        Pose end1 = GetPoseAlongArc(start, result->kappas[0], result->lengths[0], result->types[0]);
        Pose end2 = {end1.pos + end1.dir* result->lengths[1], end1.theta, end1.dir};
        result->ends = {end1, end2};
        return true;
    }
    return false;
}