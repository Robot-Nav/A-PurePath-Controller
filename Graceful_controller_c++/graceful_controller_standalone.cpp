#include "graceful_controller_standalone.hpp"

namespace graceful_controller
{

SmoothControlLaw::SmoothControlLaw(
        double k_phi, double k_delta, double beta, double lambda, double slowdown_radius,
        double v_linear_min, double v_linear_max, double v_angular_max)
        : k_phi_(k_phi), k_delta_(k_delta), beta_(beta), lambda_(lambda),
          slowdown_radius_(slowdown_radius), v_linear_min_(v_linear_min),
          v_linear_max_(v_linear_max), v_angular_max_(v_angular_max)
{
    if (k_phi_ < 0) {
        throw std::invalid_argument("k_phi must be non-negative");
    }
    if (k_delta_ < 0) {
        throw std::invalid_argument("k_delta must be non-negative");
    }
    if (beta_ <= 0) {
        throw std::invalid_argument("beta must be positive");
    }
    if (lambda_ < 1) {
        throw std::invalid_argument("lambda must be >= 1");
    }
    if (slowdown_radius_ < 0) {
        throw std::invalid_argument("slowdown_radius cannot be negative");
    }
    if (v_linear_min_ < 0) {
        throw std::invalid_argument("v_linear_min cannot be negative");
    }
    if (v_linear_max_ < v_linear_min_) {
        throw std::invalid_argument("v_linear_max must be >= v_linear_min");
    }
    if (v_angular_max_ <= 0) {
        throw std::invalid_argument("v_angular_max must be positive");
    }
}

void SmoothControlLaw::setCurvatureConstants(
        double k_phi, double k_delta, double beta, double lambda)
{
    if (k_phi < 0) {
        throw std::invalid_argument("k_phi must be non-negative");
    }
    if (k_delta < 0) {
        throw std::invalid_argument("k_delta must be non-negative");
    }
    if (beta <= 0) {
        throw std::invalid_argument("beta must be positive");
    }
    if (lambda < 1) {
        throw std::invalid_argument("lambda must be >= 1");
    }

    k_phi_ = k_phi;
    k_delta_ = k_delta;
    beta_ = beta;
    lambda_ = lambda;
}

void SmoothControlLaw::setSlowdownRadius(double slowdown_radius)
{
    if (slowdown_radius < 0) {
        throw std::invalid_argument("slowdown_radius cannot be negative");
    }
    slowdown_radius_ = slowdown_radius;
}

void SmoothControlLaw::setSpeedLimit(
        const double v_linear_min, const double v_linear_max, const double v_angular_max)
{
    if (v_linear_min < 0) {
        throw std::invalid_argument("v_linear_min cannot be negative");
    }
    if (v_linear_max < v_linear_min) {
        throw std::invalid_argument("v_linear_max must be >= v_linear_min");
    }
    if (v_angular_max <= 0) {
        throw std::invalid_argument("v_angular_max must be positive");
    }

    v_linear_min_ = v_linear_min;
    v_linear_max_ = v_linear_max;
    v_angular_max_ = v_angular_max;
}

Twist SmoothControlLaw::calculateRegularVelocity(
        const Pose2D & target, const Pose2D & current, bool backward)
{
    EgocentricPolarCoordinates ego_coords(target, current, backward);

    if (ego_coords.r < std::numeric_limits<double>::epsilon()) {
        return Twist();
    }

    double curvature = calculateCurvature(ego_coords.r, ego_coords.phi, ego_coords.delta);

    if (backward) {
        curvature = -curvature;
    }

    double v = v_linear_max_;
    if (std::abs(curvature) > std::numeric_limits<double>::epsilon()) {
        v = v_linear_max_ / (1.0 + beta_ * std::pow(std::abs(curvature), lambda_));
    }

    if (slowdown_radius_ > std::numeric_limits<double>::epsilon() &&
        ego_coords.r < slowdown_radius_) {
        double slowdown_factor = ego_coords.r / slowdown_radius_;
        v = std::min(v_linear_max_ * slowdown_factor, v);
    }

    v = std::clamp(v, v_linear_min_, v_linear_max_);

    if (backward) {
        v = -v;
    }

    double w = curvature * v;

    double w_bound = std::clamp(w, -v_angular_max_, v_angular_max_);

    if (std::abs(curvature) > std::numeric_limits<double>::epsilon()) {
        v = w_bound / curvature;
    }

    if (backward) {
        v = std::clamp(v, -v_linear_max_, -v_linear_min_);
    } else {
        v = std::clamp(v, v_linear_min_, v_linear_max_);
    }

    Twist cmd_vel;
    cmd_vel.linear_x = v;
    cmd_vel.angular_z = w_bound;
    
    std::cout << "[SmoothControlLaw] ego: r=" << ego_coords.r 
              << ", phi=" << ego_coords.phi 
              << ", delta=" << ego_coords.delta
              << ", curvature=" << curvature
              << ", v=" << v 
              << ", w=" << w_bound << std::endl;

    return cmd_vel;
}

Twist SmoothControlLaw::calculateRegularVelocity(
        const Pose2D & target, bool backward)
{
    Pose2D current(Point2D(0.0, 0.0), 0.0);
    return calculateRegularVelocity(target, current, backward);
}

Pose2D SmoothControlLaw::calculateNextPose(
        const double dt,
        const Pose2D & target,
        const Pose2D & current,
        bool backward)
{
    if (dt <= 0) {
        throw std::invalid_argument("dt must be positive");
    }

    Twist vel = calculateRegularVelocity(target, current, backward);

    Pose2D next;
    double yaw = current.orientation;

    next.position.x = current.position.x + vel.linear_x * dt * std::cos(yaw);
    next.position.y = current.position.y + vel.linear_x * dt * std::sin(yaw);

    yaw += vel.angular_z * dt;
    yaw = normalizeAngle(yaw);

    next.orientation = yaw;

    return next;
}

double SmoothControlLaw::calculateCurvature(double r, double phi, double delta)
{
    if (r < std::numeric_limits<double>::epsilon()) {
        return 0.0;
    }

    double prop_term = k_delta_ * (delta - std::atan(-k_phi_ * phi));

    double denom = 1.0 + std::pow(k_phi_ * phi, 2);
    double feedback_term = (1.0 + (k_phi_ / denom)) * std::sin(delta);

    double curvature = -(prop_term + feedback_term) / r;

    if (std::isnan(curvature) || std::isinf(curvature)) {
        return 0.0;
    }

    return curvature;
}

ParameterHandler::ParameterHandler(const std::string& plugin_name)
        : plugin_name_(plugin_name)
{
    if (params_.initial_rotation && params_.allow_backward) {
        params_.allow_backward = false;
        std::cout << "[ParameterHandler] Warning: Initial rotation and backward motion "
                  << "are both enabled. Disabling backward motion." << std::endl;
    }

    validateParameters();
}

Parameters ParameterHandler::getParams() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return params_;
}

void ParameterHandler::updateParams(const Parameters& new_params)
{
    std::lock_guard<std::mutex> lock(mutex_);

    Parameters old_params = params_;

    params_ = new_params;

    if (!validateParameters()) {
        params_ = old_params;
        throw std::invalid_argument("Invalid parameter values");
    }

    std::cout << "[ParameterHandler] Parameters updated for plugin: "
              << plugin_name_ << std::endl;
}

void ParameterHandler::updateParam(const std::string& param_name, double value)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (param_name == "min_lookahead") {
        params_.min_lookahead = value;
    } else if (param_name == "max_lookahead") {
        params_.max_lookahead = value;
    } else if (param_name == "k_phi") {
        params_.k_phi = value;
    } else if (param_name == "k_delta") {
        params_.k_delta = value;
    } else if (param_name == "beta") {
        params_.beta = value;
    } else if (param_name == "lambda") {
        params_.lambda = value;
    } else if (param_name == "v_linear_min") {
        params_.v_linear_min = value;
    } else if (param_name == "v_linear_max") {
        params_.v_linear_max = value;
        params_.v_linear_max_initial = value;
    } else if (param_name == "v_angular_max") {
        params_.v_angular_max = value;
        params_.v_angular_max_initial = value;
    } else if (param_name == "v_angular_min_in_place") {
        params_.v_angular_min_in_place = value;
    } else if (param_name == "slowdown_radius") {
        params_.slowdown_radius = value;
    } else if (param_name == "initial_rotation_tolerance") {
        params_.initial_rotation_tolerance = value;
    } else if (param_name == "rotation_scaling_factor") {
        params_.rotation_scaling_factor = value;
    } else if (param_name == "in_place_collision_resolution") {
        params_.in_place_collision_resolution = value;
    } else if (param_name == "collision_check_radius") {
        params_.collision_check_radius = value;
    } else if (param_name == "safety_distance") {
        params_.safety_distance = value;
    } else {
        std::cerr << "[ParameterHandler] Unknown double parameter: "
                  << param_name << std::endl;
        return;
    }

    validateParameters();
}

void ParameterHandler::updateParam(const std::string& param_name, bool value)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (param_name == "initial_rotation") {
        params_.initial_rotation = value;
        if (value && params_.allow_backward) {
            params_.allow_backward = false;
            std::cout << "[ParameterHandler] Warning: Initial rotation enabled, "
                      << "disabling backward motion." << std::endl;
        }
    } else if (param_name == "prefer_final_rotation") {
        params_.prefer_final_rotation = value;
    } else if (param_name == "allow_backward") {
        params_.allow_backward = value;
        if (value && params_.initial_rotation) {
            params_.initial_rotation = false;
            std::cout << "[ParameterHandler] Warning: Backward motion enabled, "
                      << "disabling initial rotation." << std::endl;
        }
    } else if (param_name == "use_collision_detection") {
        params_.use_collision_detection = value;
    } else {
        std::cerr << "[ParameterHandler] Unknown bool parameter: "
                  << param_name << std::endl;
        return;
    }

    validateParameters();
}

bool ParameterHandler::validateParameters()
{
    bool valid = true;
    std::stringstream errors;

    if (params_.min_lookahead < 0.01) {
        errors << "min_lookahead too small: " << params_.min_lookahead
               << " (min: 0.01)\n";
        valid = false;
    }

    if (params_.max_lookahead < params_.min_lookahead) {
        errors << "max_lookahead (" << params_.max_lookahead
               << ") < min_lookahead (" << params_.min_lookahead << ")\n";
        valid = false;
    }

    if (params_.k_phi < 0) {
        errors << "k_phi cannot be negative: " << params_.k_phi << "\n";
        valid = false;
    }

    if (params_.k_delta < 0) {
        errors << "k_delta cannot be negative: " << params_.k_delta << "\n";
        valid = false;
    }

    if (params_.beta <= 0) {
        errors << "beta must be positive: " << params_.beta << "\n";
        valid = false;
    }

    if (params_.lambda < 1) {
        errors << "lambda must be >= 1: " << params_.lambda << "\n";
        valid = false;
    }

    if (params_.v_linear_min < 0) {
        errors << "v_linear_min cannot be negative: " << params_.v_linear_min << "\n";
        valid = false;
    }

    if (params_.v_linear_max < params_.v_linear_min) {
        errors << "v_linear_max (" << params_.v_linear_max
               << ") < v_linear_min (" << params_.v_linear_min << ")\n";
        valid = false;
    }

    if (params_.v_angular_max <= 0) {
        errors << "v_angular_max must be positive: " << params_.v_angular_max << "\n";
        valid = false;
    }

    if (params_.v_angular_min_in_place < 0) {
        errors << "v_angular_min_in_place cannot be negative: "
               << params_.v_angular_min_in_place << "\n";
        valid = false;
    }

    if (params_.slowdown_radius < 0) {
        errors << "slowdown_radius cannot be negative: "
               << params_.slowdown_radius << "\n";
        valid = false;
    }

    if (params_.initial_rotation_tolerance <= 0) {
        errors << "initial_rotation_tolerance must be positive: "
               << params_.initial_rotation_tolerance << "\n";
        valid = false;
    }

    if (params_.rotation_scaling_factor < 0 || params_.rotation_scaling_factor > 1) {
        errors << "rotation_scaling_factor must be in [0, 1]: "
               << params_.rotation_scaling_factor << "\n";
        valid = false;
    }

    if (params_.in_place_collision_resolution <= 0) {
        errors << "in_place_collision_resolution must be positive: "
               << params_.in_place_collision_resolution << "\n";
        valid = false;
    }

    if (params_.collision_check_radius <= 0) {
        errors << "collision_check_radius must be positive: "
               << params_.collision_check_radius << "\n";
        valid = false;
    }

    if (params_.safety_distance < 0) {
        errors << "safety_distance cannot be negative: "
               << params_.safety_distance << "\n";
        valid = false;
    }

    if (!valid) {
        std::cerr << "[ParameterHandler] Parameter validation failed for plugin: "
                  << plugin_name_ << "\n" << errors.str();
    }

    return valid;
}

std::string ParameterHandler::getParametersString() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "Parameters for plugin: " << plugin_name_ << "\n";
    ss << "  min_lookahead: " << params_.min_lookahead << "\n";
    ss << "  max_lookahead: " << params_.max_lookahead << "\n";
    ss << "  k_phi: " << params_.k_phi << "\n";
    ss << "  k_delta: " << params_.k_delta << "\n";
    ss << "  beta: " << params_.beta << "\n";
    ss << "  lambda: " << params_.lambda << "\n";
    ss << "  v_linear_min: " << params_.v_linear_min << "\n";
    ss << "  v_linear_max: " << params_.v_linear_max << "\n";
    ss << "  v_angular_max: " << params_.v_angular_max << "\n";
    ss << "  v_angular_min_in_place: " << params_.v_angular_min_in_place << "\n";
    ss << "  slowdown_radius: " << params_.slowdown_radius << "\n";
    ss << "  initial_rotation: " << (params_.initial_rotation ? "true" : "false") << "\n";
    ss << "  initial_rotation_tolerance: " << params_.initial_rotation_tolerance << "\n";
    ss << "  prefer_final_rotation: " << (params_.prefer_final_rotation ? "true" : "false") << "\n";
    ss << "  rotation_scaling_factor: " << params_.rotation_scaling_factor << "\n";
    ss << "  allow_backward: " << (params_.allow_backward ? "true" : "false") << "\n";
    ss << "  in_place_collision_resolution: " << params_.in_place_collision_resolution << "\n";
    ss << "  use_collision_detection: " << (params_.use_collision_detection ? "true" : "false") << "\n";
    ss << "  collision_check_radius: " << params_.collision_check_radius << "\n";
    ss << "  safety_distance: " << params_.safety_distance << "\n";

    return ss.str();
}

LaserCollisionChecker::LaserCollisionChecker(double safety_distance, double check_radius)
        : safety_distance_(safety_distance), check_radius_(check_radius)
{
}

void LaserCollisionChecker::updateLaserScan(const LaserScan& scan, double robot_x, double robot_y, double robot_theta)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    obstacles_.clear();
    
    for (const auto& point : scan.points) {
        if (point.range < scan.range_min || point.range > scan.range_max) {
            continue;
        }
        
        obstacles_.push_back(Point2D(point.x, point.y));
    }
}

void LaserCollisionChecker::transformFootprintToWorld(double x, double y, double theta,
                                                       const Footprint& footprint,
                                                       std::vector<Point2D>& world_footprint)
{
    world_footprint.clear();
    world_footprint.reserve(footprint.size());
    
    double cos_th = std::cos(theta);
    double sin_th = std::sin(theta);
    
    for (const auto& point : footprint) {
        double wx = x + (point.x * cos_th - point.y * sin_th);
        double wy = y + (point.x * sin_th + point.y * cos_th);
        world_footprint.push_back(Point2D(wx, wy));
    }
}

bool LaserCollisionChecker::checkPointCollision(double wx, double wy, const Footprint& footprint, double theta)
{
    std::vector<Point2D> world_footprint;
    transformFootprintToWorld(wx, wy, theta, footprint, world_footprint);
    
    for (const auto& obstacle : obstacles_) {
        for (const auto& fp_point : world_footprint) {
            double dist = std::hypot(obstacle.x - fp_point.x, obstacle.y - fp_point.y);
            if (dist < safety_distance_) {
                return true;
            }
        }
        
        double dist_to_center = std::hypot(obstacle.x - wx, obstacle.y - wy);
        if (dist_to_center < check_radius_ + safety_distance_) {
            return true;
        }
    }
    
    return false;
}

bool LaserCollisionChecker::inCollision(double x, double y, double theta, const Footprint& footprint)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (obstacles_.empty()) {
        return false;
    }
    
    if (footprint.empty()) {
        for (const auto& obstacle : obstacles_) {
            double dist = std::hypot(obstacle.x - x, obstacle.y - y);
            if (dist < check_radius_ + safety_distance_) {
                return true;
            }
        }
        return false;
    }
    
    return checkPointCollision(x, y, footprint, theta);
}

namespace utils
{

Point2D circleSegmentIntersection(
        const Point2D & p1,
        const Point2D & p2,
        double r)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;

    double d2 = dx * dx + dy * dy;
    double det = p1.x * p2.y - p2.x * p1.y;

    double disc = r * r * d2 - det * det;
    if (disc < 0) {
        return p2;
    }

    double sqrt_disc = std::sqrt(disc);
    double sign_dy = dy < 0 ? -1.0 : 1.0;

    Point2D intersection;
    intersection.x = (det * dy + sign_dy * dx * sqrt_disc) / d2;
    intersection.y = (-det * dx + std::fabs(dy) * sqrt_disc) / d2;

    return intersection;
}

Point2D linearInterpolation(
        const Point2D & p1,
        const Point2D & p2,
        double target_dist)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dist = std::hypot(dx, dy);

    if (dist <= 1e-9) {
        return p1;
    }

    double ratio = target_dist / dist;
    Point2D result;
    result.x = p1.x + dx * ratio;
    result.y = p1.y + dy * ratio;

    return result;
}

Pose2D getLookAheadPoint(
        double lookahead_dist,
        const std::vector<Pose2D> & path,
        bool interpolate_after_goal)
{
    if (path.empty()) {
        return Pose2D(Point2D(0.0, 0.0), 0.0);
    }

    if (path.size() == 1) {
        return path[0];
    }

    double accumulated_dist = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double dx = path[i + 1].position.x - path[i].position.x;
        double dy = path[i + 1].position.y - path[i].position.y;
        double segment_dist = std::hypot(dx, dy);

        if (accumulated_dist + segment_dist >= lookahead_dist) {
            double remaining = lookahead_dist - accumulated_dist;
            Point2D point = linearInterpolation(
                    path[i].position, path[i + 1].position, remaining);

            double yaw = std::atan2(
                    path[i + 1].position.y - point.y,
                    path[i + 1].position.x - point.x);

            return Pose2D(point, yaw);
        }

        accumulated_dist += segment_dist;
    }

    if (interpolate_after_goal && path.size() >= 2) {
        const auto& last = path.back();
        const auto& second_last = path[path.size() - 2];

        double dx = last.position.x - second_last.position.x;
        double dy = last.position.y - second_last.position.y;
        double dist = std::hypot(dx, dy);
        double yaw = std::atan2(dy, dx);

        if (dist > 1e-9) {
            double extra_dist = lookahead_dist - accumulated_dist;
            Point2D point = linearInterpolation(
                    last.position,
                    Point2D(last.position.x + dx / dist, last.position.y + dy / dist),
                    extra_dist);

            return Pose2D(point, yaw);
        }
    }

    return path.back();
}

Pose2D orientationAroundZAxis(double angle)
{
    return Pose2D(Point2D(0.0, 0.0), angle);
}

double euclidean_distance(const Point2D & pos1, const Point2D & pos2)
{
    double dx = pos1.x - pos2.x;
    double dy = pos1.y - pos2.y;
    return std::hypot(dx, dy);
}

double euclidean_distance(const Pose2D & pos1, const Pose2D & pos2)
{
    return euclidean_distance(pos1.position, pos2.position);
}

double calculate_path_length(const std::vector<Pose2D> & path, size_t start_index)
{
    if (start_index + 1 >= path.size()) {
        return 0.0;
    }
    double path_length = 0.0;
    for (size_t idx = start_index; idx < path.size() - 1; ++idx) {
        path_length += euclidean_distance(path[idx], path[idx + 1]);
    }
    return path_length;
}

double distance_to_path_segment(
        const Point2D & point,
        const Pose2D & start,
        const Pose2D & end)
{
    const auto & p = point;
    const auto & a = start.position;
    const auto & b = end.position;

    const double dx_seg = b.x - a.x;
    const double dy_seg = b.y - a.y;
    const double seg_len_sq = dx_seg * dx_seg + dy_seg * dy_seg;

    if (seg_len_sq <= 1e-9) {
        return euclidean_distance(p, a);
    }

    const double t = std::clamp(
            ((p.x - a.x) * dx_seg + (p.y - a.y) * dy_seg) / seg_len_sq,
            0.0, 1.0);

    const double proj_x = a.x + t * dx_seg;
    const double proj_y = a.y + t * dy_seg;

    return std::hypot(p.x - proj_x, p.y - proj_y);
}

double cross_product_2d(
        const Point2D & point,
        const Pose2D & start,
        const Pose2D & end)
{
    const auto & p = point;
    const auto & a = start.position;
    const auto & b = end.position;

    const double path_vec_x = b.x - a.x;
    const double path_vec_y = b.y - a.y;
    const double robot_vec_x = p.x - a.x;
    const double robot_vec_y = p.y - a.y;

    return path_vec_x * robot_vec_y - path_vec_y * robot_vec_x;
}

PathSearchResult::PathSearchResult()
        : distance(std::numeric_limits<double>::max()), closest_segment_index(0) {}

PathSearchResult distance_from_path(
        const std::vector<Pose2D> & path,
        const Pose2D & robot_pose,
        size_t start_index,
        double search_window_length)
{
    PathSearchResult result;

    if (path.empty() || start_index >= path.size() - 1) {
        return result;
    }

    double accumulated_dist = 0.0;
    for (size_t i = start_index; i < path.size() - 1; ++i) {
        if (accumulated_dist > search_window_length) {
            break;
        }

        double segment_dist = euclidean_distance(
                path[i], path[i + 1]);

        double dist_to_segment = distance_to_path_segment(
                robot_pose.position, path[i], path[i + 1]);

        if (dist_to_segment < result.distance) {
            result.distance = dist_to_segment;
            result.closest_segment_index = i;
        }

        accumulated_dist += segment_dist;
    }

    return result;
}

size_t findFirstPathConstraint(
        std::vector<Pose2D> & path,
        bool enforce_path_inversion,
        float rotation_threshold)
{
    if (path.size() < 2) {
        return 0;
    }

    for (size_t i = 1; i < path.size(); ++i) {
        if (enforce_path_inversion) {
            double dx_prev = path[i].position.x - path[i-1].position.x;
            double dx_current = (i < path.size() - 1) ?
                                path[i+1].position.x - path[i].position.x : 0.0;

            if (dx_prev * dx_current < -1e-6) {
                return i;
            }
        }

        if (rotation_threshold > 0 && i < path.size() - 1) {
            double angle1 = path[i].orientation;
            double angle2 = path[i+1].orientation;

            double angle_diff = std::abs(angle1 - angle2);
            if (angle_diff > M_PI) {
                angle_diff = 2 * M_PI - angle_diff;
            }

            if (angle_diff > rotation_threshold) {
                return i + 1;
            }
        }
    }

    return 0;
}

size_t removePosesAfterFirstConstraint(
        std::vector<Pose2D> & path,
        bool enforce_path_inversion,
        float rotation_threshold)
{
    size_t constraint_index = findFirstPathConstraint(
            path, enforce_path_inversion, rotation_threshold);

    if (constraint_index > 0 && constraint_index < path.size()) {
        path.resize(constraint_index);
    }

    return constraint_index;
}

bool isPathUpdated(
        std::vector<Pose2D> & new_path,
        std::vector<Pose2D> & old_path)
{
    if (new_path.size() != old_path.size()) {
        return true;
    }

    for (size_t i = 0; i < new_path.size(); ++i) {
        double dist = euclidean_distance(
                new_path[i].position, old_path[i].position);

        if (dist > 1e-3) {
            return true;
        }

        double angle_diff = std::abs(new_path[i].orientation - old_path[i].orientation);
        if (angle_diff > 0.01) {
            return true;
        }
    }

    return false;
}

}

double GracefulController::euclidean_distance(const Point2D& p1, const Point2D& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::hypot(dx, dy);
}

double GracefulController::euclidean_distance(const Pose2D& p1, const Pose2D& p2) {
    return euclidean_distance(p1.position, p2.position);
}

GracefulController::GracefulController()
        : goal_dist_tolerance_(0.1),
          goal_reached_(false),
          do_initial_rotation_(false) {
}

GracefulController::~GracefulController() {
    cleanup();
}

void GracefulController::configure(const std::string& name) {
    plugin_name_ = name;

    param_handler_ = std::make_unique<ParameterHandler>(plugin_name_);
    params_ = param_handler_->getParams();

    control_law_ = std::make_unique<SmoothControlLaw>(
            params_.k_phi, params_.k_delta, params_.beta, params_.lambda,
            params_.slowdown_radius, params_.v_linear_min, params_.v_linear_max,
            params_.v_angular_max);

    goal_dist_tolerance_ = 0.1;
    goal_reached_ = false;
    do_initial_rotation_ = false;
    
    if (params_.use_collision_detection) {
        auto laser_checker = std::make_shared<LaserCollisionChecker>(
            params_.safety_distance, params_.collision_check_radius);
        collision_checker_ = laser_checker;
    }
}

void GracefulController::setCollisionChecker(std::shared_ptr<ICollisionChecker> collision_checker) {
    collision_checker_ = collision_checker;
}

void GracefulController::setRobotFootprint(const Footprint& footprint) {
    robot_footprint_ = footprint;
}

void GracefulController::updateLaserScan(const LaserScan& scan, double robot_x, double robot_y, double robot_theta) {
    if (collision_checker_) {
        collision_checker_->updateLaserScan(scan, robot_x, robot_y, robot_theta);
    }
}

Twist GracefulController::computeVelocityCommands(
        const Pose2DStamped & pose,
        const Path & transformed_plan,
        std::vector<Pose2D>& trajectory_out) {

    if (!param_handler_) {
        throw ControllerException("Parameter handler is not initialized");
    }
    if (!control_law_) {
        throw ControllerException("Control law is not initialized");
    }

    std::lock_guard<std::mutex> lock(param_handler_->getMutex());

    Twist cmd_vel;
    trajectory_out.clear();

    if (params_.use_collision_detection && !collision_checker_) {
        throw ControllerException("No collision checker available for collision detection");
    }

    control_law_->setCurvatureConstants(
            params_.k_phi, params_.k_delta, params_.beta, params_.lambda);
    control_law_->setSlowdownRadius(params_.slowdown_radius);
    control_law_->setSpeedLimit(
            params_.v_linear_min, params_.v_linear_max, params_.v_angular_max);

    std::vector<Pose2D> plan_poses = transformed_plan.poses;
    validateOrientations(plan_poses);

    double dist_to_goal = 0.0;
    if (!plan_poses.empty()) {
        dist_to_goal = euclidean_distance(Point2D(0, 0), plan_poses.back().position);
    }

    if (dist_to_goal < goal_dist_tolerance_ || goal_reached_) {
        goal_reached_ = true;

        if (plan_poses.empty()) {
            return cmd_vel;
        }

        double angle_to_goal = plan_poses.back().orientation;
        angle_to_goal = normalizeAngle(angle_to_goal);

        size_t num_steps = static_cast<size_t>(fabs(angle_to_goal) / params_.in_place_collision_resolution);
        num_steps = std::max(static_cast<size_t>(1), num_steps);
        bool collision_free = true;

        for (size_t i = 1; i <= num_steps; ++i) {
            double step = static_cast<double>(i) / static_cast<double>(num_steps);
            double yaw = step * angle_to_goal;

            if (params_.use_collision_detection &&
                inCollision(0.0, 0.0, yaw)) {
                collision_free = false;
                break;
            }
        }

        if (collision_free) {
            cmd_vel = rotateToTarget(angle_to_goal);
            return cmd_vel;
        }
    }

    std::vector<Pose2D> trajectory;
    Pose2D target_pose;

    double lookahead_dist = std::min(params_.max_lookahead, dist_to_goal);
    lookahead_dist = std::max(lookahead_dist, params_.min_lookahead);
    
    double min_dist_to_robot = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    for (size_t i = 0; i < plan_poses.size(); ++i) {
        double d = euclidean_distance(Point2D(0, 0), plan_poses[i].position);
        if (d < min_dist_to_robot) {
            min_dist_to_robot = d;
            closest_idx = i;
        }
    }
    
    for (size_t i = closest_idx; i < plan_poses.size(); ++i) {
        if (plan_poses[i].position.x >= 0) {
            closest_idx = i;
            break;
        }
    }
    
    double dist_along_path = 0.0;
    size_t target_idx = closest_idx;
    bool found_target = false;
    
    for (size_t i = closest_idx; i < plan_poses.size() - 1; ++i) {
        double segment_dist = euclidean_distance(plan_poses[i].position, plan_poses[i + 1].position);
        if (dist_along_path + segment_dist >= lookahead_dist) {
            double remaining = lookahead_dist - dist_along_path;
            double ratio = (segment_dist > 1e-6) ? remaining / segment_dist : 0.0;
            
            target_pose.position.x = plan_poses[i].position.x + 
                ratio * (plan_poses[i + 1].position.x - plan_poses[i].position.x);
            target_pose.position.y = plan_poses[i].position.y + 
                ratio * (plan_poses[i + 1].position.y - plan_poses[i].position.y);
            target_pose.orientation = std::atan2(
                plan_poses[i + 1].position.y - plan_poses[i].position.y,
                plan_poses[i + 1].position.x - plan_poses[i].position.x);
            
            found_target = true;
            break;
        }
        dist_along_path += segment_dist;
        target_idx = i + 1;
    }
    
    if (!found_target) {
        target_pose = plan_poses.back();
    }
    
    double dist_to_target = euclidean_distance(Point2D(0, 0), target_pose.position);
    
    std::cout << "[GracefulController] lookahead=" << lookahead_dist 
              << ", closest_idx=" << closest_idx
              << ", target=(" << target_pose.position.x << "," << target_pose.position.y << ")"
              << ", dist_to_target=" << dist_to_target
              << ", dist_to_goal=" << dist_to_goal << std::endl;

    if (validateTargetPose(target_pose, dist_to_target, dist_to_goal, trajectory, cmd_vel)) {
        trajectory_out = trajectory;
        return cmd_vel;
    }

    throw NoValidControl("No valid control found - collision detected in all trajectories");
}

bool GracefulController::validateTargetPose(
        Pose2D & target_pose,
        double dist_to_target,
        double dist_to_goal,
        std::vector<Pose2D> & trajectory,
        Twist & cmd_vel) {
    if (dist_to_goal < params_.max_lookahead) {
        if (params_.prefer_final_rotation) {
            double dx = target_pose.position.x;
            double dy = target_pose.position.y;
            if (std::hypot(dx, dy) > 0.01) {
                target_pose.orientation = std::atan2(dy, dx);
            }
        }
    }

    bool reversing = false;
    if (params_.allow_backward && target_pose.position.x < 0.0) {
        reversing = true;
        target_pose.orientation += M_PI;
    }

    if (simulateTrajectory(target_pose, trajectory, cmd_vel, reversing)) {
        return true;
    }

    return false;
}

bool GracefulController::simulateTrajectory(
        const Pose2D & motion_target,
        std::vector<Pose2D> & trajectory,
        Twist & cmd_vel,
        bool backward) {
    trajectory.clear();

    Pose2D next_pose;
    next_pose.position = Point2D(0.0, 0.0);
    next_pose.orientation = 0.0;

    bool sim_initial_rotation = do_initial_rotation_ && params_.initial_rotation;
    double angle_to_target = std::atan2(motion_target.position.y, motion_target.position.x);

    if (fabs(angle_to_target) < params_.initial_rotation_tolerance) {
        sim_initial_rotation = false;
        do_initial_rotation_ = false;
    }

    double distance = std::numeric_limits<double>::max();
    double resolution = 0.05;
    
    if (collision_checker_) {
        resolution = collision_checker_->getResolution();
    }
    
    double dt = (params_.v_linear_max > 0.0) ? resolution / params_.v_linear_max : 0.05;

    double target_distance = euclidean_distance(motion_target.position, next_pose.position);
    unsigned int max_iter = static_cast<unsigned int>(3 * target_distance / resolution);

    for (unsigned int iter = 0; iter < max_iter; ++iter) {
        if (sim_initial_rotation) {
            double next_pose_yaw = next_pose.orientation;
            auto cmd = rotateToTarget(angle_to_target - next_pose_yaw);

            if (trajectory.empty()) {
                cmd_vel = cmd;
            }

            if (fabs(angle_to_target - next_pose_yaw) < params_.initial_rotation_tolerance) {
                sim_initial_rotation = false;
            }

            next_pose_yaw += cmd_vel.angular_z * dt;
            next_pose.orientation = next_pose_yaw;
        } else {
            if (trajectory.empty()) {
                cmd_vel = control_law_->calculateRegularVelocity(
                        motion_target, next_pose, backward);
            }

            next_pose = control_law_->calculateNextPose(dt, motion_target, next_pose, backward);
        }

        if (params_.use_collision_detection &&
            inCollision(next_pose.position.x, next_pose.position.y, next_pose.orientation)) {
            return false;
        }

        trajectory.push_back(next_pose);

        distance = euclidean_distance(motion_target.position, next_pose.position);
        if (distance <= resolution) {
            break;
        }
    }

    return true;
}

bool GracefulController::inCollision(double x, double y, double theta) {
    if (!collision_checker_) {
        return false;
    }

    return collision_checker_->inCollision(x, y, theta, robot_footprint_);
}

Twist GracefulController::rotateToTarget(double angle_to_target) {
    Twist vel;
    vel.linear_x = 0.0;
    vel.angular_z = params_.rotation_scaling_factor * angle_to_target * params_.v_angular_max;

    double min_vel = params_.v_angular_min_in_place;
    if (fabs(vel.angular_z) < min_vel) {
        vel.angular_z = (vel.angular_z >= 0) ? min_vel : -min_vel;
    }

    double max_vel = params_.v_angular_max;
    if (fabs(vel.angular_z) > max_vel) {
        vel.angular_z = (vel.angular_z >= 0) ? max_vel : -max_vel;
    }

    return vel;
}

void GracefulController::computeDistanceAlongPath(
        const std::vector<Pose2D> & poses,
        std::vector<double> & distances) {
    distances.resize(poses.size());
    if (poses.empty()) return;

    double d = euclidean_distance(Point2D(0, 0), poses[0].position);
    distances[0] = d;

    for (size_t i = 1; i < poses.size(); ++i) {
        d += euclidean_distance(poses[i - 1].position, poses[i].position);
        distances[i] = d;
    }
}

void GracefulController::validateOrientations(std::vector<Pose2D> & path) {
    if (path.size() < 3) return;

    double initial_yaw = path[1].orientation;
    for (size_t i = 2; i < path.size() - 1; ++i) {
        double this_yaw = path[i].orientation;
        double diff = this_yaw - initial_yaw;
        diff = normalizeAngle(diff);

        if (fabs(diff) > 1e-6) {
            return;
        }
    }

    for (size_t i = 0; i < path.size() - 1; ++i) {
        double dx = path[i + 1].position.x - path[i].position.x;
        double dy = path[i + 1].position.y - path[i].position.y;
        double yaw = std::atan2(dy, dx);
        path[i].orientation = yaw;
    }

    if (path.size() > 1) {
        path.back().orientation = path[path.size() - 2].orientation;
    }
}

void GracefulController::newPathReceived(const Path & /*raw_global_path*/) {
    goal_reached_ = false;
    do_initial_rotation_ = true;
}

void GracefulController::setSpeedLimit(double speed_limit, bool percentage) {
    std::lock_guard<std::mutex> lock(param_handler_->getMutex());

    constexpr double NO_SPEED_LIMIT = std::numeric_limits<double>::max();

    if (speed_limit == NO_SPEED_LIMIT) {
        params_.v_linear_max = params_.v_linear_max_initial;
        params_.v_angular_max = params_.v_angular_max_initial;
    } else {
        if (percentage) {
            params_.v_linear_max = std::max(
                    params_.v_linear_max_initial * speed_limit / 100.0,
                    params_.v_linear_min);
            params_.v_angular_max = params_.v_angular_max_initial * speed_limit / 100.0;
        } else {
            params_.v_linear_max = std::max(speed_limit, params_.v_linear_min);
            if (params_.v_linear_max_initial > std::numeric_limits<double>::epsilon()) {
                params_.v_angular_max = params_.v_angular_max_initial *
                                        speed_limit / params_.v_linear_max_initial;
            } else {
                params_.v_angular_max = params_.v_angular_max_initial;
            }
        }
    }
}

void GracefulController::setParameters(const Parameters& params) {
    std::lock_guard<std::mutex> lock(param_handler_->getMutex());
    params_ = params;
    
    if (control_law_) {
        control_law_->setCurvatureConstants(
                params_.k_phi, params_.k_delta, params_.beta, params_.lambda);
        control_law_->setSlowdownRadius(params_.slowdown_radius);
        control_law_->setSpeedLimit(
                params_.v_linear_min, params_.v_linear_max, params_.v_angular_max);
    }
}

Parameters GracefulController::getParameters() const {
    return param_handler_->getParams();
}

void GracefulController::cleanup() {
    param_handler_.reset();
    control_law_.reset();
    collision_checker_.reset();
}

void GracefulController::activate() {
    std::lock_guard<std::mutex> lock(param_handler_->getMutex());

    std::cout << "Activating Graceful Controller: " << plugin_name_ << std::endl;

    goal_reached_ = false;
    do_initial_rotation_ = false;

    params_.v_linear_max = params_.v_linear_max_initial;
    params_.v_angular_max = params_.v_angular_max_initial;

    if (control_law_) {
        control_law_->setSpeedLimit(
                params_.v_linear_min,
                params_.v_linear_max,
                params_.v_angular_max);
    }

    std::cout << "Graceful Controller activated: " << plugin_name_ << std::endl;
}

void GracefulController::deactivate() {
    std::lock_guard<std::mutex> lock(param_handler_->getMutex());

    std::cout << "Deactivating Graceful Controller: " << plugin_name_ << std::endl;

    goal_reached_ = true;
    do_initial_rotation_ = false;

    std::cout << "Graceful Controller deactivated: " << plugin_name_ << std::endl;
}

}
