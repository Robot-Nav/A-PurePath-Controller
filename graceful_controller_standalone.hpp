#ifndef GRACEFUL_CONTROLLER_STANDALONE_HPP_
#define GRACEFUL_CONTROLLER_STANDALONE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <limits>
#include <iostream>
#include <sstream>
#include <iomanip>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace graceful_controller
{

inline double normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

struct Point2D {
    double x;
    double y;

    Point2D() : x(0.0), y(0.0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}

    Point2D operator-(const Point2D& other) const {
        return Point2D(x - other.x, y - other.y);
    }

    double length() const {
        return std::hypot(x, y);
    }
};

struct Pose2D {
    Point2D position;
    double orientation;

    Pose2D() : position(), orientation(0.0) {}
    Pose2D(const Point2D& pos, double yaw) : position(pos), orientation(yaw) {}
};

struct Twist {
    double linear_x;
    double linear_y;
    double linear_z;
    double angular_x;
    double angular_y;
    double angular_z;

    Twist() : linear_x(0), linear_y(0), linear_z(0),
              angular_x(0), angular_y(0), angular_z(0) {}
};

struct Pose2DStamped {
    Pose2D pose;
    uint64_t timestamp;

    Pose2DStamped() : timestamp(0) {}
    Pose2DStamped(const Pose2D& p, uint64_t t) : pose(p), timestamp(t) {}
};

struct Path {
    std::vector<Pose2D> poses;

    size_t size() const { return poses.size(); }
    bool empty() const { return poses.empty(); }
    const Pose2D& back() const { return poses.back(); }
    Pose2D& back() { return poses.back(); }
    void clear() { poses.clear(); }
    void push_back(const Pose2D& pose) { poses.push_back(pose); }
    void push_back(Pose2D&& pose) { poses.push_back(std::move(pose)); }
};

struct WorldPoint {
    double x;
    double y;

    WorldPoint() : x(0.0), y(0.0) {}
    WorldPoint(double x_, double y_) : x(x_), y(y_) {}
};

using Footprint = std::vector<WorldPoint>;

struct LaserPoint {
    double angle;
    double range;
    double x;
    double y;

    LaserPoint() : angle(0.0), range(0.0), x(0.0), y(0.0) {}
    LaserPoint(double a, double r) : angle(a), range(r) {
        x = range * std::cos(angle);
        y = range * std::sin(angle);
    }
};

struct LaserScan {
    std::vector<LaserPoint> points;
    double angle_min;
    double angle_max;
    double angle_increment;
    double range_min;
    double range_max;
    double timestamp;

    LaserScan() : angle_min(0), angle_max(0), angle_increment(0),
                  range_min(0), range_max(0), timestamp(0) {}

    bool empty() const { return points.empty(); }
    size_t size() const { return points.size(); }
};

struct Parameters {
    double min_lookahead;
    double max_lookahead;
    double k_phi;
    double k_delta;
    double beta;
    double lambda;
    double v_linear_min;
    double v_linear_max;
    double v_linear_max_initial;
    double v_angular_max;
    double v_angular_max_initial;
    double v_angular_min_in_place;
    double slowdown_radius;
    bool initial_rotation;
    double initial_rotation_tolerance;
    bool prefer_final_rotation;
    double rotation_scaling_factor;
    bool allow_backward;
    double in_place_collision_resolution;
    bool use_collision_detection;
    double collision_check_radius;
    double safety_distance;

    Parameters() {
        min_lookahead = 0.25;
        max_lookahead = 1.0;
        k_phi = 2.0;
        k_delta = 1.0;
        beta = 0.4;
        lambda = 2.0;
        v_linear_min = 0.1;
        v_linear_max = 0.5;
        v_linear_max_initial = 0.5;
        v_angular_max = 1.0;
        v_angular_max_initial = 1.0;
        v_angular_min_in_place = 0.25;
        slowdown_radius = 1.5;
        initial_rotation = true;
        initial_rotation_tolerance = 0.75;
        prefer_final_rotation = true;
        rotation_scaling_factor = 0.5;
        allow_backward = false;
        in_place_collision_resolution = 0.1;
        use_collision_detection = true;
        collision_check_radius = 0.5;
        safety_distance = 0.3;
    }
};

class ControllerException : public std::runtime_error
{
public:
    explicit ControllerException(const std::string & description)
            : std::runtime_error(description) {}
};

class InvalidController : public ControllerException
{
public:
    explicit InvalidController(const std::string & description)
            : ControllerException(description) {}
};

class ControllerTFError : public ControllerException
{
public:
    explicit ControllerTFError(const std::string & description)
            : ControllerException(description) {}
};

class FailedToMakeProgress : public ControllerException
{
public:
    explicit FailedToMakeProgress(const std::string & description)
            : ControllerException(description) {}
};

class PatienceExceeded : public ControllerException
{
public:
    explicit PatienceExceeded(const std::string & description)
            : ControllerException(description) {}
};

class InvalidPath : public ControllerException
{
public:
    explicit InvalidPath(const std::string & description)
            : ControllerException(description) {}
};

class NoValidControl : public ControllerException
{
public:
    explicit NoValidControl(const std::string & description)
            : ControllerException(description) {}
};

class ControllerTimedOut : public ControllerException
{
public:
    explicit ControllerTimedOut(const std::string & description)
            : ControllerException(description) {}
};

struct EgocentricPolarCoordinates
{
    double r;
    double phi;
    double delta;

    EgocentricPolarCoordinates(
            double r_in = 0.0,
            double phi_in = 0.0,
            double delta_in = 0.0)
            : r(r_in), phi(phi_in), delta(delta_in) {}

    explicit EgocentricPolarCoordinates(
            const Pose2D & target,
            const Pose2D & current = Pose2D(),
            bool backward = false)
    {
        double dX = target.position.x - current.position.x;
        double dY = target.position.y - current.position.y;

        r = std::hypot(dX, dY);

        if (r < std::numeric_limits<double>::epsilon()) {
            r = 0.0;
            phi = 0.0;
            delta = 0.0;
            return;
        }

        double line_of_sight = std::atan2(dY, dX);

        if (backward) {
            line_of_sight = normalizeAngle(line_of_sight + M_PI);
        }

        phi = normalizeAngle(target.orientation - line_of_sight);
        delta = normalizeAngle(current.orientation - line_of_sight);
    }

    bool isValid() const {
        return r >= 0 && !std::isnan(r) && !std::isnan(phi) && !std::isnan(delta);
    }

    std::string toString() const {
        return "r=" + std::to_string(r) +
               ", phi=" + std::to_string(phi) +
               ", delta=" + std::to_string(delta);
    }
};

class SmoothControlLaw
{
public:
    SmoothControlLaw(
            double k_phi, double k_delta, double beta, double lambda, double slowdown_radius,
            double v_linear_min, double v_linear_max, double v_angular_max);

    ~SmoothControlLaw() = default;

    void setCurvatureConstants(
            double k_phi, double k_delta, double beta, double lambda);

    void setSlowdownRadius(double slowdown_radius);

    void setSpeedLimit(
            const double v_linear_min, const double v_linear_max, const double v_angular_max);

    Twist calculateRegularVelocity(
            const Pose2D & target, const Pose2D & current, bool backward);

    Twist calculateRegularVelocity(
            const Pose2D & target, bool backward);

    Pose2D calculateNextPose(
            const double dt,
            const Pose2D & target,
            const Pose2D & current,
            bool backward);

    double calculateCurvature(double r, double phi, double delta);

    double getMaxLinearSpeed() const { return v_linear_max_; }
    double getMinLinearSpeed() const { return v_linear_min_; }
    double getMaxAngularSpeed() const { return v_angular_max_; }
    double getSlowdownRadius() const { return slowdown_radius_; }

private:
    double k_phi_;
    double k_delta_;
    double beta_;
    double lambda_;
    double slowdown_radius_;
    double v_linear_min_;
    double v_linear_max_;
    double v_angular_max_;
};

class ParameterHandler
{
public:
    explicit ParameterHandler(const std::string& plugin_name);
    ~ParameterHandler() = default;

    Parameters getParams() const;
    void updateParams(const Parameters& new_params);

    void updateParam(const std::string& param_name, double value);
    void updateParam(const std::string& param_name, bool value);

    std::string getParametersString() const;
    
    std::mutex& getMutex() const { return mutex_; }

private:
    bool validateParameters();

private:
    std::string plugin_name_;
    mutable std::mutex mutex_;
    Parameters params_;
};

class ICollisionChecker
{
public:
    virtual ~ICollisionChecker() = default;
    virtual bool inCollision(double x, double y, double theta, const Footprint& footprint) = 0;
    virtual double getResolution() const = 0;
    virtual void updateLaserScan(const LaserScan& scan, double robot_x, double robot_y, double robot_theta) = 0;
};

class LaserCollisionChecker : public ICollisionChecker
{
public:
    explicit LaserCollisionChecker(double safety_distance = 0.3, double check_radius = 0.5);
    ~LaserCollisionChecker() = default;

    bool inCollision(double x, double y, double theta, const Footprint& footprint) override;
    double getResolution() const override { return 0.05; }
    
    void updateLaserScan(const LaserScan& scan, double robot_x, double robot_y, double robot_theta) override;
    
    void setSafetyDistance(double distance) { safety_distance_ = distance; }
    void setCheckRadius(double radius) { check_radius_ = radius; }
    
    void clearObstacles() { obstacles_.clear(); }

private:
    bool checkPointCollision(double wx, double wy, const Footprint& footprint, double theta);
    void transformFootprintToWorld(double x, double y, double theta, 
                                   const Footprint& footprint,
                                   std::vector<Point2D>& world_footprint);

private:
    double safety_distance_;
    double check_radius_;
    std::vector<Point2D> obstacles_;
    std::mutex mutex_;
};

namespace utils
{

Point2D circleSegmentIntersection(
        const Point2D & p1,
        const Point2D & p2,
        double r);

Point2D linearInterpolation(
        const Point2D & p1,
        const Point2D & p2,
        double target_dist);

Pose2D getLookAheadPoint(
        double lookahead_dist,
        const std::vector<Pose2D> & path,
        bool interpolate_after_goal = false);

Pose2D orientationAroundZAxis(double angle);
double euclidean_distance(const Point2D & pos1, const Point2D & pos2);
double euclidean_distance(const Pose2D & pos1, const Pose2D & pos2);
double calculate_path_length(const std::vector<Pose2D> & path, size_t start_index = 0);

template<class PointT>
bool isPointInsidePolygon(
        const double px, const double py, const std::vector<PointT> & polygon)
{
    if (polygon.size() < 3) return false;

    bool inside = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        if (((polygon[i].y > py) != (polygon[j].y > py)) &&
            (px < (polygon[j].x - polygon[i].x) * (py - polygon[i].y) /
                  (polygon[j].y - polygon[i].y) + polygon[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

double distance_to_path_segment(
        const Point2D & point,
        const Pose2D & start,
        const Pose2D & end);

double cross_product_2d(
        const Point2D & point,
        const Pose2D & start,
        const Pose2D & end);

template<typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
    if (begin == end) {
        return end;
    }
    auto lowest = getCompareVal(*begin);
    Iter lowest_it = begin;
    for (Iter it = ++begin; it != end; ++it) {
        auto comp = getCompareVal(*it);
        if (comp < lowest) {
            lowest = comp;
            lowest_it = it;
        }
    }
    return lowest_it;
}

struct PathSearchResult
{
    double distance;
    size_t closest_segment_index;

    PathSearchResult();
};

PathSearchResult distance_from_path(
        const std::vector<Pose2D> & path,
        const Pose2D & robot_pose,
        size_t start_index = 0,
        double search_window_length = std::numeric_limits<double>::max());

size_t findFirstPathConstraint(
        std::vector<Pose2D> & path,
        bool enforce_path_inversion,
        float rotation_threshold);

size_t removePosesAfterFirstConstraint(
        std::vector<Pose2D> & path,
        bool enforce_path_inversion,
        float rotation_threshold);

bool isPathUpdated(
        std::vector<Pose2D> & new_path,
        std::vector<Pose2D> & old_path);

}

class GracefulController
{
public:
    GracefulController();
    ~GracefulController();

    void configure(const std::string& name);
    void cleanup();
    void activate();
    void deactivate();

    void setCollisionChecker(std::shared_ptr<ICollisionChecker> collision_checker);
    void setRobotFootprint(const Footprint& footprint);
    
    void updateLaserScan(const LaserScan& scan, double robot_x, double robot_y, double robot_theta);

    Twist computeVelocityCommands(
            const Pose2DStamped & pose,
            const Path & transformed_plan,
            std::vector<Pose2D>& trajectory_out);

    void newPathReceived(const Path & raw_global_path);
    void setSpeedLimit(double speed_limit, bool percentage);

    void setParameters(const Parameters& params);
    Parameters getParameters() const;

private:
    bool validateTargetPose(
            Pose2D & target_pose,
            double dist_to_target,
            double dist_to_goal,
            std::vector<Pose2D> & trajectory,
            Twist & cmd_vel);

    bool simulateTrajectory(
            const Pose2D & motion_target,
            std::vector<Pose2D> & trajectory,
            Twist & cmd_vel,
            bool backward);

    Twist rotateToTarget(double angle_to_target);
    bool inCollision(double x, double y, double theta);

    void computeDistanceAlongPath(
            const std::vector<Pose2D> & poses,
            std::vector<double> & distances);

    void validateOrientations(std::vector<Pose2D> & path);

    static double euclidean_distance(const Point2D& p1, const Point2D& p2);
    static double euclidean_distance(const Pose2D& p1, const Pose2D& p2);

private:
    std::string plugin_name_;
    std::shared_ptr<ICollisionChecker> collision_checker_;
    Footprint robot_footprint_;

    Parameters params_;
    double goal_dist_tolerance_;
    bool goal_reached_;
    bool do_initial_rotation_;

    std::unique_ptr<ParameterHandler> param_handler_;
    std::unique_ptr<SmoothControlLaw> control_law_;
};

}

#endif
