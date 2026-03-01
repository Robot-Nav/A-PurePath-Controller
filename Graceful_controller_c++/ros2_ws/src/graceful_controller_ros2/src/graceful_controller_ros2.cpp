#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  

#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "graceful_controller_standalone.hpp"

using namespace graceful_controller;

class GracefulControllerNode : public rclcpp::Node
{
public:
    GracefulControllerNode() : Node("graceful_controller_node")
    {
        declareParameters();
        
        controller_ = std::make_unique<GracefulController>();
        controller_->configure("graceful_controller");
        
        loadParameters();
        initializeFootprint();
        
        controller_->setRobotFootprint(robot_footprint_);
        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("local_trajectory", 10);
        obstacle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles", 10);
        
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", 10,
            std::bind(&GracefulControllerNode::pathCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&GracefulControllerNode::odomCallback, this, std::placeholders::_1));
        
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&GracefulControllerNode::laserCallback, this, std::placeholders::_1));
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
            std::bind(&GracefulControllerNode::controlLoop, this));
        
        current_path_.clear();
        path_received_ = false;
        odom_received_ = false;
        laser_received_ = false;
        goal_reached_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Graceful Controller Node initialized (Laser-based collision detection)");
        RCLCPP_INFO(this->get_logger(), "Publishing local trajectory to 'local_trajectory' topic");
    }

    ~GracefulControllerNode()
    {
        controller_->deactivate();
        controller_->cleanup();
    }

private:
    void declareParameters()
    {
        this->declare_parameter("control_frequency", 20.0);
        this->declare_parameter("goal_dist_tolerance", 0.1);
        this->declare_parameter("goal_angle_tolerance", 0.1);
        
        this->declare_parameter("lookahead.min_lookahead", 0.25);
        this->declare_parameter("lookahead.max_lookahead", 1.0);
        
        this->declare_parameter("control_params.k_phi", 2.0);
        this->declare_parameter("control_params.k_delta", 1.0);
        this->declare_parameter("control_params.beta", 0.4);
        this->declare_parameter("control_params.lambda", 2.0);
        
        this->declare_parameter("velocity.v_linear_min", 0.1);
        this->declare_parameter("velocity.v_linear_max", 0.5);
        this->declare_parameter("velocity.v_angular_max", 1.0);
        this->declare_parameter("velocity.v_angular_min_in_place", 0.25);
        
        this->declare_parameter("behavior.slowdown_radius", 1.5);
        this->declare_parameter("behavior.initial_rotation", false);
        this->declare_parameter("behavior.initial_rotation_tolerance", 0.75);
        this->declare_parameter("behavior.prefer_final_rotation", true);
        this->declare_parameter("behavior.rotation_scaling_factor", 0.5);
        this->declare_parameter("behavior.allow_backward", false);
        
        this->declare_parameter("collision.use_collision_detection", true);
        this->declare_parameter("collision.in_place_collision_resolution", 0.1);
        this->declare_parameter("collision.collision_check_radius", 0.5);
        this->declare_parameter("collision.safety_distance", 0.3);
        
        this->declare_parameter("robot_footprint", std::vector<double>{0.3, 0.3, -0.3, 0.3, -0.3, -0.3, 0.3, -0.3});
    }

    void loadParameters()
    {
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        goal_dist_tolerance_ = this->get_parameter("goal_dist_tolerance").as_double();
        goal_angle_tolerance_ = this->get_parameter("goal_angle_tolerance").as_double();
        
        Parameters params;
        
        params.min_lookahead = this->get_parameter("lookahead.min_lookahead").as_double();
        params.max_lookahead = this->get_parameter("lookahead.max_lookahead").as_double();
        
        params.k_phi = this->get_parameter("control_params.k_phi").as_double();
        params.k_delta = this->get_parameter("control_params.k_delta").as_double();
        params.beta = this->get_parameter("control_params.beta").as_double();
        params.lambda = this->get_parameter("control_params.lambda").as_double();
        
        params.v_linear_min = this->get_parameter("velocity.v_linear_min").as_double();
        params.v_linear_max = this->get_parameter("velocity.v_linear_max").as_double();
        params.v_angular_max = this->get_parameter("velocity.v_angular_max").as_double();
        params.v_angular_min_in_place = this->get_parameter("velocity.v_angular_min_in_place").as_double();
        params.v_linear_max_initial = params.v_linear_max;
        params.v_angular_max_initial = params.v_angular_max;
        
        params.slowdown_radius = this->get_parameter("behavior.slowdown_radius").as_double();
        params.initial_rotation = this->get_parameter("behavior.initial_rotation").as_bool();
        params.initial_rotation_tolerance = this->get_parameter("behavior.initial_rotation_tolerance").as_double();
        params.prefer_final_rotation = this->get_parameter("behavior.prefer_final_rotation").as_bool();
        params.rotation_scaling_factor = this->get_parameter("behavior.rotation_scaling_factor").as_double();
        params.allow_backward = this->get_parameter("behavior.allow_backward").as_bool();
        
        params.use_collision_detection = this->get_parameter("collision.use_collision_detection").as_bool();
        params.in_place_collision_resolution = this->get_parameter("collision.in_place_collision_resolution").as_double();
        params.collision_check_radius = this->get_parameter("collision.collision_check_radius").as_double();
        params.safety_distance = this->get_parameter("collision.safety_distance").as_double();
        
        controller_->setParameters(params);
        
        RCLCPP_INFO(this->get_logger(), "Parameters loaded from ROS2 parameter system");
        RCLCPP_INFO(this->get_logger(), "  lookahead: [%.2f, %.2f]", params.min_lookahead, params.max_lookahead);
        RCLCPP_INFO(this->get_logger(), "  velocity: linear=[%.2f, %.2f], angular=%.2f", 
                    params.v_linear_min, params.v_linear_max, params.v_angular_max);
        RCLCPP_INFO(this->get_logger(), "  collision: check_radius=%.2f, safety_distance=%.2f",
                    params.collision_check_radius, params.safety_distance);
        RCLCPP_INFO(this->get_logger(), "  initial_rotation: %s", params.initial_rotation ? "true" : "false");
    }

    void initializeFootprint()
    {
        auto footprint_params = this->get_parameter("robot_footprint").as_double_array();
        
        if (footprint_params.size() >= 8 && footprint_params.size() % 2 == 0)
        {
            for (size_t i = 0; i < footprint_params.size(); i += 2)
            {
                WorldPoint point;
                point.x = footprint_params[i];
                point.y = footprint_params[i + 1];
                robot_footprint_.push_back(point);
            }
            RCLCPP_INFO(this->get_logger(), "Robot footprint initialized with %zu points", robot_footprint_.size());
        }
        else
        {
            WorldPoint p1{0.3, 0.3};
            WorldPoint p2{-0.3, 0.3};
            WorldPoint p3{-0.3, -0.3};
            WorldPoint p4{0.3, -0.3};
            robot_footprint_ = {p1, p2, p3, p4};
            RCLCPP_WARN(this->get_logger(), "Using default rectangular footprint");
        }
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty path");
            return;
        }
        
        current_path_.clear();
        
        for (const auto& pose_stamped : msg->poses)
        {
            Pose2D pose;
            pose.position.x = pose_stamped.pose.position.x;
            pose.position.y = pose_stamped.pose.position.y;
            pose.orientation = getYawFromQuaternion(pose_stamped.pose.orientation);
            current_path_.push_back(pose);
        }
        
        controller_->newPathReceived(current_path_);
        path_received_ = true;
        goal_reached_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Received new path with %zu poses", current_path_.size());
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;
        odom_received_ = true;
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!odom_received_) {
            return;
        }
        
        LaserScan scan;
        scan.angle_min = msg->angle_min;
        scan.angle_max = msg->angle_max;
        scan.angle_increment = msg->angle_increment;
        scan.range_min = msg->range_min;
        scan.range_max = msg->range_max;
        scan.timestamp = msg->header.stamp.nanosec + msg->header.stamp.sec * 1000000000ULL;
        
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double angle = msg->angle_min + i * msg->angle_increment;
            double range = msg->ranges[i];
            
            if (range >= msg->range_min && range <= msg->range_max) {
                LaserPoint point(angle, range);
                scan.points.push_back(point);
            }
        }
        
        controller_->updateLaserScan(scan, 0.0, 0.0, 0.0);
        
        obstacles_.clear();
        for (const auto& point : scan.points) {
            obstacles_.push_back(Point2D(point.x, point.y));
        }
        
        laser_received_ = true;
    }

    void controlLoop()
    {
        if (!path_received_ || !odom_received_)
        {
            return;
        }
        
        if (goal_reached_)
        {
            publishZeroVelocity();
            return;
        }
        
        Pose2DStamped current_pose;
        current_pose.pose.position.x = current_odom_.pose.pose.position.x;
        current_pose.pose.position.y = current_odom_.pose.pose.position.y;
        current_pose.pose.orientation = getYawFromQuaternion(current_odom_.pose.pose.orientation);
        current_pose.timestamp = current_odom_.header.stamp.nanosec + 
                                  current_odom_.header.stamp.sec * 1000000000ULL;
        
        if (current_path_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Path is empty in control loop");
            return;
        }
        
        double robot_x = current_pose.pose.position.x;
        double robot_y = current_pose.pose.position.y;
        double robot_theta = current_pose.pose.orientation;
        
        Path local_path;
        for (const auto& world_pose : current_path_.poses) {
            double dx = world_pose.position.x - robot_x;
            double dy = world_pose.position.y - robot_y;
            double cos_th = std::cos(-robot_theta);
            double sin_th = std::sin(-robot_theta);
            
            Pose2D local_pose;
            local_pose.position.x = dx * cos_th - dy * sin_th;
            local_pose.position.y = dx * sin_th + dy * cos_th;
            local_pose.orientation = normalizeAngle(world_pose.orientation - robot_theta);
            local_path.poses.push_back(local_pose);
        }
        
        try
        {
            std::vector<Pose2D> local_trajectory;
            Twist cmd_vel = controller_->computeVelocityCommands(current_pose, local_path, local_trajectory);
            
            if (!local_trajectory.empty()) {
                publishLocalTrajectory(local_trajectory, robot_x, robot_y, robot_theta);
            }
            
            if (!local_path.poses.empty()) {
                double min_dist = std::numeric_limits<double>::max();
                size_t closest_idx = 0;
                for (size_t i = 0; i < local_path.poses.size(); ++i) {
                    double d = std::hypot(local_path.poses[i].position.x, local_path.poses[i].position.y);
                    if (d < min_dist) {
                        min_dist = d;
                        closest_idx = i;
                    }
                }
                const auto& closest = local_path.poses[closest_idx];
                const auto& goal = local_path.poses.back();
                
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "Local path: size=%zu, closest_idx=%zu, closest=(%.2f,%.2f), goal=(%.2f,%.2f), "
                    "cmd_vel: v=%.3f, w=%.3f",
                    local_path.poses.size(), closest_idx,
                    closest.position.x, closest.position.y,
                    goal.position.x, goal.position.y,
                    cmd_vel.linear_x, cmd_vel.angular_z);
            }
            
            geometry_msgs::msg::Twist twist_msg;
            twist_msg.linear.x = cmd_vel.linear_x;
            twist_msg.linear.y = cmd_vel.linear_y;
            twist_msg.linear.z = cmd_vel.linear_z;
            twist_msg.angular.x = cmd_vel.angular_x;
            twist_msg.angular.y = cmd_vel.angular_y;
            twist_msg.angular.z = cmd_vel.angular_z;
            
            cmd_vel_pub_->publish(twist_msg);
            
            checkGoalReached(current_pose.pose);
        }
        catch (const NoValidControl& e)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "No valid control (collision detected): %s", e.what());
            publishZeroVelocity();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception in control loop: %s", e.what());
            publishZeroVelocity();
        }
    }
    
    void publishLocalTrajectory(const std::vector<Pose2D>& local_trajectory, 
                                 double robot_x, double robot_y, double robot_theta)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->get_clock()->now();
        
        double cos_th = std::cos(robot_theta);
        double sin_th = std::sin(robot_theta);
        
        for (const auto& local_pose : local_trajectory) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;
            
            double world_x = robot_x + (local_pose.position.x * cos_th - local_pose.position.y * sin_th);
            double world_y = robot_y + (local_pose.position.x * sin_th + local_pose.position.y * cos_th);
            
            pose_stamped.pose.position.x = world_x;
            pose_stamped.pose.position.y = world_y;
            pose_stamped.pose.position.z = 0.0;
            
            double world_yaw = robot_theta + local_pose.orientation;
            tf2::Quaternion q;
            q.setRPY(0, 0, world_yaw);
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();
            
            path_msg.poses.push_back(pose_stamped);
        }
        
        local_path_pub_->publish(path_msg);
    }

    void checkGoalReached(const Pose2D& current_pose)
    {
        if (current_path_.empty())
        {
            return;
        }
        
        const Pose2D& goal_pose = current_path_.back();
        
        double dx = current_pose.position.x - goal_pose.position.x;
        double dy = current_pose.position.y - goal_pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        double angle_diff = std::abs(normalizeAngle(current_pose.orientation - goal_pose.orientation));
        
        if (dist < goal_dist_tolerance_ && angle_diff < goal_angle_tolerance_)
        {
            goal_reached_ = true;
            publishZeroVelocity();
            RCLCPP_INFO(this->get_logger(), "Goal reached! Distance: %.3f, Angle: %.3f", dist, angle_diff);
        }
    }

    void publishZeroVelocity()
    {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(twist_msg);
    }

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
    {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

private:
    std::unique_ptr<GracefulController> controller_;
    Footprint robot_footprint_;
    std::vector<Point2D> obstacles_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    Path current_path_;
    nav_msgs::msg::Odometry current_odom_;
    
    bool path_received_;
    bool odom_received_;
    bool laser_received_;
    bool goal_reached_;
    
    double control_frequency_;
    double goal_dist_tolerance_;
    double goal_angle_tolerance_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GracefulControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
