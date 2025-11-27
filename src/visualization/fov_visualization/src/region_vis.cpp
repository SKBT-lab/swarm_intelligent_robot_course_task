#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>
#include <unordered_map>
#include <chrono>

// 网格状态枚举
enum GridStatus {
    UNEXPLORED = 0,  // 未探索
    EXPLORED = 1,    // 已探索  
    OBSTACLE = 2     // 障碍物
};

class ExplorationVisualizer {
private:
    ros::NodeHandle nh_;
    
    // 订阅者
    std::vector<ros::Subscriber> odom_subs_;
    ros::Subscriber map_sub_;
    
    // 发布者
    ros::Publisher grid_info_pub_;
    ros::Publisher marker_pub_;  // 新增：碰撞标记发布者
    
    // 探索区域参数
    Eigen::Vector2d exploration_center_;
    double exploration_width_;
    double exploration_height_;
    
    // 网格参数
    double grid_resolution_;
    int grid_width_;
    int grid_height_;
    std::vector<GridStatus> grid_status_;  // 网格状态数组
    
    // 无人机信息
    std::map<std::string, Eigen::Vector3d> uav_positions_;
    std::map<std::string, geometry_msgs::Vector3> uav_velocities_;
    std::vector<std::string> uav_names_;
    
    // FOV参数
    double fov_scale_;
    
    // 地图处理标志
    bool map_processed_;
    
    // 碰撞检测相关
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
    std::vector<geometry_msgs::Point> uav_uav_collisions_;
    std::vector<geometry_msgs::Point> uav_obstacle_collisions_;
    
    // 规则参数
    double uav_obstacle_threshold_;
    double uav_uav_threshold_;
    double min_marker_distance_;
    double max_speed_;
    double min_distance_;
    double max_distance_;
    double max_height_;
    
    // 任务状态
    bool mission_success_;
    std::chrono::steady_clock::time_point start_time_;
    double mission_duration_;
    
    // 统计信息
    struct MissionStats {
        int uav_uav_collisions = 0;
        int uav_obstacle_collisions = 0;
        int speed_violations = 0;
        int distance_near_violations = 0;
        int distance_far_violations = 0;
        int height_violations = 0;
        std::unordered_map<std::string, int> uav_speed_violations;
        std::unordered_map<std::string, int> uav_height_violations;
    } stats_;
    
public:
    ExplorationVisualizer() : 
        exploration_center_(0.0, 0.0),
        exploration_width_(50.0),
        exploration_height_(50.0),
        fov_scale_(0.5),
        grid_resolution_(0.1),
        map_processed_(false),
        obstacle_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
        mission_success_(false) {
        
        // 从参数服务器读取参数
        nh_.param<double>("grid_resolution", grid_resolution_, 0.1);
        nh_.param<double>("exploration_center_x", exploration_center_.x(), 0.0);
        nh_.param<double>("exploration_center_y", exploration_center_.y(), 0.0);
        nh_.param<double>("exploration_width", exploration_width_, 50.0);
        nh_.param<double>("exploration_height", exploration_height_, 50.0);
        nh_.param<double>("fov_scale", fov_scale_, 0.5);
        
        // 规则参数
        nh_.param("uav_obstacle_threshold", uav_obstacle_threshold_, 0.2);
        nh_.param("uav_uav_threshold", uav_uav_threshold_, 0.4);
        nh_.param("min_marker_distance", min_marker_distance_, 1.0);
        nh_.param("max_speed", max_speed_, 6.0);
        nh_.param("min_distance", min_distance_, 0.5);
        nh_.param("max_distance", max_distance_, 5.0);
        nh_.param("max_height", max_height_, 8.0);
        
        // 初始化网格
        updateGridSize();
        
        // 初始化无人机名称
        uav_names_ = {"uav2", "uav3", "uav4", "uav5", "uav6", "uav7"};
        
        // 初始化发布者
        grid_info_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("exploration_grid", 10, true);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/collision_markers", 10);
        
        // 订阅地图话题
        map_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/mock_map", 1, 
                         &ExplorationVisualizer::mapCallback, this);
        
        // 为每个无人机创建订阅者
        for (const auto& uav_name : uav_names_) {
            std::string odom_topic = "/" + uav_name + "/sim/odom";
            ros::Subscriber sub = nh_.subscribe<nav_msgs::Odometry>(
                odom_topic, 10,
                boost::bind(&ExplorationVisualizer::odomCallback, this, _1, uav_name)
            );
            odom_subs_.push_back(sub);
            
            // 初始化统计信息
            stats_.uav_speed_violations[uav_name] = 0;
            stats_.uav_height_violations[uav_name] = 0;
        }
        
        // 记录开始时间
        start_time_ = std::chrono::steady_clock::now();
        
        ROS_INFO("Exploration Visualizer with Rule Checking initialized");
        ROS_INFO("Waiting for map data from /mock_map...");
    }
    
    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (map_processed_) {
            return;  // 地图已经处理过，避免重复处理
        }
        
        ROS_INFO("Received map point cloud with %d points", msg->width * msg->height);
        
        // 转换点云数据
        pcl::fromROSMsg(*msg, *obstacle_cloud_);
        
        // 构建KD树用于碰撞检测
        kdtree_.setInputCloud(obstacle_cloud_);
        
        // 处理障碍物
        processObstacles(obstacle_cloud_);
        
        // 发布初始网格信息
        publishGridInfo();
        
        map_processed_ = true;
        ROS_INFO("Map processing completed. Obstacles marked in exploration region.");
    }
    
    void processObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        int obstacle_count = 0;
        
        for (const auto& point : cloud->points) {
            // 检查点是否在探索区域内
            if (isPointInExplorationRegion(point.x, point.y)) {
                // 标记对应的网格为障碍物
                if (markGridAsObstacle(point.x, point.y)) {
                    obstacle_count++;
                }
            }
        }
        
        ROS_INFO("Marked %d obstacle cells in exploration region", obstacle_count);
        
        // 计算可探索网格统计
        int total_cells = grid_width_ * grid_height_;
        int obstacle_cells = countGridsByStatus(OBSTACLE);
        int explorable_cells = total_cells - obstacle_cells;
        
        ROS_INFO("Grid statistics: Total=%d, Obstacles=%d, Explorable=%d", 
                 total_cells, obstacle_cells, explorable_cells);
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& uav_name) {
        if (!map_processed_) {
            ROS_WARN_THROTTLE(5.0, "Map not processed yet, skipping exploration update");
            return;
        }
        
        // 更新无人机位置
        Eigen::Vector3d position(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        
        uav_positions_[uav_name] = position;
        
        // 更新无人机速度
        uav_velocities_[uav_name] = msg->twist.twist.linear;
        
        // 更新探索状态
        updateExplorationStatus();
        
        // 检查所有规则
        if (!mission_success_) {
            checkAllRules();
        }
        
        // 发布探索状态可视化
        publishExplorationStatus();
        
        // 发布更新的网格信息
        publishGridInfo();
        
        ROS_DEBUG_THROTTLE(2.0, "Updated position for %s: (%.2f, %.2f, %.2f)", 
                          uav_name.c_str(), position.x(), position.y(), position.z());
    }
    
private:
    void updateGridSize() {
        grid_width_ = static_cast<int>(exploration_width_ / grid_resolution_);
        grid_height_ = static_cast<int>(exploration_height_ / grid_resolution_);
        
        // 限制最大网格数量
        int max_cells = 10000000000;
        if (grid_width_ * grid_height_ > max_cells) {
            ROS_WARN("Grid too dense: %d cells. Limiting resolution.", grid_width_ * grid_height_);
            grid_resolution_ = std::max(exploration_width_, exploration_height_) / 100.0;
            grid_width_ = static_cast<int>(exploration_width_ / grid_resolution_);
            grid_height_ = static_cast<int>(exploration_height_ / grid_resolution_);
        }
        
        // 初始化所有网格为未探索状态
        grid_status_.resize(grid_width_ * grid_height_, UNEXPLORED);
        
        ROS_INFO("Grid initialized: resolution=%.3f, size=%dx%d=%d cells", 
                 grid_resolution_, grid_width_, grid_height_, grid_width_ * grid_height_);
    }
    
    bool isPointInExplorationRegion(double x, double y) {
        return (x >= exploration_center_.x() - exploration_width_/2) &&
               (x <= exploration_center_.x() + exploration_width_/2) &&
               (y >= exploration_center_.y() - exploration_height_/2) &&
               (y <= exploration_center_.y() + exploration_height_/2);
    }
    
    bool markGridAsObstacle(double world_x, double world_y) {
        int grid_x, grid_y;
        if (worldToGrid(world_x, world_y, grid_x, grid_y)) {
            int index = grid_y * grid_width_ + grid_x;
            grid_status_[index] = OBSTACLE;
            return true;
        }
        return false;
    }
    
    void markGridAsExplored(double world_x, double world_y) {
        int grid_x, grid_y;
        if (worldToGrid(world_x, world_y, grid_x, grid_y)) {
            int index = grid_y * grid_width_ + grid_x;
            // 只有非障碍物网格才能被标记为已探索
            if (grid_status_[index] != OBSTACLE) {
                grid_status_[index] = EXPLORED;
            }
        }
    }
    
    bool worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) {
        grid_x = static_cast<int>((world_x - (exploration_center_.x() - exploration_width_/2)) / grid_resolution_);
        grid_y = static_cast<int>((world_y - (exploration_center_.y() - exploration_height_/2)) / grid_resolution_);
        
        return (grid_x >= 0 && grid_x < grid_width_ && grid_y >= 0 && grid_y < grid_height_);
    }
    
    Eigen::Vector2d gridToWorld(int grid_x, int grid_y) {
        double world_x = exploration_center_.x() - exploration_width_/2 + (grid_x + 0.5) * grid_resolution_;
        double world_y = exploration_center_.y() - exploration_height_/2 + (grid_y + 0.5) * grid_resolution_;
        return Eigen::Vector2d(world_x, world_y);
    }
    
    int countGridsByStatus(GridStatus status) {
        int count = 0;
        for (const auto& grid_status : grid_status_) {
            if (grid_status == status) count++;
        }
        return count;
    }
    
    void updateExplorationStatus() {
        for (const auto& uav_pair : uav_positions_) {
            const Eigen::Vector3d& position = uav_pair.second;
            double fov_radius = fov_scale_ * position.z();
            
            // 计算FOV圆在探索区域内的边界
            double min_x = std::max(exploration_center_.x() - exploration_width_/2, 
                                   position.x() - fov_radius);
            double max_x = std::min(exploration_center_.x() + exploration_width_/2, 
                                   position.x() + fov_radius);
            double min_y = std::max(exploration_center_.y() - exploration_height_/2, 
                                   position.y() - fov_radius);
            double max_y = std::min(exploration_center_.y() + exploration_height_/2, 
                                   position.y() + fov_radius);
            
            // 遍历FOV圆内的所有网格
            for (double x = min_x; x <= max_x; x += grid_resolution_) {
                for (double y = min_y; y <= max_y; y += grid_resolution_) {
                    double dx = x - position.x();
                    double dy = y - position.y();
                    if (dx*dx + dy*dy <= fov_radius * fov_radius) {
                        markGridAsExplored(x, y);
                    }
                }
            }
        }
    }
    
    void checkAllRules() {
        if (uav_positions_.size() < 6 || obstacle_cloud_->empty()) {
            return; // 等待所有数据就绪
        }
        
        // 检查任务成功条件
        checkMissionSuccess();
        if (mission_success_) {
            return;
        }
        
        std::vector<geometry_msgs::Point> new_uav_uav_collisions;
        std::vector<geometry_msgs::Point> new_uav_obstacle_collisions;
        
        // 1. 检查无人机与障碍物的碰撞
        for (const auto& uav_pair : uav_positions_) {
            Eigen::Vector3d uav_pos = uav_pair.second;
            pcl::PointXYZ search_point(uav_pos.x(), uav_pos.y(), uav_pos.z());
            
            std::vector<int> point_idx(1);
            std::vector<float> point_sqr_distance(1);
            
            if (kdtree_.nearestKSearch(search_point, 1, point_idx, point_sqr_distance) > 0) {
                double distance = std::sqrt(point_sqr_distance[0]);
                
                if (distance < uav_obstacle_threshold_) {
                    geometry_msgs::Point collision_point;
                    collision_point.x = uav_pos.x();
                    collision_point.y = uav_pos.y();
                    collision_point.z = uav_pos.z();
                    
                    // 检查是否在最小距离内已存在同类碰撞标记
                    if (!isNearExistingCollision(collision_point, uav_obstacle_collisions_)) {
                        new_uav_obstacle_collisions.push_back(collision_point);
                        uav_obstacle_collisions_.push_back(collision_point);
                        stats_.uav_obstacle_collisions++;
                        ROS_WARN("UAV-%s collided with obstacle! Distance: %.3f", 
                                uav_pair.first.c_str(), distance);
                    }
                }
            }
        }
        
        // 2. 检查无人机之间的碰撞和距离规则
        std::vector<std::string> uav_names;
        for (const auto& pair : uav_positions_) {
            uav_names.push_back(pair.first);
        }
        
        for (size_t i = 0; i < uav_names.size(); i++) {
            for (size_t j = i + 1; j < uav_names.size(); j++) {
                Eigen::Vector3d pos1 = uav_positions_[uav_names[i]];
                Eigen::Vector3d pos2 = uav_positions_[uav_names[j]];
                
                double distance = calculateDistance(pos1, pos2);
                
                // 检查碰撞
                if (distance < uav_uav_threshold_) {
                    geometry_msgs::Point collision_point;
                    collision_point.x = (pos1.x() + pos2.x()) / 2.0;
                    collision_point.y = (pos1.y() + pos2.y()) / 2.0;
                    collision_point.z = (pos1.z() + pos2.z()) / 2.0;
                    
                    // 检查是否在最小距离内已存在同类碰撞标记
                    if (!isNearExistingCollision(collision_point, uav_uav_collisions_)) {
                        new_uav_uav_collisions.push_back(collision_point);
                        uav_uav_collisions_.push_back(collision_point);
                        stats_.uav_uav_collisions++;
                        ROS_WARN("Collision between %s and %s! Distance: %.3f", 
                                uav_names[i].c_str(), uav_names[j].c_str(), distance);
                    }
                }
                
                // 检查距离规则
                if (distance < min_distance_) {
                    stats_.distance_near_violations++;
                    ROS_WARN("%s and %s are too close! Distance: %.3f", 
                            uav_names[i].c_str(), uav_names[j].c_str(), distance);
                } else if (distance > max_distance_) {
                    stats_.distance_far_violations++;
                    ROS_WARN("%s and %s are too far! Distance: %.3f", 
                            uav_names[i].c_str(), uav_names[j].c_str(), distance);
                }
            }
        }
        
        // 3. 检查速度规则
        for (const auto& uav_pair : uav_velocities_) {
            double speed = calculateSpeed(uav_pair.second);
            if (speed > max_speed_) {
                stats_.speed_violations++;
                stats_.uav_speed_violations[uav_pair.first]++;
                ROS_WARN("%s overspeed! Current speed: %.3f m/s", uav_pair.first.c_str(), speed);
            }
        }
        
        // 4. 检查高度规则
        for (const auto& uav_pair : uav_positions_) {
            if (uav_pair.second.z() > max_height_) {
                stats_.height_violations++;
                stats_.uav_height_violations[uav_pair.first]++;
                ROS_WARN("%s exceeded height limit! Current height: %.3f m", 
                        uav_pair.first.c_str(), uav_pair.second.z());
            }
        }
        
        // 发布碰撞标记
        publishCollisionMarkers(new_uav_uav_collisions, new_uav_obstacle_collisions);
    }
    
    void checkMissionSuccess() {
        // 计算探索进度（排除障碍物）
        int explored_count = countGridsByStatus(EXPLORED);
        int obstacle_count = countGridsByStatus(OBSTACLE);
        int explorable_count = grid_status_.size() - obstacle_count;
        
        double progress = (explorable_count > 0) ? 
                         static_cast<double>(explored_count) / explorable_count : 0.0;
        
        if (progress >= 1.0 && !mission_success_) {
            mission_success_ = true;
            auto end_time = std::chrono::steady_clock::now();
            mission_duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                end_time - start_time_).count() / 1000.0;
            
            ROS_INFO("==========================================");
            ROS_INFO("  MISSION SUCCESS! ");
            ROS_INFO("Exploration progress: %.1f%% (%d/%d cells)", 
                     progress * 100.0, explored_count, explorable_count);
            ROS_INFO("Mission completed in: %.3f seconds", mission_duration_);
            ROS_INFO("==========================================");
            
            // 打印任务统计信息
            printMissionSummary();
        }
    }
    
    void printMissionSummary() {
        ROS_INFO("  MISSION SUMMARY:");
        ROS_INFO("Total mission time: %.3f seconds", mission_duration_);
        ROS_INFO("Collision statistics:");
        ROS_INFO("  - UAV-UAV collisions: %d", stats_.uav_uav_collisions);
        ROS_INFO("  - UAV-Obstacle collisions: %d", stats_.uav_obstacle_collisions);
        ROS_INFO("Speed violations: %d", stats_.speed_violations);
        ROS_INFO("Distance violations:");
        ROS_INFO("  - Too close: %d", stats_.distance_near_violations);
        ROS_INFO("  - Too far: %d", stats_.distance_far_violations);
        ROS_INFO("Height violations: %d", stats_.height_violations);
        
        ROS_INFO("Per-UAV statistics:");
        for (const auto& uav_name : uav_names_) {
            ROS_INFO("  %s: Speed violations: %d, Height violations: %d", 
                    uav_name.c_str(), 
                    stats_.uav_speed_violations[uav_name],
                    stats_.uav_height_violations[uav_name]);
        }
        
        // 计算总违规次数
        int total_violations = stats_.uav_uav_collisions + stats_.uav_obstacle_collisions +
                              stats_.speed_violations + stats_.distance_near_violations +
                              stats_.distance_far_violations + stats_.height_violations;
        
        if (total_violations == 0) {
            ROS_INFO("  Perfect mission! No rule violations detected.");
        } else {
            ROS_INFO("  Total rule violations: %d", total_violations);
        }
    }
    
    bool isNearExistingCollision(const geometry_msgs::Point& new_point, 
                                const std::vector<geometry_msgs::Point>& existing_collisions) {
        for (const auto& existing_point : existing_collisions) {
            double distance = calculateDistance(existing_point, new_point);
            if (distance < min_marker_distance_) {
                return true;
            }
        }
        return false;
    }
    
    double calculateDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        return (p1 - p2).norm();
    }
    
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    double calculateSpeed(const geometry_msgs::Vector3& velocity) {
        return std::sqrt(velocity.x*velocity.x + velocity.y*velocity.y + velocity.z*velocity.z);
    }
    
    void publishCollisionMarkers(const std::vector<geometry_msgs::Point>& uav_uav_collisions,
                                const std::vector<geometry_msgs::Point>& uav_obstacle_collisions) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "collisions";
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(0);
        
        // 发布无人机-无人机碰撞标记（橙色）
        if (!uav_uav_collisions.empty()) {
            marker.id = 0;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.r = 1.0;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.points = uav_uav_collisions;
            marker_pub_.publish(marker);
        }
        
        // 发布无人机-障碍物碰撞标记（红色）
        if (!uav_obstacle_collisions.empty()) {
            marker.id = 1;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.points = uav_obstacle_collisions;
            marker_pub_.publish(marker);
        }
    }
    
    void publishGridInfo() {
        nav_msgs::OccupancyGrid grid_msg;
        
        // 设置header
        grid_msg.header.stamp = ros::Time::now();
        grid_msg.header.frame_id = "world";
        
        // 设置地图信息
        grid_msg.info.resolution = grid_resolution_;
        grid_msg.info.width = grid_width_;
        grid_msg.info.height = grid_height_;
        grid_msg.info.origin.position.x = exploration_center_.x() - exploration_width_/2;
        grid_msg.info.origin.position.y = exploration_center_.y() - exploration_height_/2;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;
        
        grid_msg.data.resize(grid_width_ * grid_height_);
        
        for (int y = 0; y < grid_height_; ++y) {
            for (int x = 0; x < grid_width_; ++x) {
                int our_index = y * grid_width_ + x;
                int occupancy_index = y * grid_width_ + x;
                
                switch (grid_status_[our_index]) {
                    case OBSTACLE:
                        grid_msg.data[occupancy_index] = 100;
                        break;
                    case EXPLORED:
                        grid_msg.data[occupancy_index] = 0;
                        break;
                    case UNEXPLORED:
                    default:
                        grid_msg.data[occupancy_index] = -1;
                        break;
                }
            }
        }
        
        grid_info_pub_.publish(grid_msg);
    }
    
    void publishExplorationStatus() {
        // 计算探索进度（排除障碍物）
        int explored_count = countGridsByStatus(EXPLORED);
        int obstacle_count = countGridsByStatus(OBSTACLE);
        int explorable_count = grid_status_.size() - obstacle_count;
        
        double progress = (explorable_count > 0) ? 
                         static_cast<double>(explored_count) / explorable_count : 0.0;
        
        if (!mission_success_) {
            ROS_INFO_THROTTLE(5.0, "Exploration progress: %.1f%% (%d/%d cells, %d obstacles)", 
                             progress * 100.0, explored_count, explorable_count, obstacle_count);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_visualizer_with_rule_checking");
    ExplorationVisualizer visualizer;
    ros::spin();
    return 0;
}