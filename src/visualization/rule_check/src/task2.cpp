#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <unordered_map>
#include <cmath>
#include <chrono>

class CollisionDetector {
private:
    ros::NodeHandle nh_;
    
    // 订阅器
    std::vector<ros::Subscriber> odom_subs_;
    ros::Subscriber pointcloud_sub_;
    
    // 发布器
    ros::Publisher marker_pub_;
    
    // 存储无人机位置和速度
    std::unordered_map<std::string, geometry_msgs::Point> uav_positions_;
    std::unordered_map<std::string, geometry_msgs::Vector3> uav_velocities_;
    
    // 存储碰撞标记位置，用于去重
    std::vector<geometry_msgs::Point> uav_uav_collisions_;
    std::vector<geometry_msgs::Point> uav_obstacle_collisions_;
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
    
    // 参数
    double uav_obstacle_threshold_;
    double uav_uav_threshold_;
    double min_marker_distance_;
    double max_speed_;
    double min_distance_;
    double max_distance_;
    double max_height_;
    
    // 目标点和任务状态
    geometry_msgs::Point target_point_;
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
    CollisionDetector() : obstacle_cloud_(new pcl::PointCloud<pcl::PointXYZ>()), mission_success_(false) {
        // 初始化参数
        nh_.param("uav_obstacle_threshold", uav_obstacle_threshold_, 0.2);
        nh_.param("uav_uav_threshold", uav_uav_threshold_, 0.4);
        nh_.param("min_marker_distance", min_marker_distance_, 1.0);
        nh_.param("max_speed", max_speed_, 6.0);
        nh_.param("min_distance", min_distance_, 0.5);
        nh_.param("max_distance", max_distance_, 5.0);
        nh_.param("max_height", max_height_, 8.0);
        
        // 设置目标点 (30, 30, 2.5)
        target_point_.x = 30.0;
        target_point_.y = 30.0;
        target_point_.z = 2.5;
        
        // 初始化无人机订阅器
        for (int i = 2; i <= 7; i++) {
            std::string uav_name = "uav" + std::to_string(i);
            std::string topic_name = "/" + uav_name + "/sim/odom";
            
            odom_subs_.push_back(
                nh_.subscribe<nav_msgs::Odometry>(
                    topic_name, 10,
                    [this, uav_name](const nav_msgs::Odometry::ConstPtr& msg) {
                        this->odomCallback(msg, uav_name);
                    }
                )
            );
            
            // 初始化统计信息
            stats_.uav_speed_violations[uav_name] = 0;
            stats_.uav_height_violations[uav_name] = 0;
        }
        
        // 订阅点云地图
        pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/mock_map", 10, &CollisionDetector::pointcloudCallback, this);
        
        // 发布标记
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/collision_markers", 10);
        
        // 记录开始时间
        start_time_ = std::chrono::steady_clock::now();
        
        ROS_INFO("Collision detector initialized");
        ROS_INFO("Target point: (%.1f, %.1f, %.1f)", target_point_.x, target_point_.y, target_point_.z);
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& uav_name) {
        // 存储无人机位置和速度
        uav_positions_[uav_name] = msg->pose.pose.position;
        uav_velocities_[uav_name] = msg->twist.twist.linear;
        
        // 检查所有规则
        if (!mission_success_) {
            checkAllRules(uav_name);
        }
    }
    
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // 转换点云数据
        pcl::fromROSMsg(*msg, *obstacle_cloud_);
        
        // 构建KD树用于最近邻搜索
        kdtree_.setInputCloud(obstacle_cloud_);
        
        // ROS_INFO("Received point cloud with %ld points", obstacle_cloud_->size());
    }
    
    void checkAllRules(const std::string& current_uav = "") {
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
            geometry_msgs::Point uav_pos = uav_pair.second;
            pcl::PointXYZ search_point(uav_pos.x, uav_pos.y, uav_pos.z);
            
            std::vector<int> point_idx(1);
            std::vector<float> point_sqr_distance(1);
            
            if (kdtree_.nearestKSearch(search_point, 1, point_idx, point_sqr_distance) > 0) {
                double distance = std::sqrt(point_sqr_distance[0]);
                
                if (distance < uav_obstacle_threshold_) {
                    geometry_msgs::Point collision_point;
                    collision_point.x = uav_pos.x;
                    collision_point.y = uav_pos.y;
                    collision_point.z = uav_pos.z;
                    
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
                geometry_msgs::Point pos1 = uav_positions_[uav_names[i]];
                geometry_msgs::Point pos2 = uav_positions_[uav_names[j]];
                
                double distance = calculateDistance(pos1, pos2);
                
                // 检查碰撞
                if (distance < uav_uav_threshold_) {
                    geometry_msgs::Point collision_point;
                    collision_point.x = (pos1.x + pos2.x) / 2.0;
                    collision_point.y = (pos1.y + pos2.y) / 2.0;
                    collision_point.z = (pos1.z + pos2.z) / 2.0;
                    
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
            if (uav_pair.second.z > max_height_) {
                stats_.height_violations++;
                stats_.uav_height_violations[uav_pair.first]++;
                ROS_WARN("%s exceeded height limit! Current height: %.3f m", 
                        uav_pair.first.c_str(), uav_pair.second.z);
            }
        }
        
        // 发布标记
        publishMarkers(new_uav_uav_collisions, new_uav_obstacle_collisions);
    }
    
    void checkMissionSuccess() {
        if (uav_positions_.size() < 6) return;
        
        double total_distance = 0.0;
        int uav_count = 0;
        
        for (const auto& uav_pair : uav_positions_) {
            double distance = calculateDistance(uav_pair.second, target_point_);
            total_distance += distance;
            uav_count++;
        }
        
        double average_distance = total_distance / uav_count;
        
        if (average_distance <= 2.5 && !mission_success_) {
            mission_success_ = true;
            auto end_time = std::chrono::steady_clock::now();
            mission_duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                end_time - start_time_).count() / 1000.0;
            
            ROS_INFO("==========================================");
            ROS_INFO(" MISSION SUCCESS! ");
            ROS_INFO("Average distance to target: %.3f m", average_distance);
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
        for (int i = 2; i <= 7; i++) {
            std::string uav_name = "uav" + std::to_string(i);
            ROS_INFO("  %s: Speed violations: %d, Height violations: %d", 
                    uav_name.c_str(), 
                    stats_.uav_speed_violations[uav_name],
                    stats_.uav_height_violations[uav_name]);
        }
        
        // 计算成功率
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
            double distance = calculateDistance(new_point, existing_point);
            if (distance < min_marker_distance_) {
                return true;
            }
        }
        return false;
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
    
    void publishMarkers(const std::vector<geometry_msgs::Point>& uav_uav_collisions,
                       const std::vector<geometry_msgs::Point>& uav_obstacle_collisions) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "collisions";
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(0); // 永久显示
        
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
        
        // 发布目标点标记（绿色）
        visualization_msgs::Marker target_marker;
        target_marker.header.frame_id = "world";
        target_marker.header.stamp = ros::Time::now();
        target_marker.ns = "target";
        target_marker.id = 2;
        target_marker.type = visualization_msgs::Marker::SPHERE;
        target_marker.action = visualization_msgs::Marker::ADD;
        target_marker.pose.position = target_point_;
        target_marker.pose.orientation.w = 1.0;
        target_marker.scale.x = 0.5;
        target_marker.scale.y = 0.5;
        target_marker.scale.z = 0.5;
        target_marker.color.r = 0.0;
        target_marker.color.g = 1.0;
        target_marker.color.b = 0.0;
        target_marker.color.a = 1.0;
        target_marker.lifetime = ros::Duration(0);
        marker_pub_.publish(target_marker);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_detector");
    
    CollisionDetector detector;
    
    ros::spin();
    
    return 0;
}