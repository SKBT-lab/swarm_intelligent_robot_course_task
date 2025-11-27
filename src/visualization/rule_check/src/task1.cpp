#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>
#include <map>
#include <string>

class UAVMonitor {
private:
    ros::NodeHandle nh_;
    
    // 订阅器
    std::vector<ros::Subscriber> odom_subs_;
    
    // 发布器
    ros::Publisher collision_marker_pub_;
    
    // 存储无人机位置和状态
    std::map<std::string, geometry_msgs::Point> uav_positions_;
    std::map<std::string, geometry_msgs::Vector3> uav_velocities_;
    std::map<std::string, geometry_msgs::Vector3> uav_accelerations_;
    
    // 碰撞记录
    std::vector<geometry_msgs::Point> collision_points_;
    
    // 跟随状态记录
    std::map<std::string, bool> individual_following_status_;
    bool all_following_;
    ros::Time group_follow_start_time_;
    double group_follow_duration_;
    
    // 任务状态
    bool task_started_;
    bool task_finished_;
    ros::Time task_start_time_;
    ros::Time task_end_time_;
    
    // 参数
    double collision_distance_;
    double marker_min_distance_;
    double following_min_distance_;
    double following_max_distance_;
    double zero_threshold_;
    double start_threshold_;
    
public:
    UAVMonitor() : task_started_(false), task_finished_(false), all_following_(false), group_follow_duration_(0.0) {
        // 初始化参数
        collision_distance_ = 0.4;
        marker_min_distance_ = 1.0;
        following_min_distance_ = 1.0;
        following_max_distance_ = 2.0;
        zero_threshold_ = 0.05;
        start_threshold_ = 4;
        
        // 初始化发布器
        collision_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/collision_markers", 10);
        
        // 订阅所有无人机的里程计话题
        for (int i = 2; i <= 7; i++) {
            std::string uav_name = "uav" + std::to_string(i);
            std::string topic_name = "/" + uav_name + "/sim/odom";
            
            // 使用lambda函数避免boost::bind问题
            ros::Subscriber sub = nh_.subscribe<nav_msgs::Odometry>(
                topic_name, 10,
                [this, uav_name](const nav_msgs::Odometry::ConstPtr& msg) {
                    this->odomCallback(msg, uav_name);
                }
            );
            odom_subs_.push_back(sub);
            
            // 初始化个体跟随状态
            individual_following_status_[uav_name] = false;
        }
        
        // 单独订阅uav1
        ros::Subscriber uav1_sub = nh_.subscribe<nav_msgs::Odometry>(
            "/uav1/sim/odom", 10, 
            &UAVMonitor::uav1OdomCallback, this
        );
        odom_subs_.push_back(uav1_sub);
        
        ROS_INFO("UAV Monitor initialized, monitoring 7 UAVs");
    }
    
    void uav1OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 更新位置
        uav_positions_["uav1"] = msg->pose.pose.position;
        
        // 更新速度
        uav_velocities_["uav1"] = msg->twist.twist.linear;
        
        // 检查任务开始和结束
        checkTaskStatus(msg, "uav1");
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& uav_name) {
        // 更新位置
        uav_positions_[uav_name] = msg->pose.pose.position;
        
        // 更新速度
        uav_velocities_[uav_name] = msg->twist.twist.linear;
        
        // 检查个体跟随状态
        checkIndividualFollowingStatus(uav_name);
        
        // 检查群体跟随状态
        checkGroupFollowingStatus();
        
        // 检查碰撞
        checkCollisions();
    }
    
    void checkTaskStatus(const nav_msgs::Odometry::ConstPtr& msg, const std::string& uav_name) {
        if (!task_started_) {
            // 检查uav1是否开始运动（速度或加速度大于阈值）
            double speed = std::sqrt(
                msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                msg->twist.twist.linear.y * msg->twist.twist.linear.y +
                msg->twist.twist.linear.z * msg->twist.twist.linear.z
            );
            
            if (speed > start_threshold_) {
                task_started_ = true;
                task_start_time_ = ros::Time::now();
                ROS_INFO("Task started! UAV1 began moving.");
                // std::cout << "speed: " << speed << std::endl;
            }
        } 
        else if (!task_finished_) {
            // 检查任务是否结束（速度和加速度都接近0）
            double speed = std::sqrt(
                msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                msg->twist.twist.linear.y * msg->twist.twist.linear.y +
                msg->twist.twist.linear.z * msg->twist.twist.linear.z
            );
            
            double accel_magnitude = std::sqrt(
                msg->twist.twist.angular.x * msg->twist.twist.angular.x +
                msg->twist.twist.angular.y * msg->twist.twist.angular.y +
                msg->twist.twist.angular.z * msg->twist.twist.angular.z
            );
            
            if (speed < zero_threshold_ && accel_magnitude < zero_threshold_) {
                task_finished_ = true;
                task_end_time_ = ros::Time::now();
                
                // 如果当前正在群体跟随，结束计时
                if (all_following_) {
                    ros::Time end_time = ros::Time::now();
                    group_follow_duration_ += (end_time - group_follow_start_time_).toSec();
                    all_following_ = false;
                }
                
                printFinalStatistics();
            }
        }
    }
    
    void checkIndividualFollowingStatus(const std::string& uav_name) {
        if (uav_name == "uav1" || !task_started_ || task_finished_) return;
        
        // 检查该无人机与uav1的距离
        if (uav_positions_.find("uav1") == uav_positions_.end() || 
            uav_positions_.find(uav_name) == uav_positions_.end()) {
            return;
        }
        
        geometry_msgs::Point pos1 = uav_positions_["uav1"];
        geometry_msgs::Point pos2 = uav_positions_[uav_name];
        
        double distance = std::sqrt(
            std::pow(pos1.x - pos2.x, 2) +
            std::pow(pos1.y - pos2.y, 2) +
            std::pow(pos1.z - pos2.z, 2)
        );
        
        individual_following_status_[uav_name] = (distance >= following_min_distance_ && distance <= following_max_distance_);
    }
    
    void checkGroupFollowingStatus() {
        if (!task_started_ || task_finished_) return;
        
        // 检查是否所有uav2-uav7都在有效跟随距离内
        bool all_following_now = true;
        for (int i = 2; i <= 7; i++) {
            std::string uav_name = "uav" + std::to_string(i);
            if (uav_positions_.find(uav_name) == uav_positions_.end()) {
                all_following_now = false;
                break;
            }
            if (!individual_following_status_[uav_name]) {
                all_following_now = false;
                break;
            }
        }
        
        if (all_following_now && !all_following_) {
            // 开始群体跟随计时
            all_following_ = true;
            group_follow_start_time_ = ros::Time::now();
            ROS_INFO("All UAVs (2-7) are now in effective following range!");
        } 
        else if (!all_following_now && all_following_) {
            // 结束群体跟随计时
            all_following_ = false;
            ros::Time end_time = ros::Time::now();
            double duration = (end_time - group_follow_start_time_).toSec();
            group_follow_duration_ += duration;
            ROS_INFO("Group following period ended. Duration: %.2f seconds", duration);
        }
    }
    
    void checkCollisions() {
        if (!task_started_) return;
        
        std::vector<std::string> uav_names;
        for (const auto& pair : uav_positions_) {
            uav_names.push_back(pair.first);
        }
        
        // 检查所有无人机对之间的碰撞
        for (size_t i = 0; i < uav_names.size(); i++) {
            for (size_t j = i + 1; j < uav_names.size(); j++) {
                std::string uav1 = uav_names[i];
                std::string uav2 = uav_names[j];
                
                geometry_msgs::Point pos1 = uav_positions_[uav1];
                geometry_msgs::Point pos2 = uav_positions_[uav2];
                
                double distance = std::sqrt(
                    std::pow(pos1.x - pos2.x, 2) +
                    std::pow(pos1.y - pos2.y, 2) +
                    std::pow(pos1.z - pos2.z, 2)
                );
                
                if (distance < collision_distance_) {
                    // 检测到碰撞
                    geometry_msgs::Point collision_point;
                    collision_point.x = (pos1.x + pos2.x) / 2.0;
                    collision_point.y = (pos1.y + pos2.y) / 2.0;
                    collision_point.z = (pos1.z + pos2.z) / 2.0;
                    
                    // 检查是否在最小距离内已存在碰撞点
                    bool too_close = false;
                    for (const auto& existing_point : collision_points_) {
                        double point_distance = std::sqrt(
                            std::pow(collision_point.x - existing_point.x, 2) +
                            std::pow(collision_point.y - existing_point.y, 2) +
                            std::pow(collision_point.z - existing_point.z, 2)
                        );
                        
                        if (point_distance < marker_min_distance_) {
                            too_close = true;
                            break;
                        }
                    }
                    
                    if (!too_close) {
                        collision_points_.push_back(collision_point);
                        publishCollisionMarker(collision_point, uav1, uav2);
                        ROS_WARN("Collision detected between %s and %s at (%.2f, %.2f, %.2f)", 
                                uav1.c_str(), uav2.c_str(), collision_point.x, collision_point.y, collision_point.z);
                    }
                }
            }
        }
    }
    
    void publishCollisionMarker(const geometry_msgs::Point& point, const std::string& uav1, const std::string& uav2) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "collisions";
        marker.id = collision_points_.size();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position = point;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        
        // 橙色
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        
        marker.lifetime = ros::Duration(0); // 永久显示
        
        collision_marker_pub_.publish(marker);
    }
    
    void printFinalStatistics() {
        if (!task_finished_) return;
        
        double total_flight_time = (task_end_time_ - task_start_time_).toSec();
        double follow_percentage = (group_follow_duration_ / total_flight_time) * 100.0;
        
        ROS_INFO("======= TASK FINISHED =======");
        ROS_INFO("Total flight time of UAV1: %.2f seconds", total_flight_time);
        ROS_INFO("Group effective following time: %.2f seconds", group_follow_duration_);
        ROS_INFO("Effective following percentage: %.1f%%", follow_percentage);
        ROS_INFO("Collision points detected: %zu", collision_points_.size());
        
        ROS_INFO("------ Individual Following Status Summary ------");
        for (int i = 2; i <= 7; i++) {
            std::string uav_name = "uav" + std::to_string(i);
            ROS_INFO("%s: %s", uav_name.c_str(), 
                    individual_following_status_[uav_name] ? "In range" : "Out of range");
        }
        
        ROS_INFO("------ Collision Information ------");
        for (size_t i = 0; i < collision_points_.size(); i++) {
            ROS_INFO("Collision %zu: (%.2f, %.2f, %.2f)", 
                    i + 1, 
                    collision_points_[i].x, 
                    collision_points_[i].y, 
                    collision_points_[i].z);
        }
        
        // 性能评估
        ROS_INFO("------ Performance Evaluation ------");
        if (follow_percentage >= 80.0) {
            ROS_INFO("Excellent following performance!");
        } else if (follow_percentage >= 60.0) {
            ROS_INFO("Good following performance.");
        } else if (follow_percentage >= 40.0) {
            ROS_INFO("Average following performance.");
        } else {
            ROS_INFO("Poor following performance.");
        }
        
        if (collision_points_.size() == 0) {
            ROS_INFO("Perfect collision avoidance!");
        } else if (collision_points_.size() <= 2) {
            ROS_INFO("Good collision avoidance.");
        } else {
            ROS_INFO("Collision avoidance needs improvement.");
        }
    }
    
    void run() {
        ros::Rate rate(10); // 10Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_monitor");
    
    UAVMonitor monitor;
    monitor.run();
    
    return 0;
}