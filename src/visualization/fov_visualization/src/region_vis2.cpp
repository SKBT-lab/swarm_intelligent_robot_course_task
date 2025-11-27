#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>
#include <random>
#include <unordered_map>
#include <chrono>

enum GridStatus {
    UNEXPLORED = 0,
    EXPLORED = 1
};

// 无人机状态枚举
enum UAVStatus {
    INITIALIZING = 0,  // 初始化中，正在飞向初始位置
    READY = 1,         // 初始化完成，准备起飞
    TAKEN_OFF = 2      // 已起飞
};

struct ExplorationRegion {
    Eigen::Vector2d center;
    double width;
    double height;
    double grid_resolution;
    int grid_width;
    int grid_height;
    std::vector<GridStatus> grid_status;
    std::string grid_topic;
};

class ExplorationVisualizer {
private:
    ros::NodeHandle nh_;
    
    std::vector<ros::Subscriber> odom_subs_;
    std::vector<ros::Publisher> grid_info_pubs_;
    ros::Publisher marker_pub_;  // 新增：标记发布者
    
    std::vector<ExplorationRegion> regions_;
    std::map<std::string, Eigen::Vector3d> uav_positions_;
    std::map<std::string, geometry_msgs::Vector3> uav_velocities_;
    std::map<std::string, UAVStatus> uav_status_;  // 修改为状态枚举
    std::map<std::string, Eigen::Vector3d> uav_initial_positions_;  // 记录无人机初始位置
    std::vector<std::string> uav_names_;
    
    double fov_scale_;

    // 随机数生成器
    std::random_device rd_;
    std::mt19937 gen_;
    
    // 添加总探索统计变量
    int total_explored_count_;
    int total_grid_cells_;

    double total_progress;
    
    // 规则检测相关
    std::vector<geometry_msgs::Point> distance_violation_markers_;  // 距离违规标记
    double min_marker_distance_;
    double max_speed_;
    double min_distance_;  // 已起飞无人机最小间距
    double initialization_threshold_;  // 初始化完成阈值
    double takeoff_threshold_;  // 起飞判定阈值
    
    // 任务状态
    bool mission_success_;
    bool mission_started_;  // 任务是否已开始（计时开始）
    std::chrono::steady_clock::time_point start_time_;
    double mission_duration_;
    
    // 统计信息
    struct MissionStats {
        int speed_violations = 0;
        int distance_violations = 0;
        std::unordered_map<std::string, int> uav_speed_violations;
        std::unordered_map<std::string, int> uav_distance_violations;
    } stats_;
    
public:
    ExplorationVisualizer() : 
        fov_scale_(0.5), 
        gen_(getTrueRandomSeed()), 
        total_explored_count_(0), 
        total_grid_cells_(0),
        mission_success_(false),
        mission_started_(false) {
        
        ROS_INFO("Random seed initialized for true random behavior");
        
        // 规则参数
        nh_.param("min_marker_distance", min_marker_distance_, 1.0);
        nh_.param("max_speed", max_speed_, 6.0);
        nh_.param("min_distance", min_distance_, 3.0);  // 已起飞无人机最小间距3m
        nh_.param("initialization_threshold", initialization_threshold_, 0.05);  // 初始化完成阈值0.05m
        nh_.param("takeoff_threshold", takeoff_threshold_, 0.1);  // 起飞判定阈值0.1m
        
        // 初始化无人机初始位置（根据launch文件配置）
        initializeUAVInitialPositions();
        
        // 初始化三个矩形区域（使用真随机参数）
        initializeRegions();
        
        // 初始化总网格数量
        updateTotalGridStats();
        
        // 初始化9个无人机名称和状态
        for (int i = 2; i <= 10; ++i) {
            std::string uav_name = "uav" + std::to_string(i);
            uav_names_.push_back(uav_name);
            uav_status_[uav_name] = INITIALIZING;  // 初始状态为初始化中
            
            // 初始化统计信息
            stats_.uav_speed_violations[uav_name] = 0;
            stats_.uav_distance_violations[uav_name] = 0;
            
            ROS_INFO("UAV %s target initial position: (%.1f, %.1f, %.1f)", 
                    uav_name.c_str(), 
                    uav_initial_positions_[uav_name].x(),
                    uav_initial_positions_[uav_name].y(),
                    uav_initial_positions_[uav_name].z());
        }
        
        // 为每个区域创建发布者
        for (const auto& region : regions_) {
            ros::Publisher pub = nh_.advertise<nav_msgs::OccupancyGrid>(region.grid_topic, 10, true);
            grid_info_pubs_.push_back(pub);
        }
        
        // 创建标记发布者
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/collision_markers", 10);
        
        // 为每个无人机创建订阅者
        for (const auto& uav_name : uav_names_) {
            std::string odom_topic = "/" + uav_name + "/sim/odom";
            ros::Subscriber sub = nh_.subscribe<nav_msgs::Odometry>(
                odom_topic, 10,
                boost::bind(&ExplorationVisualizer::odomCallback, this, _1, uav_name)
            );
            odom_subs_.push_back(sub);
        }
        
        // 发布所有区域的初始网格信息
        for (size_t i = 0; i < regions_.size(); ++i) {
            publishGridInfo(regions_[i], i);
        }
        
        ROS_INFO("Multi-Region Exploration Visualizer with Rule Checking initialized");
        ROS_INFO("Monitoring %zu regions with %zu UAVs", regions_.size(), uav_names_.size());
        ROS_INFO("Total grid cells: %d", total_grid_cells_);
        ROS_INFO("Rule parameters: max_speed=%.1f m/s, min_distance=%.1f m (for taken-off UAVs only)", max_speed_, min_distance_);
        ROS_INFO("Initialization threshold: %.3f m, Takeoff threshold: %.3f m", initialization_threshold_, takeoff_threshold_);
    }
    
private:
    void initializeUAVInitialPositions() {
        // 根据launch文件配置设置各无人机的初始位置
        uav_initial_positions_["uav2"] = Eigen::Vector3d(-2.0, -1.0, 1.0);
        uav_initial_positions_["uav3"] = Eigen::Vector3d(-2.0, 0.0, 1.0);
        uav_initial_positions_["uav4"] = Eigen::Vector3d(-2.0, 1.0, 1.0);
        uav_initial_positions_["uav5"] = Eigen::Vector3d(-1.0, -1.0, 1.0);
        uav_initial_positions_["uav6"] = Eigen::Vector3d(-1.0, 0.0, 1.0);
        uav_initial_positions_["uav7"] = Eigen::Vector3d(-1.0, 1.0, 1.0);
        uav_initial_positions_["uav8"] = Eigen::Vector3d(0.0, -1.0, 1.0);
        uav_initial_positions_["uav9"] = Eigen::Vector3d(0.0, 0.0, 1.0);
        uav_initial_positions_["uav10"] = Eigen::Vector3d(0.0, 1.0, 1.0);
    }
    
    unsigned int getTrueRandomSeed() {
        unsigned int seed = rd_();
        ROS_INFO("Generated true random seed: %u", seed);
        return seed;
    }
    
    void initializeRegions() {
        // 区域参数范围定义
        struct RegionParamRange {
            double center_x_min, center_x_max;
            double center_y_min, center_y_max;
            double width_min, width_max;
            double height_min, height_max;
        };
        
        // 定义三个区域的不同参数范围
        std::vector<RegionParamRange> region_ranges = {
            // 区域1：较大的区域，在正x轴方向
            {0.0, 150.0, -10.0, 100.0, 30.0, 80.0, 30.0, 80.0},
            // 区域2：中等区域，在负x轴方向  
            {-150.0, -30, -10.0, 100.0, 20.0, 50.0, 20.0, 50.0},
            // 区域3：较小的区域，在负y轴方向
            {-150.0, 150.0, -200.0, -50, 10.0, 60.0, 10.0, 60.0}
        };
        
        for (size_t i = 0; i < region_ranges.size(); ++i) {
            ExplorationRegion region;
            const auto& range = region_ranges[i];
            
            // 生成随机中心点
            std::uniform_real_distribution<double> center_x_dis(range.center_x_min, range.center_x_max);
            std::uniform_real_distribution<double> center_y_dis(range.center_y_min, range.center_y_max);
            region.center = Eigen::Vector2d(center_x_dis(gen_), center_y_dis(gen_));
            
            // 生成随机宽度和高度
            std::uniform_real_distribution<double> width_dis(range.width_min, range.width_max);
            std::uniform_real_distribution<double> height_dis(range.height_min, range.height_max);
            region.width = width_dis(gen_);
            region.height = height_dis(gen_);
            
            // 固定网格分辨率
            region.grid_resolution = 0.1;
            region.grid_topic = "region" + std::to_string(i+1) + "_exploration_grid";
            
            updateRegionGridSize(region);
            regions_.push_back(region);
        }
        
        // 从参数服务器读取参数（可选）
        loadParametersFromServer();
        
        // 输出区域信息
        for (size_t i = 0; i < regions_.size(); ++i) {
            const auto& region = regions_[i];
            ROS_INFO("Region %zu: center(%.1f, %.1f), size(%.1f x %.1f), grid(%dx%d), topic: %s",
                    i+1, region.center.x(), region.center.y(), region.width, region.height,
                    region.grid_width, region.grid_height, region.grid_topic.c_str());
        }
    }
    
    void loadParametersFromServer() {
        // 从参数服务器加载区域参数
        for (size_t i = 0; i < regions_.size(); ++i) {
            std::string prefix = "region" + std::to_string(i+1) + "_";
            
            nh_.param<double>(prefix + "center_x", regions_[i].center.x(), regions_[i].center.x());
            nh_.param<double>(prefix + "center_y", regions_[i].center.y(), regions_[i].center.y());
            nh_.param<double>(prefix + "width", regions_[i].width, regions_[i].width);
            nh_.param<double>(prefix + "height", regions_[i].height, regions_[i].height);
            nh_.param<double>(prefix + "grid_resolution", regions_[i].grid_resolution, regions_[i].grid_resolution);
            
            // 更新网格尺寸
            updateRegionGridSize(regions_[i]);
        }
        
        nh_.param<double>("fov_scale", fov_scale_, 0.5);
        
        // 更新总统计信息
        updateTotalGridStats();
    }
    
    void updateRegionGridSize(ExplorationRegion& region) {
        region.grid_width = static_cast<int>(region.width / region.grid_resolution);
        region.grid_height = static_cast<int>(region.height / region.grid_resolution);
        
        // 限制最大网格数量
        int max_cells = 1000000;
        if (region.grid_width * region.grid_height > max_cells) {
            ROS_WARN("Region grid too dense: %d cells. Limiting resolution.", 
                     region.grid_width * region.grid_height);
            region.grid_resolution = std::max(region.width, region.height) / 100.0;
            region.grid_width = static_cast<int>(region.width / region.grid_resolution);
            region.grid_height = static_cast<int>(region.height / region.grid_resolution);
        }
        
        // 初始化网格状态（全部为未探索）
        region.grid_status.resize(region.grid_width * region.grid_height, UNEXPLORED);
    }
    
    // 新增：更新总网格统计信息
    void updateTotalGridStats() {
        total_grid_cells_ = 0;
        total_explored_count_ = 0;
        
        for (const auto& region : regions_) {
            total_grid_cells_ += region.grid_status.size();
            for (const auto& status : region.grid_status) {
                if (status == EXPLORED) {
                    total_explored_count_++;
                }
            }
        }
    }
    
    // 新增：计算总探索率
    double getTotalExplorationProgress() const {
        if (total_grid_cells_ == 0) return 0.0;
        return static_cast<double>(total_explored_count_) / total_grid_cells_;
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& uav_name) {
        Eigen::Vector3d position(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        
        uav_positions_[uav_name] = position;
        uav_velocities_[uav_name] = msg->twist.twist.linear;
        
        // 更新无人机状态
        updateUAVStatus(uav_name, position);
        
        // 为每个区域更新探索状态
        bool exploration_updated = false;
        for (size_t i = 0; i < regions_.size(); ++i) {
            if (updateExplorationStatusForRegion(regions_[i], position)) {
                exploration_updated = true;
                publishGridInfo(regions_[i], i);
            }
        }
        
        // 如果探索状态有更新，更新总统计并检查任务成功
        if (exploration_updated) {
            updateTotalGridStats();
            total_progress = getTotalExplorationProgress();
            checkMissionSuccess();
        }

        // 检查所有规则（仅在任务开始后且任务成功前）
        if (mission_started_ && !mission_success_) {
            checkAllRules();
        }
        
        // 输出状态信息（限频）
        static ros::Time last_status_time = ros::Time::now();
        if ((ros::Time::now() - last_status_time).toSec() > 5.0) {
            printUAVStatus();
            last_status_time = ros::Time::now();
        }
        
        ROS_INFO_THROTTLE(5.0, "Total Exploration Progress: %.1f%% (%d/%d)", 
                         total_progress * 100.0, total_explored_count_, total_grid_cells_);
        
        ROS_DEBUG_THROTTLE(2.0, "Updated position for %s: (%.2f, %.2f, %.2f)", 
                          uav_name.c_str(), position.x(), position.y(), position.z());
    }
    
    void updateUAVStatus(const std::string& uav_name, const Eigen::Vector3d& position) {
        double distance_from_target = calculateDistance(position, uav_initial_positions_[uav_name]);
        
        switch (uav_status_[uav_name]) {
            case INITIALIZING:
                // 检查是否到达初始位置（距离小于0.05m）
                if (distance_from_target < initialization_threshold_) {
                    uav_status_[uav_name] = READY;
                    ROS_INFO("%s initialization completed! Ready for takeoff. Distance: %.3f m", 
                            uav_name.c_str(), distance_from_target);
                } else {
                    ROS_DEBUG_THROTTLE(2.0, "%s initializing... Distance to target: %.3f m", 
                                      uav_name.c_str(), distance_from_target);
                }
                break;
                
            case READY:
                // 检查是否起飞（距离初始位置大于0.1m）
                if (distance_from_target > takeoff_threshold_) {
                    uav_status_[uav_name] = TAKEN_OFF;
                    ROS_INFO("%s has taken off! Distance from initial position: %.3f m", 
                            uav_name.c_str(), distance_from_target);
                    
                    // 检查是否需要开始任务计时
                    checkMissionStart();
                }
                break;
                
            case TAKEN_OFF:
                // 已起飞状态，无需额外处理
                break;
        }
    }
    
    void checkMissionStart() {
        // 如果任务尚未开始，检查是否有无人机已起飞
        if (!mission_started_) {
            int taken_off_count = 0;
            for (const auto& uav_name : uav_names_) {
                if (uav_status_[uav_name] == TAKEN_OFF) {
                    taken_off_count++;
                }
            }
            
            // 如果有至少一架无人机起飞，开始任务计时
            if (taken_off_count > 0) {
                mission_started_ = true;
                start_time_ = std::chrono::steady_clock::now();
                ROS_INFO("==========================================");
                ROS_INFO("MISSION STARTED! Timer begins now.");
                ROS_INFO("First UAV has taken off. Starting mission timer.");
                ROS_INFO("==========================================");
            }
        }
    }
    
    void printUAVStatus() {
        int initializing_count = 0;
        int ready_count = 0;
        int taken_off_count = 0;
        
        for (const auto& uav_name : uav_names_) {
            switch (uav_status_[uav_name]) {
                case INITIALIZING: initializing_count++; break;
                case READY: ready_count++; break;
                case TAKEN_OFF: taken_off_count++; break;
            }
        }
        
        ROS_INFO("UAV Status Summary: Initializing: %d, Ready: %d, Taken Off: %d", 
                initializing_count, ready_count, taken_off_count);
    }
    
    void checkAllRules() {
        if (uav_positions_.size() < 9) {
            return; // 等待所有无人机数据就绪
        }
        
        std::vector<geometry_msgs::Point> new_distance_violation_markers;
        
        // 检查已起飞无人机的最小间距规则
        std::vector<std::string> uav_names;
        for (const auto& pair : uav_positions_) {
            uav_names.push_back(pair.first);
        }
        
        for (size_t i = 0; i < uav_names.size(); i++) {
            for (size_t j = i + 1; j < uav_names.size(); j++) {
                // 仅当两架无人机都已起飞时才检查间距
                if (uav_status_[uav_names[i]] == TAKEN_OFF && uav_status_[uav_names[j]] == TAKEN_OFF) {
                    Eigen::Vector3d pos1 = uav_positions_[uav_names[i]];
                    Eigen::Vector3d pos2 = uav_positions_[uav_names[j]];
                    
                    double distance = calculateDistance(pos1, pos2);
                    
                    if (distance < min_distance_) {
                        // 统计违规次数
                        stats_.distance_violations++;
                        stats_.uav_distance_violations[uav_names[i]]++;
                        stats_.uav_distance_violations[uav_names[j]]++;
                        
                        // 在两架无人机中间创建标记点
                        geometry_msgs::Point violation_point;
                        violation_point.x = (pos1.x() + pos2.x()) / 2.0;
                        violation_point.y = (pos1.y() + pos2.y()) / 2.0;
                        violation_point.z = (pos1.z() + pos2.z()) / 2.0;
                        
                        // 检查是否在最小距离内已存在同类标记
                        if (!isNearExistingMarker(violation_point, distance_violation_markers_)) {
                            new_distance_violation_markers.push_back(violation_point);
                            distance_violation_markers_.push_back(violation_point);
                        }
                        
                        ROS_WARN("%s and %s are too close! Distance: %.3f m (min: %.1f m)", 
                                uav_names[i].c_str(), uav_names[j].c_str(), distance, min_distance_);
                    }
                }
                // 如果至少有一架无人机未起飞，则不检查间距（允许近距离）
            }
        }
        
        // 检查速度规则（对所有已初始化的无人机）
        for (const auto& uav_pair : uav_velocities_) {
            // 只检查READY和TAKEN_OFF状态的无人机
            if (uav_status_[uav_pair.first] != INITIALIZING) {
                double speed = calculateSpeed(uav_pair.second);
                if (speed > max_speed_) {
                    stats_.speed_violations++;
                    stats_.uav_speed_violations[uav_pair.first]++;
                    ROS_WARN("%s overspeed! Current speed: %.3f m/s (max: %.1f m/s)", 
                            uav_pair.first.c_str(), speed, max_speed_);
                }
            }
        }
        
        // 发布违规标记
        publishViolationMarkers(new_distance_violation_markers);
    }
    
    void checkMissionSuccess() {
        if (total_progress >= 1.0 && !mission_success_ && mission_started_) {
            mission_success_ = true;
            auto end_time = std::chrono::steady_clock::now();
            mission_duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                end_time - start_time_).count() / 1000.0;
            
            ROS_INFO("==========================================");
            ROS_INFO("MISSION SUCCESS!");
            ROS_INFO("Total exploration progress: %.1f%% (%d/%d cells)", 
                     total_progress * 100.0, total_explored_count_, total_grid_cells_);
            ROS_INFO("Mission completed in: %.3f seconds", mission_duration_);
            ROS_INFO("==========================================");
            
            // 打印任务统计信息
            printMissionSummary();
        }
    }
    
    void printMissionSummary() {
        ROS_INFO("MISSION SUMMARY:");
        ROS_INFO("Total mission time: %.3f seconds", mission_duration_);
        ROS_INFO("Rule violation statistics:");
        ROS_INFO("  - Speed violations: %d", stats_.speed_violations);
        ROS_INFO("  - Distance violations: %d", stats_.distance_violations);
        
        ROS_INFO("Final UAV status:");
        int initializing_count = 0;
        int ready_count = 0;
        int taken_off_count = 0;
        
        for (const auto& uav_name : uav_names_) {
            switch (uav_status_[uav_name]) {
                case INITIALIZING: initializing_count++; break;
                case READY: ready_count++; break;
                case TAKEN_OFF: taken_off_count++; break;
            }
            
            ROS_INFO("  %s: Status: %s, Speed violations: %d, Distance violations: %d", 
                    uav_name.c_str(), 
                    getStatusString(uav_status_[uav_name]).c_str(),
                    stats_.uav_speed_violations[uav_name],
                    stats_.uav_distance_violations[uav_name]);
        }
        
        ROS_INFO("Final status summary: Initializing: %d, Ready: %d, Taken Off: %d", 
                initializing_count, ready_count, taken_off_count);
        
        // 计算总违规次数
        int total_violations = stats_.speed_violations + stats_.distance_violations;
        
        if (total_violations == 0) {
            ROS_INFO("Perfect mission! No rule violations detected.");
        } else {
            ROS_INFO("⚠️  Total rule violations: %d", total_violations);
        }
    }
    
    std::string getStatusString(UAVStatus status) {
        switch (status) {
            case INITIALIZING: return "INITIALIZING";
            case READY: return "READY";
            case TAKEN_OFF: return "TAKEN_OFF";
            default: return "UNKNOWN";
        }
    }
    
    bool isNearExistingMarker(const geometry_msgs::Point& new_point, 
                             const std::vector<geometry_msgs::Point>& existing_markers) {
        for (const auto& existing_point : existing_markers) {
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
    
    void publishViolationMarkers(const std::vector<geometry_msgs::Point>& distance_violation_markers) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "distance_violations";
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(0);
        
        // 发布距离违规标记（黄色）
        if (!distance_violation_markers.empty()) {
            marker.id = 0;
            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.scale.z = 0.4;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.points = distance_violation_markers;
            marker_pub_.publish(marker);
        }
    }
    
    bool isPointInRegion(double x, double y, const ExplorationRegion& region) {
        return (x >= region.center.x() - region.width/2) &&
               (x <= region.center.x() + region.width/2) &&
               (y >= region.center.y() - region.height/2) &&
               (y <= region.center.y() + region.height/2);
    }
    
    bool markGridAsExploredInRegion(double world_x, double world_y, ExplorationRegion& region) {
        int grid_x, grid_y;
        if (worldToGridInRegion(world_x, world_y, grid_x, grid_y, region)) {
            int index = grid_y * region.grid_width + grid_x;
            if (region.grid_status[index] == UNEXPLORED) {
                region.grid_status[index] = EXPLORED;
                return true;
            }
        }
        return false;
    }
    
    bool worldToGridInRegion(double world_x, double world_y, int& grid_x, int& grid_y, const ExplorationRegion& region) {
        grid_x = static_cast<int>((world_x - (region.center.x() - region.width/2)) / region.grid_resolution);
        grid_y = static_cast<int>((world_y - (region.center.y() - region.height/2)) / region.grid_resolution);
        
        return (grid_x >= 0 && grid_x < region.grid_width && grid_y >= 0 && grid_y < region.grid_height);
    }
    
    bool updateExplorationStatusForRegion(ExplorationRegion& region, const Eigen::Vector3d& position) {
        bool updated = false;
        double fov_radius = fov_scale_ * position.z();
        
        double min_x = std::max(region.center.x() - region.width/2, position.x() - fov_radius);
        double max_x = std::min(region.center.x() + region.width/2, position.x() + fov_radius);
        double min_y = std::max(region.center.y() - region.height/2, position.y() - fov_radius);
        double max_y = std::min(region.center.y() + region.height/2, position.y() + fov_radius);
        
        for (double x = min_x; x <= max_x; x += region.grid_resolution) {
            for (double y = min_y; y <= max_y; y += region.grid_resolution) {
                double dx = x - position.x();
                double dy = y - position.y();
                if (dx*dx + dy*dy <= fov_radius * fov_radius) {
                    if (markGridAsExploredInRegion(x, y, region)) {
                        updated = true;
                    }
                }
            }
        }
        
        return updated;
    }
    
    void publishGridInfo(const ExplorationRegion& region, int region_index) {
        nav_msgs::OccupancyGrid grid_msg;
        
        grid_msg.header.stamp = ros::Time::now();
        grid_msg.header.frame_id = "world";
        
        grid_msg.info.resolution = region.grid_resolution;
        grid_msg.info.width = region.grid_width;
        grid_msg.info.height = region.grid_height;
        grid_msg.info.origin.position.x = region.center.x() - region.width/2;
        grid_msg.info.origin.position.y = region.center.y() - region.height/2;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;
        
        grid_msg.data.resize(region.grid_width * region.grid_height);
        
        for (int y = 0; y < region.grid_height; ++y) {
            for (int x = 0; x < region.grid_width; ++x) {
                int index = y * region.grid_width + x;
                int occupancy_index = y * region.grid_width + x;
                
                switch (region.grid_status[index]) {
                    case EXPLORED:
                        grid_msg.data[occupancy_index] = 0;    // 已探索
                        break;
                    case UNEXPLORED:
                    default:
                        grid_msg.data[occupancy_index] = -1;   // 未探索
                        break;
                }
            }
        }
        
        grid_info_pubs_[region_index].publish(grid_msg);
        
        // 计算并输出单个区域的探索进度（可选，用于调试）
        int explored_count = 0;
        for (const auto& status : region.grid_status) {
            if (status == EXPLORED) explored_count++;
        }
        double progress = static_cast<double>(explored_count) / region.grid_status.size();
        
        ROS_DEBUG_THROTTLE(10.0, "Region %s: %.1f%% explored (%d/%d)", 
                          region.grid_topic.c_str(), progress * 100.0, explored_count, region.grid_status.size());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_region_exploration_visualizer_with_rules");
    ExplorationVisualizer visualizer;
    ros::spin();
    return 0;
}