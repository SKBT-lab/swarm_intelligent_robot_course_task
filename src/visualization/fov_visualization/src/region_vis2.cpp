#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>
#include <random>

enum GridStatus {
    UNEXPLORED = 0,
    EXPLORED = 1
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
    
    std::vector<ExplorationRegion> regions_;
    std::map<std::string, Eigen::Vector3d> uav_positions_;
    std::vector<std::string> uav_names_;
    
    double fov_scale_;

    // 随机数生成器
    std::random_device rd_;
    std::mt19937 gen_;
    
public:
    ExplorationVisualizer() : fov_scale_(0.5), gen_(getTrueRandomSeed()) {
        ROS_INFO("Random seed initialized for true random behavior");
        
        // 初始化三个矩形区域（使用真随机参数）
        initializeRegions();
        
        // 初始化9个无人机名称
        for (int i = 2; i <= 10; ++i) {
            uav_names_.push_back("uav" + std::to_string(i));
        }
        
        // 为每个区域创建发布者
        for (const auto& region : regions_) {
            ros::Publisher pub = nh_.advertise<nav_msgs::OccupancyGrid>(region.grid_topic, 10, true);
            grid_info_pubs_.push_back(pub);
        }
        
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
        
        ROS_INFO("Multi-Region Exploration Visualizer initialized");
        ROS_INFO("Monitoring %zu regions with %zu UAVs", regions_.size(), uav_names_.size());
    }
    
private:
    unsigned int getTrueRandomSeed() {
        // 方法1: 使用random_device
        unsigned int seed = rd_();
        ROS_INFO("Generated true random seed: %u", seed);
        return seed;
        
        // 方法2: 结合时间戳（备选方案）
        // auto now = std::chrono::high_resolution_clock::now();
        // unsigned int time_seed = static_cast<unsigned int>(now.time_since_epoch().count());
        // unsigned int combined_seed = rd_() ^ time_seed;
        // return combined_seed;
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
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& uav_name) {
        Eigen::Vector3d position(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        
        uav_positions_[uav_name] = position;
        
        // 为每个区域更新探索状态
        for (size_t i = 0; i < regions_.size(); ++i) {
            updateExplorationStatusForRegion(regions_[i], position);
            publishGridInfo(regions_[i], i);
        }
        
        ROS_DEBUG_THROTTLE(2.0, "Updated position for %s: (%.2f, %.2f, %.2f)", 
                          uav_name.c_str(), position.x(), position.y(), position.z());
    }
    
    bool isPointInRegion(double x, double y, const ExplorationRegion& region) {
        return (x >= region.center.x() - region.width/2) &&
               (x <= region.center.x() + region.width/2) &&
               (y >= region.center.y() - region.height/2) &&
               (y <= region.center.y() + region.height/2);
    }
    
    void markGridAsExploredInRegion(double world_x, double world_y, ExplorationRegion& region) {
        int grid_x, grid_y;
        if (worldToGridInRegion(world_x, world_y, grid_x, grid_y, region)) {
            int index = grid_y * region.grid_width + grid_x;
            region.grid_status[index] = EXPLORED;
        }
    }
    
    bool worldToGridInRegion(double world_x, double world_y, int& grid_x, int& grid_y, const ExplorationRegion& region) {
        grid_x = static_cast<int>((world_x - (region.center.x() - region.width/2)) / region.grid_resolution);
        grid_y = static_cast<int>((world_y - (region.center.y() - region.height/2)) / region.grid_resolution);
        
        return (grid_x >= 0 && grid_x < region.grid_width && grid_y >= 0 && grid_y < region.grid_height);
    }
    
    void updateExplorationStatusForRegion(ExplorationRegion& region, const Eigen::Vector3d& position) {
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
                    markGridAsExploredInRegion(x, y, region);
                }
            }
        }
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
        grid_msg.info.origin.orientation.x = 0.0;
        grid_msg.info.origin.orientation.y = 0.0;
        grid_msg.info.origin.orientation.z = 0.0;
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
        
        // 计算并输出探索进度
        int explored_count = 0;
        for (const auto& status : region.grid_status) {
            if (status == EXPLORED) explored_count++;
        }
        double progress = static_cast<double>(explored_count) / region.grid_status.size();
        
        ROS_DEBUG_THROTTLE(5.0, "Region %s: %.1f%% explored (%d/%d)", 
                          region.grid_topic.c_str(), progress * 100.0, explored_count, region.grid_status.size());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_region_exploration_visualizer");
    ExplorationVisualizer visualizer;
    ros::spin();
    return 0;
}