#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>

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
    // ros::Publisher exploration_pub_;
    // ros::Publisher rectangle_pub_;
    // ros::Publisher obstacle_pub_;
    ros::Publisher grid_info_pub_;  // 新增：网格信息发布者
    
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
    std::vector<std::string> uav_names_;
    
    // FOV参数
    double fov_scale_;
    
    // 地图处理标志
    bool map_processed_;
    
public:
    ExplorationVisualizer() : 
        exploration_center_(0.0, 0.0),
        exploration_width_(50.0),
        exploration_height_(50.0),
        fov_scale_(0.5),
        grid_resolution_(0.1),
        map_processed_(false) {
        
        // 从参数服务器读取参数
        nh_.param<double>("grid_resolution", grid_resolution_, 0.1);
        nh_.param<double>("exploration_center_x", exploration_center_.x(), 0.0);
        nh_.param<double>("exploration_center_y", exploration_center_.y(), 0.0);
        nh_.param<double>("exploration_width", exploration_width_, 50.0);
        nh_.param<double>("exploration_height", exploration_height_, 50.0);
        nh_.param<double>("fov_scale", fov_scale_, 0.5);
        
        // 初始化网格
        updateGridSize();
        
        // 初始化无人机名称
        uav_names_ = {"uav2", "uav3", "uav4", "uav5", "uav6", "uav7"};
        
        // 创建发布者
        // exploration_pub_ = nh_.advertise<visualization_msgs::Marker>("exploration_status", 10, true);
        // rectangle_pub_ = nh_.advertise<visualization_msgs::Marker>("exploration_region", 10, true);
        // obstacle_pub_ = nh_.advertise<visualization_msgs::Marker>("obstacle_map", 10, true);
        grid_info_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("exploration_grid", 10, true);  // 新增
        
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
        }
        
        ROS_INFO("Exploration Visualizer initialized");
        ROS_INFO("Waiting for map data from /mock_map...");
    }
    
    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (map_processed_) {
            return;  // 地图已经处理过，避免重复处理
        }
        
        ROS_INFO("Received map point cloud with %d points", msg->width * msg->height);
        
        // 转换点云数据
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        // 处理障碍物
        processObstacles(cloud);
        
        // 发布障碍物可视化
        // publishObstacleMap();
        
        // 发布探索区域
        // publishExplorationRegion();
        
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
        
        // 更新探索状态
        updateExplorationStatus();
        
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
        grid_msg.info.origin.orientation.x = 0.0;
        grid_msg.info.origin.orientation.y = 0.0;
        grid_msg.info.origin.orientation.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;
        
        // 修复：正确的坐标映射
        grid_msg.data.resize(grid_width_ * grid_height_);
        
        for (int y = 0; y < grid_height_; ++y) {
            for (int x = 0; x < grid_width_; ++x) {
                int our_index = y * grid_width_ + x;
                
                // 修复：OccupancyGrid使用行优先存储，但Y轴是从上到下的
                // 所以我们需要将Y坐标翻转：grid_height_ - 1 - y
                int occupancy_index = y * grid_width_ + x;
                
                switch (grid_status_[our_index]) {
                    case OBSTACLE:
                        grid_msg.data[occupancy_index] = 100;  // 障碍物
                        break;
                    case EXPLORED:
                        grid_msg.data[occupancy_index] = 0;    // 已探索的自由空间
                        break;
                    case UNEXPLORED:
                    default:
                        grid_msg.data[occupancy_index] = -1;   // 未探索
                        break;
                }
            }
        }
        
        grid_info_pub_.publish(grid_msg);
        
        ROS_DEBUG_THROTTLE(5.0, "Published grid info: %dx%d, resolution: %.3f", 
                        grid_width_, grid_height_, grid_resolution_);
}
    
    void publishExplorationStatus() {
        // visualization_msgs::Marker explored_marker;
        // setupMarker(explored_marker, "explored_area", 0, visualization_msgs::Marker::CUBE_LIST);
        // explored_marker.scale.x = grid_resolution_;
        // explored_marker.scale.y = grid_resolution_;
        // explored_marker.scale.z = 0.01;
        // explored_marker.color.r = 0.0;
        // explored_marker.color.g = 1.0;
        // explored_marker.color.b = 0.0;
        // explored_marker.color.a = 0.8;

        // // std::cout << grid_resolution_ << std::endl;
        
        // // 添加所有已探索的网格
        // for (int y = 0; y < grid_height_; ++y) {
        //     for (int x = 0; x < grid_width_; ++x) {
        //         int index = y * grid_width_ + x;
        //         if (grid_status_[index] == EXPLORED) {
        //             Eigen::Vector2d world_pos = gridToWorld(x, y);
        //             geometry_msgs::Point p;
        //             p.x = world_pos.x();
        //             p.y = world_pos.y();
        //             p.z = 0.0;
        //             explored_marker.points.push_back(p);
        //         }
        //     }
        // }
        
        // exploration_pub_.publish(explored_marker);
        
        // 计算探索进度（排除障碍物）
        int explored_count = countGridsByStatus(EXPLORED);
        int obstacle_count = countGridsByStatus(OBSTACLE);
        int explorable_count = grid_status_.size() - obstacle_count;
        
        double progress = (explorable_count > 0) ? 
                         static_cast<double>(explored_count) / explorable_count : 0.0;
        
        ROS_INFO_THROTTLE(5.0, "Exploration progress: %.1f%% (%d/%d cells, %d obstacles)", 
                         progress * 100.0, explored_count, explorable_count, obstacle_count);
    }
    
    // void publishObstacleMap() {
    //     visualization_msgs::Marker obstacle_marker;
    //     setupMarker(obstacle_marker, "obstacles", 0, visualization_msgs::Marker::CUBE_LIST);
    //     obstacle_marker.scale.x = grid_resolution_;
    //     obstacle_marker.scale.y = grid_resolution_;
    //     obstacle_marker.scale.z = 0.02;  // 比探索区域稍高
    //     obstacle_marker.color.r = 0.5;   // 灰色
    //     obstacle_marker.color.g = 0.5;
    //     obstacle_marker.color.b = 0.5;
    //     obstacle_marker.color.a = 0.8;
        
    //     // 添加所有障碍物网格
    //     for (int y = 0; y < grid_height_; ++y) {
    //         for (int x = 0; x < grid_width_; ++x) {
    //             int index = y * grid_width_ + x;
    //             if (grid_status_[index] == OBSTACLE) {
    //                 Eigen::Vector2d world_pos = gridToWorld(x, y);
    //                 geometry_msgs::Point p;
    //                 p.x = world_pos.x();
    //                 p.y = world_pos.y();
    //                 p.z = 0.01;  // 稍微抬高避免z-fighting
    //                 obstacle_marker.points.push_back(p);
    //             }
    //         }
    //     }
        
    //     obstacle_pub_.publish(obstacle_marker);
    // }
    
    // void publishExplorationRegion() {
    //     visualization_msgs::Marker region_marker;
    //     setupMarker(region_marker, "exploration_region", 0, visualization_msgs::Marker::LINE_STRIP);
    //     region_marker.scale.x = 0.05;
    //     region_marker.color.r = 1.0;
    //     region_marker.color.g = 0.0;
    //     region_marker.color.b = 0.0;
    //     region_marker.color.a = 1.0;
        
    //     // 添加矩形顶点
    //     std::vector<Eigen::Vector2d> vertices = {
    //         Eigen::Vector2d(exploration_center_.x() - exploration_width_/2, exploration_center_.y() - exploration_height_/2),
    //         Eigen::Vector2d(exploration_center_.x() + exploration_width_/2, exploration_center_.y() - exploration_height_/2),
    //         Eigen::Vector2d(exploration_center_.x() + exploration_width_/2, exploration_center_.y() + exploration_height_/2),
    //         Eigen::Vector2d(exploration_center_.x() - exploration_width_/2, exploration_center_.y() + exploration_height_/2)
    //     };
        
    //     for (const auto& vertex : vertices) {
    //         geometry_msgs::Point p;
    //         p.x = vertex.x();
    //         p.y = vertex.y();
    //         p.z = 0.0;
    //         region_marker.points.push_back(p);
    //     }
        
    //     // 闭合矩形
    //     geometry_msgs::Point first_point;
    //     first_point.x = vertices[0].x();
    //     first_point.y = vertices[0].y();
    //     first_point.z = 0.0;
    //     region_marker.points.push_back(first_point);
        
    //     rectangle_pub_.publish(region_marker);
    // }
    
    void setupMarker(visualization_msgs::Marker& marker, const std::string& ns, int id, int type) {
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = type;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        marker.lifetime = ros::Duration();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_visualizer_with_obstacles");
    ExplorationVisualizer visualizer;
    ros::spin();
    return 0;
}