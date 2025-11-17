#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <string>

class MultiUAVFOVVisualizer {
private:
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> odom_subs_;
    std::vector<ros::Publisher> marker_pubs_;
    std::string marker_frame_id_;
    std::vector<std::string> uav_names_;
    
public:
    MultiUAVFOVVisualizer() : marker_frame_id_("world") {
        // 从参数服务器获取帧ID（可选）
        nh_.param<std::string>("marker_frame_id", marker_frame_id_, "world");
        
        // 初始化无人机名称列表
        uav_names_ = {"uav2", "uav3", "uav4", "uav5", "uav6", "uav7", "uav8", "uav9", "uav10"};
        
        // 为每个无人机创建订阅者和发布者
        for (int i = 0; i < uav_names_.size(); ++i) {
            std::string uav_name = uav_names_[i];
            
            // 创建odometry订阅者
            std::string odom_topic = "/" + uav_name + "/sim/odom";
            ros::Subscriber odom_sub = nh_.subscribe<nav_msgs::Odometry>(
                odom_topic, 10, 
                boost::bind(&MultiUAVFOVVisualizer::odomCallback, this, _1, uav_name)
            );
            odom_subs_.push_back(odom_sub);
            
            // 创建marker发布者
            std::string marker_topic = "/" + uav_name + "/fov_marker";
            ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>(marker_topic, 10);
            marker_pubs_.push_back(marker_pub);
            
            ROS_INFO("Initialized for %s:", uav_name.c_str());
            ROS_INFO("  Subscribing to: %s", odom_topic.c_str());
            ROS_INFO("  Publishing to: %s", marker_topic.c_str());
        }
        
        ROS_INFO("Multi-UAV FOV Visualizer initialized for %zu UAVs", uav_names_.size());
        ROS_INFO("Using frame: %s", marker_frame_id_.c_str());
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& uav_name) {
        // 获取无人机位置
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        
        // 计算FOV半径 (r = 0.5 * z)
        double radius = 0.5 * z;
        
        // 创建圆形标记
        visualization_msgs::Marker fov_marker;
        fov_marker.header.frame_id = marker_frame_id_;
        fov_marker.header.stamp = ros::Time::now();
        fov_marker.ns = "fov_circle_" + uav_name;
        fov_marker.id = 0;
        fov_marker.type = visualization_msgs::Marker::LINE_STRIP;
        fov_marker.action = visualization_msgs::Marker::ADD;
        
        // 设置标记位置和方向
        fov_marker.pose.position.x = x;
        fov_marker.pose.position.y = y;
        fov_marker.pose.position.z = 0.0; // 在地面上
        
        // 设置为水平（绕Z轴旋转）
        tf2::Quaternion quat;
        quat.setRPY(0, 0, 0);
        fov_marker.pose.orientation = tf2::toMsg(quat);
        
        // 为不同无人机设置不同颜色
        setColorByUAV(fov_marker, uav_name);
        
        // 设置标记比例
        fov_marker.scale.x = 0.1; // 线宽
        
        fov_marker.lifetime = ros::Duration(0.5); // 自动消失时间
        
        // 生成圆形点
        int num_points = 36; // 圆的细分程度
        fov_marker.points.clear();
        
        for (int i = 0; i <= num_points; ++i) {
            double angle = 2.0 * M_PI * i / num_points;
            geometry_msgs::Point p;
            p.x = radius * cos(angle);
            p.y = radius * sin(angle);
            p.z = 0.0;
            fov_marker.points.push_back(p);
        }
        
        // 闭合圆形
        if (!fov_marker.points.empty()) {
            fov_marker.points.push_back(fov_marker.points[0]);
        }
        
        // 发布到对应的无人机话题
        publishToUAV(fov_marker, uav_name);
        
        ROS_DEBUG_THROTTLE(1.0, "Published FOV marker for %s: pos(%.2f, %.2f, %.2f), radius: %.2f", 
                          uav_name.c_str(), x, y, z, radius);
    }
    
private:
    void setColorByUAV(visualization_msgs::Marker& marker, const std::string& uav_name) {
        // 为不同无人机分配不同颜色
        if (uav_name == "uav2") {
            marker.color.r = 0.0; // 红色
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else if (uav_name == "uav3") {
            marker.color.r = 0.0; // 绿色
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else if (uav_name == "uav4") {
            marker.color.r = 0.0; // 蓝色
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else if (uav_name == "uav5") {
            marker.color.r = 0.0; // 黄色
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else if (uav_name == "uav6") {
            marker.color.r = 0.0; // 紫色
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else if (uav_name == "uav7") {
            marker.color.r = 0.0; // 青色
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else if (uav_name == "uav8") {
            marker.color.r = 0.0; // 青色
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else if (uav_name == "uav9") {
            marker.color.r = 0.0; // 青色
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else if (uav_name == "uav10") {
            marker.color.r = 0.0; // 青色
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        marker.color.a = 1.0; // 不透明度
    }
    
    void publishToUAV(const visualization_msgs::Marker& marker, const std::string& uav_name) {
        // 找到对应无人机的发布者并发布
        for (int i = 0; i < uav_names_.size(); ++i) {
            if (uav_names_[i] == uav_name) {
                marker_pubs_[i].publish(marker);
                break;
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_uav_fov_visualizer");
    MultiUAVFOVVisualizer visualizer;
    ros::spin();
    return 0;
}