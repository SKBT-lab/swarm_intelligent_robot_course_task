#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <cmath>

class MarkerPublisher {
private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    visualization_msgs::Marker marker_;
    std::string marker_ns_;
    int marker_id_;
    double rate_;

public:
    MarkerPublisher() : marker_ns_("basic_shapes"), marker_id_(0), rate_(1.0) {
        // 创建发布者
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("Target_Point", 10);
        
        // 初始化marker的基本属性
        initializeMarker();
        
        ROS_INFO("Marker publisher initialized");
    }

    void initializeMarker() {
        // 设置frame ID和时间戳
        marker_.header.frame_id = "world";
        marker_.header.stamp = ros::Time::now();
        
        // 设置命名空间和ID
        marker_.ns = marker_ns_;
        marker_.id = marker_id_;
        
        // 设置marker类型为立方体
        marker_.type = visualization_msgs::Marker::CUBE;
        
        // 设置动作：ADD/MODIFY/DELETE
        marker_.action = visualization_msgs::Marker::ADD;
        
        // 设置位置
        marker_.pose.position.x = 0;
        marker_.pose.position.y = 0;
        marker_.pose.position.z = 0;
        marker_.pose.orientation.x = 0.0;
        marker_.pose.orientation.y = 0.0;
        marker_.pose.orientation.z = 0.0;
        marker_.pose.orientation.w = 1.0;
        
        // 设置尺寸 (单位：米)
        marker_.scale.x = 1.0;
        marker_.scale.y = 1.0;
        marker_.scale.z = 1.0;
        
        // 设置颜色 (RGBA)
        marker_.color.r = 0.0f;
        marker_.color.g = 1.0f;
        marker_.color.b = 0.0f;
        marker_.color.a = 1.0;  // 不透明度
        
        // 设置生存时间 (0表示无限)
        marker_.lifetime = ros::Duration();
    }

    void publishMarker() {
        ros::Rate rate(rate_);
        int count = 0;
        
        while (ros::ok()) {
            // 更新header的时间戳
            marker_.header.stamp = ros::Time::now();
            
            // // 添加一些动态效果 - 让marker旋转和变色
            // double angle = count * 0.1;
            // marker_.pose.position.x = 2.0 * cos(angle);
            // marker_.pose.position.y = 2.0 * sin(angle);
            
            // // 颜色变化
            // marker_.color.r = (sin(angle) + 1.0) / 2.0;
            // marker_.color.g = (cos(angle) + 1.0) / 2.0;
            // marker_.color.b = 0.5;
            
            // 发布marker
            marker_pub_.publish(marker_);
            
            // ROS_INFO_THROTTLE(2.0, "Published marker at position: (%.2f, %.2f, %.2f)", 
            //                  marker_.pose.position.x, 
            //                  marker_.pose.position.y, 
            //                  marker_.pose.position.z);
            
            count++;
            rate.sleep();
        }
    }

    // 设置marker位置
    void setPosition(double x, double y, double z) {
        marker_.pose.position.x = x;
        marker_.pose.position.y = y;
        marker_.pose.position.z = z;
    }

    // 设置marker颜色
    void setColor(float r, float g, float b, float a) {
        marker_.color.r = r;
        marker_.color.g = g;
        marker_.color.b = b;
        marker_.color.a = a;
    }

    // 设置marker尺寸
    void setScale(double x, double y, double z) {
        marker_.scale.x = x;
        marker_.scale.y = y;
        marker_.scale.z = z;
    }

    // 设置marker类型
    void setType(int type) {
        marker_.type = type;
    }
};

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "marker_publisher");
    
    // 创建发布者对象
    MarkerPublisher publisher;
    
    // 设置marker属性
    publisher.setPosition(30, 30, 2.5);
    publisher.setColor(1.0, 0.0, 0.0, 1.0);  // 红色
    publisher.setScale(1, 1, 1);       // 尺寸
    publisher.setType(visualization_msgs::Marker::SPHERE);  // 球体
    
    // 开始发布
    publisher.publishMarker();
    
    return 0;
}