#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_tf_broadcaster");
    ros::NodeHandle nh;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    double x = 0.0;
    double y = 0.0;
    double z = 1.0;  // 初始高度
    double step = 0.02;  // 每次移动的距离
    ros::Rate rate(30);  // 控制帧率

    while (ros::ok()) {
        transform.setOrigin(tf::Vector3(x, y, z));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));  // 保持方向不变

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_frame"));
        
        y += step;  // 沿 X 轴移动
        rate.sleep();
    }
    return 0;
}
