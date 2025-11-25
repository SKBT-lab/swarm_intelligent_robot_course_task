#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>

class TargetPublisher : public QWidget {
    Q_OBJECT

public:
    TargetPublisher(QWidget *parent = nullptr) : QWidget(parent) {
        // ROS节点初始化
        ros::NodeHandle nh;
        target_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

        // 界面布局
        QVBoxLayout *mainLayout = new QVBoxLayout(this);

        QLabel *label = new QLabel("Enter Target Coordinates:", this);
        mainLayout->addWidget(label);

        QFormLayout *formLayout = new QFormLayout;
        input_x_ = new QLineEdit(this);
        input_y_ = new QLineEdit(this);
        input_z_ = new QLineEdit(this);
        formLayout->addRow("X:", input_x_);
        formLayout->addRow("Y:", input_y_);
        formLayout->addRow("Z:", input_z_);
        mainLayout->addLayout(formLayout);

        publish_button_ = new QPushButton("Publish", this);
        mainLayout->addWidget(publish_button_);

        connect(publish_button_, &QPushButton::clicked, this, &TargetPublisher::publishTarget);
    }

private slots:
    void publishTarget() {
        // 获取输入的目标坐标
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.frame_id = "map";
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.pose.position.x = input_x_->text().toDouble();
        goal_msg.pose.position.y = input_y_->text().toDouble();
        goal_msg.pose.position.z = input_z_->text().toDouble();
        goal_msg.pose.orientation.w = 1.0;

        // 发布目标坐标
        target_pub_.publish(goal_msg);

        ROS_INFO("Published target: [%.2f, %.2f, %.2f]",
                 goal_msg.pose.position.x,
                 goal_msg.pose.position.y,
                 goal_msg.pose.position.z);
    }

private:
    QLineEdit *input_x_;
    QLineEdit *input_y_;
    QLineEdit *input_z_;
    QPushButton *publish_button_;
    ros::Publisher target_pub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "publish_target");

    QApplication app(argc, argv);
    TargetPublisher window;
    window.setWindowTitle("Target Publisher");
    window.show();

    return app.exec();
}
#include "publish_target.moc"
