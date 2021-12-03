
#ifndef TELEOP_WIDGET_H
#define TELEOP_WIDGET_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "motion_msgs/msg/se3_velocity_cmd.hpp"

#include <QWidget>
#include <QPushButton>
#include <QGridLayout>
#include <QLineEdit>
#include <QCheckBox>
#include <QLabel>
#include <QTimer>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QKeyEvent>
#include <QEvent>

namespace cyberdog_rviz2_control_plugin
{

class DummyNode : public rclcpp::Node
{
public:
    DummyNode():Node("dummy_node"){};
    ~DummyNode(){};
};

class TeleopButton : public QGridLayout
{
Q_OBJECT

public:
    TeleopButton(QWidget *parent = nullptr);
    void key_to_button(QEvent *event);

public Q_SLOTS:
    void setTopic( const QString& topic );

private Q_SLOTS:
    void discover_topics();
    void update_topic(int pos);
    void send_vel();
    void set_linear_speed(double value);
    void set_angular_speed(double value);    

private:
    QLabel output_topic_label_;

    QDoubleSpinBox linear_speed_box_;
    QDoubleSpinBox angular_speed_box_;
    QLabel v_label_;
    QLabel w_label_;

    QPushButton bt_q_, bt_w_, bt_e_;
    QPushButton bt_a_, bt_s_, bt_d_;
    QPushButton bt_z_, bt_x_, bt_c_;

    void set_vel(const char &key);

    QComboBox cmd_topic_box_;
    QPushButton discover_topic_;
    std::vector<QString> cmd_topic_list_;

    rclcpp::Publisher<motion_msgs::msg::SE3VelocityCMD>::SharedPtr cmd_pub_;

    double linear_velocity_;
    double target_linear_velocity_;
    double lateral_velocity_;
    double angular_velocity_;
    double target_angular_velocity_;
    bool cmd_topic_selected_ = false;
    std::shared_ptr<DummyNode> dummy_node_;
    
};

}
#endif // TELEOP_WIDGET_H