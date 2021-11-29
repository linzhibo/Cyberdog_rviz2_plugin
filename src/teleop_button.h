
#ifndef TELEOP_WIDGET_H
#define TELEOP_WIDGET_H

#include <rclcpp/rclcpp.hpp>
#include <rviz/panel.h>
#include <geometry_msgs/Twist.h>

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

namespace escooter_control_panel
{
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

    // QLineEdit output_topic_editor_;
    QComboBox cmd_topic_box_;
    QPushButton discover_topic_;
    std::vector<QString> cmd_topic_list_;

    ros::NodeHandle nh_;
    ros::Publisher velocity_publisher_;

    double linear_velocity_, target_linear_velocity_;
    double angular_velocity_, target_angular_velocity_;
};

}
#endif // TELEOP_WIDGET_H