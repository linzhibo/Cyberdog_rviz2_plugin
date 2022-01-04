#ifndef MISSION_PANEL_H
#define MISSION_PANEL_H


#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include <QPainter>
#include <QLineEdit>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPixmap>

#include "motion_msgs/action/change_mode.hpp"
#include "motion_msgs/action/change_gait.hpp"
#include "motion_msgs/action/ext_mon_order.hpp"
#include "motion_msgs/msg/parameters.hpp"

#include "switch.h"
#include "teleop_button.h"
#include "gait_combo_box.h"

class QLineEdit;
namespace cyberdog_rviz2_control_plugin
{

class MissionPanel: public rviz_common::Panel
{
Q_OBJECT
public:
  MissionPanel( QWidget* parent = 0 );

  virtual void load( const rviz_common::Config& config );
  virtual void save( rviz_common::Config config ) const;
  
protected Q_SLOTS:
  void trigger_service(bool msg, std::string service_name);
  void set_mode(int mode_id);
  void set_gait(int gait_id);
  void set_height(int height);

protected:
  bool event(QEvent *event);

private:
  void discover_dogs_ns();
  std::shared_ptr<DummyNode> dummy_node_;

  rclcpp_action::Client<motion_msgs::action::ChangeMode>::SharedPtr mode_client_;
  rclcpp_action::Client<motion_msgs::action::ChangeGait>::SharedPtr gait_client_;
  rclcpp::Publisher<motion_msgs::msg::Parameters>::SharedPtr para_pub_;

  QString icon_on_path_;
  QString icon_off_path_;
  TeleopButton* teleop_button_;
  SwitchButton* camera_switch_button_;
  QPushButton* stand_up_button_;
  QPushButton* get_down_button_;
  QLabel* label_;
  GaitComboBox* gait_list_;
  QSlider* height_slider_;
  
  std::string srv_name_camera_ = "camera/enable";
  std::string dogs_namespace_ = "/mi1046017/";
};

} //namespace cyberdog_rviz2_control_plugin

#endif // MISSION_PANEL_H