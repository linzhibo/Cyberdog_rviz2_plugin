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

#include "interaction_msgs/action/audio_play.hpp"
#include "interaction_msgs/srv/token_pass.hpp"
#include "interaction_msgs/srv/camera_service.hpp"
#include "motion_msgs/action/change_mode.hpp"
#include "motion_msgs/action/change_gait.hpp"
#include "motion_msgs/action/ext_mon_order.hpp"
#include "motion_msgs/msg/parameters.hpp"

#include "switch.h"
#include "teleop_button.h"
#include "gait_combo_box.h"
#include "order_combo_box.h"

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
  void set_dog_status(bool msg);
  void set_mode(int mode_id);
  void set_gait(int gait_id);
  void set_height(int height);
  void set_order_id(int order_id);
  void send_order();
  void set_wav_id();
  void play_wav();
  void set_volume(int vol);
  void set_t2s_text();
  void play_t2s();

protected:
  bool event(QEvent *event);

private:
  void discover_dogs_ns();
  std::shared_ptr<DummyNode> dummy_node_;

  rclcpp_action::Client<interaction_msgs::action::AudioPlay>::SharedPtr audio_client_;
  rclcpp::Client<interaction_msgs::srv::TokenPass>::SharedPtr token_pass_service_;
  rclcpp_action::Client<motion_msgs::action::ChangeMode>::SharedPtr mode_client_;
  rclcpp_action::Client<motion_msgs::action::ChangeGait>::SharedPtr gait_client_;
  rclcpp_action::Client<motion_msgs::action::ExtMonOrder>::SharedPtr order_client_;
  rclcpp::Publisher<motion_msgs::msg::Parameters>::SharedPtr para_pub_;
  rclcpp::Client<interaction_msgs::srv::CameraService>::SharedPtr t2s_service_;

  QString icon_on_path_;
  QString icon_off_path_;
  TeleopButton* teleop_button_;
  SwitchButton* camera_switch_button_;
  SwitchButton* dog_switch_button_;
  QPushButton* stand_up_button_;
  QPushButton* get_down_button_;
  QLabel* label_;
  QLabel* height_label_;
  GaitComboBox* gait_list_;
  OrderComboBox* order_list_;
  QSlider* height_slider_;
  QLineEdit* wav_input_, *text_input_;
  QPushButton* play_button_, *t2s_button_;
  
  std::string srv_name_camera_ = "camera/enable";
  std::string dogs_namespace_ = "/mi123456789/";
  int order_id_ = 0;
  int wav_id_ = 0;
  std::string text_ready_to_speech_ = "";
};

} //namespace cyberdog_rviz2_control_plugin

#endif // MISSION_PANEL_H