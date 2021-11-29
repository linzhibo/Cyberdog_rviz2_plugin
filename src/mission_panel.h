#ifndef MISSION_PANEL_H
#define MISSION_PANEL_H


#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <QVBoxLayout>
#include <QLabel>
#include <string>

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
  


  void setMissionStatus(const std::string text);

  // Here we declare some internal slots.
protected Q_SLOTS:
  void trigger_service(bool msg, std::string service_name);
protected:
  // One-line text editor for displaying the mission status.
  QLineEdit* output_status_editor_;

  // The current name of the output topic.
  QString output_status_;
  QVBoxLayout* layout;
  QHBoxLayout* iconlayout;
  QLabel* label;

  QString icon_on_path_;
  QString icon_off_path_;
  
  
};

} //namespace cyberdog_rviz2_control_plugin

#endif // MISSION_PANEL_H