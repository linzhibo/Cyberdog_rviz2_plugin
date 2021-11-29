#include "mission_panel.h"

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPixmap>
#include "switch.h"

namespace cyberdog_rviz2_control_plugin
{
MissionPanel::MissionPanel( QWidget* parent )
  : rviz_common::Panel( parent )

{
  icon_off_path_ = QString::fromStdString(ament_index_cpp::get_package_share_directory("cyberdog_rviz2_control_plugin") + "/data/cd_off_64.jpg");
  icon_on_path_ = QString::fromStdString(ament_index_cpp::get_package_share_directory("cyberdog_rviz2_control_plugin") + "/data/cd_on_64.jpg");

  QHBoxLayout* status_layout = new QHBoxLayout;
  label = new QLabel;
  QPixmap pic(icon_off_path_);
  label->setPixmap(pic);
  status_layout->addWidget(label);

  SwitchButton* switch_button = new SwitchButton(this);
  status_layout->addWidget(switch_button);

  layout = new QVBoxLayout;
  layout->addLayout( status_layout );;
  setLayout( layout );

  connect( switch_button, SIGNAL(valueChanged(bool, std::string) ), this, SLOT( trigger_service(bool, std::string) ));
}

void MissionPanel::trigger_service(bool msg, std::string service_name)
{

  QPixmap pic(msg>0 ? icon_on_path_:icon_off_path_);
  label->setPixmap(pic);
  std::cout<<"\n"<< service_name<< std::endl;
  // setLayout( layout );
  // if (client.call(srv))
  // {
  //   std::cout<<"\n"<< (switch_msg>0?"enable " + service_name :"disable " + service_name)<< std::endl;
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service: %s", service_name.c_str());
  //   if (service_name == srv_name_cmd_)
  //     switch_button_->setValue(true);
  // }  
}


void MissionPanel::save( rviz_common::Config config ) const
{
  rviz_common::Panel::save( config );
}

// Load all configuration data for this panel from the given Config object.
void MissionPanel::load( const rviz_common::Config& config )
{
  rviz_common::Panel::load( config );
}

} // namespace cyberdog_rviz2_control_plugin


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cyberdog_rviz2_control_plugin::MissionPanel, rviz_common::Panel)