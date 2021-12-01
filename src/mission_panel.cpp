#include "mission_panel.h"


namespace cyberdog_rviz2_control_plugin
{
MissionPanel::MissionPanel(QWidget* parent):rviz_common::Panel(parent)
{
  dummy_node_ = std::make_shared<DummyNode>();
  setFocusPolicy(Qt::ClickFocus);

  icon_off_path_ = QString::fromStdString(ament_index_cpp::get_package_share_directory("cyberdog_rviz2_control_plugin") + "/data/cd_off_64.jpg");
  icon_on_path_ = QString::fromStdString(ament_index_cpp::get_package_share_directory("cyberdog_rviz2_control_plugin") + "/data/cd_on_64.jpg");

  mode_client_ = rclcpp_action::create_client<motion_msgs::action::ChangeMode>(dummy_node_,"checkout_mode");

  //interface 
  QVBoxLayout* layout = new QVBoxLayout;
  QHBoxLayout* mode_box_layout = new QHBoxLayout;

  label_ = new QLabel;
  QPixmap pic(icon_off_path_);
  label_->setPixmap(pic);
  mode_box_layout->addWidget(label_);

  // switch_button_ = new SwitchButton(this, SwitchButton::Style::EMPTY);
  // mode_box_layout->addWidget(switch_button_);
  mode_button_ = new QPushButton("Activate");
  mode_button_->setCheckable(true);
  mode_box_layout->addWidget(mode_button_);

  teleop_button_ = new TeleopButton(this);
  
  layout->addLayout( mode_box_layout );
  layout->addLayout( teleop_button_ );
  setLayout( layout );

  // connect( switch_button_, SIGNAL(valueChanged(bool, std::string) ), this, SLOT( trigger_action(bool) ));
  connect(mode_button_, &QPushButton::clicked, [this](void) { trigger_action(1); });
}

bool MissionPanel::event(QEvent *event)
{
  teleop_button_->key_to_button(event);
  return 0;
}

void MissionPanel::trigger_action(bool state)
{
  if (!mode_button_->isDown())
    mode_button_->setDown(true);
  auto mode = state>0? motion_msgs::msg::Mode::MODE_MANUAL:motion_msgs::msg::Mode::MODE_LOCK;
  std::cout<<"mode"<< mode <<std::endl;
  auto goal = motion_msgs::action::ChangeMode::Goal();
  goal.modestamped.control_mode = mode;

  auto send_goal_options = rclcpp_action::Client<motion_msgs::action::ChangeMode>::SendGoalOptions();
  send_goal_options.result_callback = [&](const rclcpp_action::ClientGoalHandle<motion_msgs::action::ChangeMode>::WrappedResult &result) 
  {
    if (result.result->succeed) 
    {
      mode_button_->setDown(false);
      std::cout<<"Changed mode to "<< (state>0? "MANUAL":"LOCK " )<< std::endl;
      QPixmap pic(state>0 ? icon_on_path_:icon_off_path_);
      label_->setPixmap(pic);
    } 
    else 
    { 
      std::cerr<<"Unable to send ChangeMode action" << std::endl;
      // switch_button_->setValue(true);
    }
  };
  auto goal_handle = mode_client_->async_send_goal(goal, send_goal_options);
}

void MissionPanel::trigger_service(bool msg, std::string service_name)
{
  QPixmap pic(msg>0 ? icon_on_path_:icon_off_path_);
  label_->setPixmap(pic);
  std::cout<<"\n"<< msg<< service_name <<std::endl;
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