#include "mission_panel.h"


namespace cyberdog_rviz2_control_plugin
{
MissionPanel::MissionPanel(QWidget* parent):rviz_common::Panel(parent)
{
  dummy_node_ = std::make_shared<DummyNode>();
  discover_dogs_ns();
  setFocusPolicy(Qt::ClickFocus);

  icon_off_path_ = QString::fromStdString(ament_index_cpp::get_package_share_directory("cyberdog_rviz2_control_plugin") + "/data/cd_off_64.jpg");
  icon_on_path_ = QString::fromStdString(ament_index_cpp::get_package_share_directory("cyberdog_rviz2_control_plugin") + "/data/cd_on_64.jpg");

  mode_client_ = rclcpp_action::create_client<motion_msgs::action::ChangeMode>(dummy_node_, dogs_namespace_ + "checkout_mode");
  gait_client_ = rclcpp_action::create_client<motion_msgs::action::ChangeGait>(dummy_node_, dogs_namespace_ + "checkout_gait");
  para_pub_ = dummy_node_->create_publisher<motion_msgs::msg::Parameters>(dogs_namespace_ + "para_change", rclcpp::SensorDataQoS());

  //interface 
  QVBoxLayout* layout = new QVBoxLayout;
  QHBoxLayout* mode_box_layout = new QHBoxLayout;

  label_ = new QLabel;
  QPixmap pic(icon_off_path_);
  label_->setPixmap(pic);
  mode_box_layout->addWidget(label_);

  // camera_switch_button_ = new SwitchButton(this, SwitchButton::Style::EMPTY);
  // camera_switch_button_->setServiceName(srv_name_camera_);
  stand_up_button_ = new QPushButton("Stand Up");
  get_down_button_ = new QPushButton("Get Down");

  mode_box_layout->addWidget(stand_up_button_);
  mode_box_layout->addWidget(get_down_button_);
  // mode_box_layout->addWidget(camera_switch_button_);

  gait_list_ = new GaitComboBox(this);

    //teleop layout
  QGroupBox *teleop_group_box = new QGroupBox(tr("🕹 Teleop layout"));
  teleop_button_ = new TeleopButton(this);
  // teleop_group_box->setLayout(teleop_button_);

  height_slider_ = new QSlider(Qt::Vertical);
  height_slider_->setMinimum(20);
  height_slider_->setMaximum(40);
  height_slider_->setValue(30);
  QHBoxLayout* teleop_layout = new QHBoxLayout;
  teleop_layout->addLayout(teleop_button_);
  teleop_layout->addWidget(height_slider_);
  teleop_group_box->setLayout(teleop_layout);
  
  //main layout
  layout->addLayout( mode_box_layout );
  layout->addLayout( gait_list_ );
  layout->addWidget( teleop_group_box );
  setLayout( layout );

  // connect( camera_switch_button_, SIGNAL(valueChanged(bool, std::string) ), this, SLOT( trigger_service(bool, std::string)));
  connect(gait_list_,SIGNAL(valueChanged(int)),SLOT(set_gait(int)));
  connect(stand_up_button_, &QPushButton::clicked, [this](void) { set_mode(1); });
  connect(get_down_button_, &QPushButton::clicked, [this](void) { set_mode(0); });
  connect(height_slider_,SIGNAL(valueChanged(int)),SLOT(set_height(int)));
}

void MissionPanel::set_height(int height)
{
  std::cout<<height<<std::endl;
}

void MissionPanel::discover_dogs_ns()
{
  std::string allowed_topic = "motion_msgs/msg/SE3VelocityCMD";
  auto topics_and_types = dummy_node_->get_topic_names_and_types();
  for (auto it : topics_and_types)
  {
    for (auto type: it.second)
    {
      if (type == allowed_topic)
      {
          std::string topic_name = it.first;
          topic_name = topic_name.erase(0,1);
          dogs_namespace_ = '/' + topic_name.substr(0, topic_name.find('/')+1);
          std::cout<<"Found mi dog's namespace: "<< dogs_namespace_ << std::endl;
          return;
      }
    }
  }
}

void MissionPanel::set_gait(int gait_id)
{
  std::cout<< gait_id<< std::endl;

  auto goal = motion_msgs::action::ChangeGait::Goal();
  goal.motivation = 253;
  goal.gaitstamped.timestamp = dummy_node_->get_clock()->now();
  goal.gaitstamped.gait = gait_id;
  auto goal_handle = gait_client_->async_send_goal(goal);
}

bool MissionPanel::event(QEvent *event)
{
  teleop_button_->key_to_button(event);
  return 0;
}

void MissionPanel::set_mode(int mode_id)
{
  if (!mode_client_->wait_for_action_server(std::chrono::seconds(1)))
  {
    std::cerr<<"Unable to find mode action server" << std::endl;
    return;
  }
  uint8_t mode = mode_id>0? motion_msgs::msg::Mode::MODE_MANUAL:motion_msgs::msg::Mode::MODE_DEFAULT;
  std::cout<<"mode: "<< (mode>0? "MANUAL":"DEFAULT " ) <<std::endl;
  auto goal = motion_msgs::action::ChangeMode::Goal();
  goal.modestamped.timestamp = dummy_node_->get_clock()->now();
  goal.modestamped.control_mode = mode;

  auto send_goal_options = rclcpp_action::Client<motion_msgs::action::ChangeMode>::SendGoalOptions();
  send_goal_options.result_callback = [&](const rclcpp_action::ClientGoalHandle<motion_msgs::action::ChangeMode>::WrappedResult &result) 
  {
    if (result.result->succeed) 
    {
      std::cout<<"Changed mode to "<< (mode_id>0? "MANUAL":"DEFAULT " )<< std::endl;
      QPixmap pic(mode_id>0 ? icon_on_path_:icon_off_path_);
      label_->setPixmap(pic);
    } 
    else 
    { 
      std::cerr<<"Unable to send ChangeMode action" << std::endl;
    }
  };
  auto goal_handle = mode_client_->async_send_goal(goal, send_goal_options);
}

void MissionPanel::trigger_service(bool msg, std::string service_name)
{
  auto client = dummy_node_->create_client<std_srvs::srv::SetBool>(service_name);
  if(!client->wait_for_service(std::chrono::seconds(1)))
  {
    std::cout<< "Service not found"<<std::endl;
    camera_switch_button_->setValue(true);
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = msg;
  
  auto result = client->async_send_request(request);
  if (!result.get().get()->success) 
  {
      std::cout<<" service call failed"<<std::endl;
      camera_switch_button_->setValue(true);
  } 
  else 
  {
      std::cout<<" service call success"<<std::endl;
  } 
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