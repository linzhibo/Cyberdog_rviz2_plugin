#include "mission_panel.h"


namespace cyberdog_rviz2_control_plugin
{
MissionPanel::MissionPanel(QWidget* parent):rviz_common::Panel(parent)
{
  dummy_node_ = std::make_shared<DummyNode>("panel_dummy_node");
  discover_dogs_ns();
  setFocusPolicy(Qt::ClickFocus);

  icon_off_path_ = QString::fromStdString(ament_index_cpp::get_package_share_directory("cyberdog_rviz2_control_plugin") + "/data/cd_off_64.jpg");
  icon_on_path_ = QString::fromStdString(ament_index_cpp::get_package_share_directory("cyberdog_rviz2_control_plugin") + "/data/cd_on_64.jpg");

  audio_client_ = rclcpp_action::create_client<interaction_msgs::action::AudioPlay>(dummy_node_, dogs_namespace_ + "audio_play");
  mode_client_ = rclcpp_action::create_client<motion_msgs::action::ChangeMode>(dummy_node_, dogs_namespace_ + "checkout_mode");
  gait_client_ = rclcpp_action::create_client<motion_msgs::action::ChangeGait>(dummy_node_, dogs_namespace_ + "checkout_gait");
  order_client_ = rclcpp_action::create_client<motion_msgs::action::ExtMonOrder>(dummy_node_, dogs_namespace_ + "exe_monorder");
  para_pub_ = dummy_node_->create_publisher<motion_msgs::msg::Parameters>(dogs_namespace_ + "para_change", rclcpp::SensorDataQoS());
  token_pass_service_ = dummy_node_->create_client<interaction_msgs::srv::TokenPass>(dogs_namespace_ +"token_update");
  t2s_service_ = dummy_node_->create_client<interaction_msgs::srv::CameraService>("text_to_sound");

  //interface 
  QVBoxLayout* layout = new QVBoxLayout;
  QHBoxLayout* mode_box_layout = new QHBoxLayout;

  label_ = new QLabel;
  QPixmap pic(icon_off_path_);
  label_->setPixmap(pic);
  mode_box_layout->addWidget(label_);

  dog_switch_button_ = new SwitchButton(this, SwitchButton::Style::EMPTY);
  dog_switch_button_->setValue(false);
  stand_up_button_ = new QPushButton("Stand Up");
  stand_up_button_->setEnabled(false);
  get_down_button_ = new QPushButton("Get Down");
  get_down_button_->setEnabled(false);

  mode_box_layout->addWidget(dog_switch_button_);
  mode_box_layout->addWidget(stand_up_button_);
  mode_box_layout->addWidget(get_down_button_);

  // order layout
  order_list_ = new OrderComboBox(this);

  // gait layout
  gait_list_ = new GaitComboBox(this);

  // wav_input layout
  QHBoxLayout* wav_layout = new QHBoxLayout;
  QLabel* void_label = new QLabel("🎵  Music:");
  wav_layout->addWidget(void_label);

  QSlider* vol_slider = new QSlider(Qt::Horizontal);
  vol_slider->setMinimum(0);
  vol_slider->setMaximum(10);
  vol_slider->setValue(5);
  vol_slider->setFixedWidth(60);
  wav_layout->addWidget(vol_slider);

  wav_input_ = new QLineEdit(this);
  wav_input_->setFixedWidth(80);
  wav_input_->setValidator( new QIntValidator(0, 999, this) );
  wav_layout->addWidget(wav_input_);
  play_button_ = new QPushButton("Play");
  play_button_->setFixedWidth(80);
  wav_layout->addWidget(play_button_);

  // text input layout
  QHBoxLayout* t2s_layout = new QHBoxLayout;
  text_input_ = new QLineEdit(this);
  t2s_layout->addWidget(text_input_);
  t2s_button_ = new QPushButton("Text to speech");
  t2s_button_->setFixedWidth(100);
  t2s_layout->addWidget(t2s_button_);

  // teleop layout
  QGroupBox *teleop_group_box = new QGroupBox(tr("🕹  Teleop layout"));
  teleop_button_ = new TeleopButton(this);
  
  height_slider_ = new QSlider(Qt::Vertical);
  height_slider_->setMinimum(10);
  height_slider_->setMaximum(40);
  height_slider_->setValue(30);
  height_label_ = new QLabel(QString::number(30));
  QVBoxLayout* height_layout = new QVBoxLayout;
  height_layout->addWidget(height_label_);
  height_layout->addWidget(height_slider_);

  QHBoxLayout* teleop_layout = new QHBoxLayout;
  teleop_layout->addLayout(teleop_button_);
  teleop_layout->addLayout(height_layout);
  teleop_group_box->setLayout(teleop_layout);
  
  //main layout
  layout->addLayout( mode_box_layout );
  layout->addLayout( gait_list_, Qt::AlignLeft);
  layout->addLayout( order_list_, Qt::AlignLeft);
  layout->addLayout( wav_layout );
  layout->addLayout( t2s_layout );
  layout->addWidget( teleop_group_box );
  setLayout( layout );

  connect(dog_switch_button_, SIGNAL(valueChanged(bool) ), this, SLOT( set_dog_status(bool)));
  connect(gait_list_,SIGNAL(valueChanged(int)),SLOT(set_gait(int)));
  connect(stand_up_button_, &QPushButton::clicked, [this](void) { set_mode(1); });
  connect(get_down_button_, &QPushButton::clicked, [this](void) { set_mode(0); });
  connect(height_slider_,SIGNAL(valueChanged(int)),SLOT(set_height(int)));
  connect(order_list_,SIGNAL(valueChanged(int)),SLOT(set_order_id(int)));
  connect(order_list_, &OrderComboBox::clicked, [this](void) { send_order(); });
  connect(wav_input_, SIGNAL(editingFinished()), this, SLOT( set_wav_id()) );
  connect(play_button_, &QPushButton::clicked, [this](void) { play_wav(); });
  connect(vol_slider, SIGNAL(valueChanged(int)), SLOT(set_volume(int)));
  connect(text_input_, SIGNAL(editingFinished()), this, SLOT( set_t2s_text()) );
  connect(t2s_button_, &QPushButton::clicked, [this](void) { play_t2s(); });
}

void MissionPanel::play_t2s()
{
  if (text_ready_to_speech_ == "")
    return;

  auto request = std::make_shared<interaction_msgs::srv::CameraService::Request>();
  request->args = text_ready_to_speech_;
  auto result =t2s_service_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(dummy_node_, result) ==rclcpp::FutureReturnCode::SUCCESS)
  {
    if(result.get()->result == 1)
    {
      std::cout<<"text to speech service successful, playing now: "<< text_ready_to_speech_<<std::endl;
      auto audio_goal = interaction_msgs::action::AudioPlay::Goal();
      audio_goal.order.name.id = 777;
      audio_goal.order.user.id = 4;
      auto audio_goal_handle = audio_client_->async_send_goal(audio_goal); 
    }
  } 
  else 
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service t2s");
  }
  
  // if(response.get()->result)
  // {
  //   std::cout<<"response.result: "<< response.get()->result <<std::endl;
  //   std::cout<<"playing: "<< text_ready_to_speech_ <<std::endl;
    // auto audio_goal = interaction_msgs::action::AudioPlay::Goal();
    // audio_goal.order.name.id = 777;
    // audio_goal.order.user.id = 4;
    // auto audio_goal_handle = audio_client_->async_send_goal(audio_goal); 
  // }
}

void MissionPanel::set_t2s_text()
{
  text_ready_to_speech_ = text_input_->text().toStdString(); 
}

void MissionPanel::set_volume(int vol)
{
  std::cout<< "set volume to: "<< vol << std::endl;
  auto request = std::make_shared<interaction_msgs::srv::TokenPass::Request>();
  request->ask = request->ASK_SET_VOLUME;
  request->vol = vol;
  auto result = token_pass_service_->async_send_request(request);
}

void MissionPanel::play_wav()
{
  std::cout<<"playing: "<< wav_id_ <<std::endl;
  auto audio_goal = interaction_msgs::action::AudioPlay::Goal();
  audio_goal.order.name.id = wav_id_;
  audio_goal.order.user.id = 4;
  auto audio_goal_handle = audio_client_->async_send_goal(audio_goal); 
}

void MissionPanel::set_wav_id()
{
  wav_id_ = wav_input_->text().toInt(); 
}

void MissionPanel::set_order_id(int order_id)
{
  order_id_ = order_id;
}

void MissionPanel::send_order()
{
  auto goal = motion_msgs::action::ExtMonOrder::Goal();
  goal.orderstamped.timestamp = dummy_node_->get_clock()->now();
  goal.orderstamped.id = order_id_;
  goal.orderstamped.para = 0.0;
  auto goal_handle = order_client_->async_send_goal(goal);

  // if (order_id_ == motion_msgs::msg::MonOrder::MONO_ORDER_DANCE)
  // {
  //   std::cout<<"send_order: "<< order_id_ <<std::endl;
  //   auto audio_goal = interaction_msgs::action::AudioPlay::Goal();
  //   audio_goal.order.name.id = 888;
  //   audio_goal.order.user.id = 4;
  //   auto audio_goal_handle = audio_client_->async_send_goal(audio_goal);
  // }

  // if (order_id_ == motion_msgs::msg::MonOrder::MONO_ORDER_SHOW)
  // {
  //   std::cout<<"send_order: "<< order_id_ <<std::endl;
  //   auto audio_goal = interaction_msgs::action::AudioPlay::Goal();
  //   audio_goal.order.name.id = 666;
  //   audio_goal.order.user.id = 4;
  //   auto audio_goal_handle = audio_client_->async_send_goal(audio_goal);
  // }
}

void MissionPanel::set_dog_status(bool msg)
{
  if(msg)
  {
    std::cout<< "set_dog_status "<< msg <<std::endl;
    label_->setPixmap(QPixmap(icon_on_path_));
    stand_up_button_->setEnabled(true);
    get_down_button_->setEnabled(true);
    
  }
  else
  {
    std::cout<< "set_dog_status "<< msg <<std::endl;
    label_->setPixmap(QPixmap(icon_off_path_));
    stand_up_button_->setEnabled(false);
    get_down_button_->setEnabled(false);
  }
}

void MissionPanel::set_height(int height)
{
  motion_msgs::msg::Parameters param;
  param.gait_height = 0.8;
  param.body_height = (float)height/100.0;
  param.timestamp = dummy_node_->now();;
  para_pub_->publish(param);
  height_label_->setText(QString::number(height));
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
  std::cout<<"Did nod found mi dog's namespace, default: "<< dogs_namespace_ << std::endl;
}

void MissionPanel::set_gait(int gait_id)
{
  // std::cout<< gait_id<< std::endl;

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
  if (!mode_client_->wait_for_action_server(std::chrono::milliseconds(500)))
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