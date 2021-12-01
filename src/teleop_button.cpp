#include "teleop_button.h"

namespace cyberdog_rviz2_control_plugin
{
TeleopButton::TeleopButton(QWidget*)
    : QGridLayout(),
    v_label_("speed(m/s)"),
    w_label_("steer(rad/s)"),
    bt_q_("↖"), bt_w_("↑"), bt_e_("↗"),
    bt_a_("←"), bt_s_("STOP"), bt_d_("→"),
    bt_z_("↙"), bt_x_("↓"), bt_c_("↘"),
    discover_topic_("Detect"),
    target_linear_velocity_(1.0),
    target_angular_velocity_(0.5)
    
{
    dummy_node_ = std::make_shared<DummyNode>();

    this->addWidget(&discover_topic_, 0, 0);
    this->addWidget(&cmd_topic_box_, 0, 1);

    linear_speed_box_.setRange(0, 2.0);
    linear_speed_box_.setValue(target_linear_velocity_);
    linear_speed_box_.setSingleStep(0.1);
    angular_speed_box_.setRange(0, 1.0);
    angular_speed_box_.setValue(target_angular_velocity_);
    angular_speed_box_.setSingleStep(0.1);
    this->addWidget(&v_label_, 1, 0);
    this->addWidget(&linear_speed_box_, 1, 1);
    this->addWidget(&w_label_, 1, 2);
    this->addWidget(&angular_speed_box_, 1, 3);

    this->addWidget(&bt_q_, 2, 0, 1, 1);
    this->addWidget(&bt_w_, 2, 1, 1, 1);
    this->addWidget(&bt_e_, 2, 2, 1, 1);

    this->addWidget(&bt_a_, 3, 0);
    bt_s_.setStyleSheet("QPushButton {color: red;}");
    this->addWidget(&bt_s_, 3, 1);
    this->addWidget(&bt_d_, 3, 2);

    this->addWidget(&bt_z_, 4, 0);
    this->addWidget(&bt_x_, 4, 1);
    this->addWidget(&bt_c_, 4, 2);

    QTimer* output_timer = new QTimer( this );
    connect( output_timer, SIGNAL( timeout() ), this, SLOT( send_vel() ));

    connect(&discover_topic_, &QPushButton::clicked, [this](bool) { discover_topics(); });
    connect(&cmd_topic_box_,SIGNAL(currentIndexChanged(int)),SLOT(update_topic(int)));
    connect(&linear_speed_box_, SIGNAL( valueChanged(double)), this, SLOT( set_linear_speed(double)));
    connect(&angular_speed_box_, SIGNAL( valueChanged(double)), this, SLOT( set_angular_speed(double)));

    connect(&bt_q_, &QPushButton::pressed, [this](void) { set_vel('q'); });
    connect(&bt_w_, &QPushButton::pressed, [this](void) { set_vel('w'); });
    connect(&bt_e_, &QPushButton::pressed, [this](void) { set_vel('e'); });
    connect(&bt_a_, &QPushButton::pressed, [this](void) { set_vel('a'); });
    connect(&bt_s_, &QPushButton::pressed, [this](void) { set_vel('s'); });
    connect(&bt_d_, &QPushButton::pressed, [this](void) { set_vel('d'); });
    connect(&bt_z_, &QPushButton::pressed, [this](void) { set_vel('z'); });
    connect(&bt_x_, &QPushButton::pressed, [this](void) { set_vel('x'); });
    connect(&bt_c_, &QPushButton::pressed, [this](void) { set_vel('c'); });

    connect(&bt_q_, &QPushButton::released, [this](void) { set_vel(' '); });
    connect(&bt_w_, &QPushButton::released, [this](void) { set_vel(' '); });
    connect(&bt_e_, &QPushButton::released, [this](void) { set_vel(' '); });
    connect(&bt_a_, &QPushButton::released, [this](void) { set_vel(' '); });
    connect(&bt_s_, &QPushButton::released, [this](void) { set_vel(' '); });
    connect(&bt_d_, &QPushButton::released, [this](void) { set_vel(' '); });
    connect(&bt_z_, &QPushButton::released, [this](void) { set_vel(' '); });
    connect(&bt_x_, &QPushButton::released, [this](void) { set_vel(' '); });
    connect(&bt_c_, &QPushButton::released, [this](void) { set_vel(' '); });

    output_timer->start(100);
}

void TeleopButton::key_to_button(QEvent *event)
{
    if (event->type() == QEvent::KeyPress)
    {
        QKeyEvent *k = static_cast<QKeyEvent *>(event);
        switch(k->key())
        {
            case(Qt::Key_Q):
                set_vel('q');
                break;
            case(Qt::Key_W):
                set_vel('w');
                break;
            case(Qt::Key_E):
                set_vel('e');
                break;
            case(Qt::Key_A):
                set_vel('a');
                break;
            case(Qt::Key_S):
                set_vel('s');
                break;
            case(Qt::Key_D):
                set_vel('d');
                break;
            case(Qt::Key_Z):
                set_vel('z');
                break;
            case(Qt::Key_X):
                set_vel('x');
                break;
            case(Qt::Key_C):
                set_vel('c');
                break;
        }
    
    }
    else if (event->type() == QEvent::KeyRelease)
    {
        set_vel(' ');
    }

}

void TeleopButton::set_linear_speed(double value)
{
    target_linear_velocity_ = value;
    linear_velocity_ = ((linear_velocity_ > 0) - (linear_velocity_ < 0)) * target_linear_velocity_;
}

void TeleopButton::set_angular_speed(double value)
{
    target_angular_velocity_ = value;
    angular_velocity_ = ((angular_velocity_ > 0) - (angular_velocity_ < 0)) * target_angular_velocity_;
}

void TeleopButton::set_vel(const char &key)
{
    linear_velocity_ = target_linear_velocity_ * ((key == 'q') + (key == 'w') + (key == 'e') + -1*(key == 'z') + -1*(key == 'x') + -1*(key == 'c'));
    angular_velocity_ = target_angular_velocity_ * ((key == 'q') + (key == 'a') + (key == 'z') + -1*(key == 'e') + -1*(key == 'd') + -1*(key == 'c'));
}

void TeleopButton::send_vel()
{
    motion_msgs::msg::SE3VelocityCMD cmd_vel_msg;
    cmd_vel_msg.velocity.timestamp = dummy_node_->now();
    cmd_vel_msg.sourceid = 2;
    cmd_vel_msg.velocity.frameid.id = 1;
    cmd_vel_msg.velocity.linear_x = linear_velocity_;
    cmd_vel_msg.velocity.linear_y = 0;
    cmd_vel_msg.velocity.linear_z = 0;

    cmd_vel_msg.velocity.angular_x = 0;
    cmd_vel_msg.velocity.angular_y = 0;
    cmd_vel_msg.velocity.angular_z = angular_velocity_;
    if(cmd_topic_selected_)
    {
        cmd_pub_->publish(cmd_vel_msg);
    }
}

void TeleopButton::update_topic(int pos)
{
    if (pos>=0)
        setTopic(cmd_topic_list_[pos]);
}

void TeleopButton::discover_topics()
{
    std::string allowed_topic = "motion_msgs/msg/SE3VelocityCMD";

    cmd_topic_list_.clear();
    cmd_topic_list_.emplace_back(" ");

    auto topics_and_types = dummy_node_->get_topic_names_and_types();
    for (auto it : topics_and_types)
    {
        for (auto type: it.second)
        {
           if (type == allowed_topic)
           {
               cmd_topic_list_.emplace_back(QString::fromStdString(it.first));
           }
        }
    }

    cmd_topic_box_.clear();
    for (size_t pos =0; pos < cmd_topic_list_.size(); pos++)
    {
        cmd_topic_box_.addItem(cmd_topic_list_[pos]);
    }
}

void TeleopButton::setTopic( const QString& topic )
{
    if( topic == "" || topic == " " )
    {
        cmd_topic_selected_ = false;
    }
    else
    {
        try
        {
            cmd_pub_ = dummy_node_->create_publisher<motion_msgs::msg::SE3VelocityCMD>(topic.toStdString(), rclcpp::SystemDefaultsQoS());
            std::cout<<"set velocity topic to: "<< topic.toStdString()<<std::endl;
            cmd_topic_selected_ = true;
        }
        catch (...)
        {
            std::cerr<<"Unable to initialize velocity publisher, please verify topic name."<<std::endl;
        }
    }

}

}