#include "gait_combo_box.h"

GaitComboBox::GaitComboBox(QWidget*): QHBoxLayout()
{
    gait_list_ = new QComboBox();
    gait_list_->addItem(" ");
    gait_list_->addItem("GAIT_STAND_R");
    gait_list_->addItem("GAIT_WALK");
    gait_list_->addItem("GAIT_SLOW_TROT");
    gait_list_->addItem("GAIT_TROT");
    gait_list_->addItem("GAIT_FLYTROT");
    gait_list_->addItem("GAIT_BOUND");
    gait_list_->addItem("GAIT_PRONK");

    QLabel* gait_label = new QLabel("🐾  Gait: ");
    QSizePolicy spLeft(QSizePolicy::Preferred, QSizePolicy::Preferred);
    spLeft.setHorizontalStretch(1);
    gait_label->setSizePolicy(spLeft);
    this->addWidget(gait_label);

    QSizePolicy spRight(QSizePolicy::Preferred, QSizePolicy::Preferred);
    spRight.setHorizontalStretch(1);
    gait_list_->setSizePolicy(spRight);

    this->addWidget(gait_list_);

    connect(gait_list_ ,SIGNAL(currentIndexChanged(int)), SLOT(reemit_signal(int)));
}

GaitComboBox::~GaitComboBox(){}

void GaitComboBox::reemit_signal(int newvalue)
{
    int gait_number = 0;
    switch(newvalue)
    {
        case(1):
            gait_number = motion_msgs::msg::Gait::GAIT_STAND_R;
            break;
        case(2):
            gait_number = motion_msgs::msg::Gait::GAIT_WALK;
            break;
        case(3):
            gait_number = motion_msgs::msg::Gait::GAIT_SLOW_TROT;
            break;
        case(4):
            gait_number = motion_msgs::msg::Gait::GAIT_TROT;
            break;
        case(5):
            gait_number = motion_msgs::msg::Gait::GAIT_FLYTROT;
            break;
        case(6):
            gait_number = motion_msgs::msg::Gait::GAIT_BOUND;
            break;
        case(7):
            gait_number = motion_msgs::msg::Gait::GAIT_PRONK;
            break;
    }

    Q_EMIT valueChanged(gait_number);
}
