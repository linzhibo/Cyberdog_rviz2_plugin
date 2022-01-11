#include "order_combo_box.h"

OrderComboBox::OrderComboBox(QWidget*): QHBoxLayout()
{
    order_list_ = new QComboBox();
    order_list_->addItem(" ");
    order_list_->addItem("STAND_UP");
    order_list_->addItem("PROSTRATE");
    order_list_->addItem("TURN_AROUND");
    order_list_->addItem("HI_FIVE");
    order_list_->addItem("DANCE");
    order_list_->addItem("WELCOME");
    order_list_->addItem("TURN_OVER");
    order_list_->addItem("SIT");
    order_list_->addItem("SHOW");

    QLabel* order_label = new QLabel("Order: ");
    order_label->setFixedWidth(40);
    this->addWidget(order_label, Qt::AlignLeft);

    order_list_->setFixedWidth(80);
    this->addWidget(order_list_);

    connect(order_list_ ,SIGNAL(currentIndexChanged(int)), SLOT(reemit_signal(int)));
}

OrderComboBox::~OrderComboBox(){}

void OrderComboBox::reemit_signal(int newvalue)
{
    int order_number = 0;
    switch(newvalue)
    {
        case(1):
            order_number = motion_msgs::msg::MonOrder::MONO_ORDER_STAND_UP;
            break;
        case(2):
            order_number = motion_msgs::msg::MonOrder::MONO_ORDER_PROSTRATE;
            break;
        case(3):
            order_number = motion_msgs::msg::MonOrder::MONO_ORDER_STEP_BACK;
            break;
        case(4):
            order_number = motion_msgs::msg::MonOrder::MONO_ORDER_TURN_AROUND;
            break;
        case(5):
            order_number = motion_msgs::msg::MonOrder::MONO_ORDER_HI_FIVE;
            break;
        case(6):
            order_number = motion_msgs::msg::MonOrder::MONO_ORDER_DANCE;
            break;
        case(7):
            order_number = motion_msgs::msg::MonOrder::MONO_ORDER_WELCOME;
            break;
        case(8):
            order_number = motion_msgs::msg::MonOrder::MONO_ORDER_TURN_OVER;
            break;
        case(9):
            order_number = motion_msgs::msg::MonOrder::MONO_ORDER_SIT;
            break;
        case(10):
            order_number = motion_msgs::msg::MonOrder::MONO_ORDER_SHOW;
            break;
    }

    Q_EMIT valueChanged(order_number);
}
