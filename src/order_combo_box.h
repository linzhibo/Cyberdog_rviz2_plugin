#ifndef ORDER_COMBO_BOX_H
#define ORDER_COMBO_BOX_H

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSizePolicy>

#include "motion_msgs/msg/mon_order.hpp"

class OrderComboBox : public QHBoxLayout
{
Q_OBJECT

public:
    OrderComboBox(QWidget *parent = nullptr);
    ~OrderComboBox();

Q_SIGNALS:
    void valueChanged(int newvalue);
    bool clicked();

private Q_SLOTS:
    void reemit_signal(int index);

private:
    QComboBox* order_list_;
    QPushButton* order_button_;
};

#endif