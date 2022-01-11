#ifndef GAIT_COMBO_BOX_H
#define GAIT_COMBO_BOX_H

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>

#include "motion_msgs/msg/gait.hpp"

class GaitComboBox : public QHBoxLayout
{
Q_OBJECT

public:
    GaitComboBox(QWidget *parent = nullptr);
    ~GaitComboBox();

Q_SIGNALS:
    void valueChanged(int newvalue);

private Q_SLOTS:
    void reemit_signal(int index);

private:
    QComboBox* gait_list_;
};

#endif