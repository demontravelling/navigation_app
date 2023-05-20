#include "LocatingPanel.h"
#include "ui_LocatingPanel.h"

LocatingPanel::LocatingPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LocatingPanel)
{
    ui->setupUi(this);
    ui->dial_yaw->setRange(-180,180);
    ui->dial_yaw->setValue(0);

}

LocatingPanel::~LocatingPanel()
{
    delete ui;
}



void LocatingPanel::on_button_up_clicked()
{
    ++compensate_y_;
    emit adjustXYSignal();
}

void LocatingPanel::on_button_down_clicked()
{
    --compensate_y_;
    emit adjustXYSignal();
}

void LocatingPanel::on_button_left_clicked()
{
    --compensate_x_;
    emit adjustXYSignal();
}

void LocatingPanel::on_button_right_clicked()
{
    ++compensate_x_;
    emit adjustXYSignal();
}

void LocatingPanel::on_dial_yaw_valueChanged(int value)
{
    compensate_yaw_ = value;
    emit adjustYawSignal();
}

void LocatingPanel::on_button_add_yaw_clicked()
{
    int val = ui->dial_yaw->value();
    ++ val;
    ui->dial_yaw->setValue(val);
}



void LocatingPanel::on_button_sub_yaw_clicked()
{
    int val = ui->dial_yaw->value();
    -- val;
    ui->dial_yaw->setValue(val);
}
