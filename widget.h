#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QPixmap>
#include <QFileDialog>
#include <QInputDialog>

#include <iostream>

#include "ThreadRosMsg.h"
#include "LocatingPanel.h"
namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();

private:
    Ui::Widget *ui;
    std::shared_ptr<ThreadRosMsg> ptr_thread_ros_msg_;

    int map_width_;
    int map_height_;
    bool flag_setting_goal_ = false;
    bool flag_draw_arrow_ = false;
    bool flag_creating_map_ = false;
    double goal_x_ , goal_y_ , goal_yaw_;
    double dir_x_ , dir_y_;
    double map_goal_x_,map_goal_y_;

    bool flag_udpate_map = false;

    void paintEvent(QPaintEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);

    QString status_ = "standing by...";
    bool status_locating_ = false;
    LocatingPanel locating_panel_;

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

    QString using_map_;


public slots:
    void updateMap();
    void updateLaser();
    void updateXY();
    void updateYaw();

private slots:

    void on_button_manual_locate_clicked();
    void on_button_navigation_clicked();

    void on_button_create_map_clicked();
    void on_button_mapping_clicked();
};

#endif // WIDGET_H
