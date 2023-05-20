#include "widget.h"
#include "ui_widget.h"

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    ptr_thread_ros_msg_ = std::make_shared<ThreadRosMsg>();
    ptr_thread_ros_msg_->start();
    connect(ptr_thread_ros_msg_.get(),SIGNAL(MapUpdateSignal()),
            this,SLOT(updateMap()));
    connect(ptr_thread_ros_msg_.get(),SIGNAL(LaserUpdateSignal()),
            this,SLOT(updateLaser()));
    connect(&locating_panel_,SIGNAL(adjustXYSignal()),
            this,SLOT(updateXY()));
    connect(&locating_panel_,SIGNAL(adjustYawSignal()),
            this,SLOT(updateYaw()));
}

Widget::~Widget()
{
    delete ui;
}

void Widget::paintEvent(QPaintEvent *event)
{

    ptr_thread_ros_msg_->flag_draw_finished = false;

    QPainter painter(this);
    QPen pen_goal;
    painter.setRenderHints(QPainter::Antialiasing|QPainter::SmoothPixmapTransform);

    QPen pen_background;
    QPen pen_obstacle;
    QPen pen_road;
    pen_background.setWidth(2);
    pen_obstacle.setWidth(2);
    pen_road.setWidth(2);
    pen_background.setColor(Qt::darkGray);
    pen_obstacle.setColor(Qt::black);
    pen_road.setColor(Qt::white);

    map_width_ = ptr_thread_ros_msg_->map_data_.info.width;
    map_height_ = ptr_thread_ros_msg_->map_data_.info.height;



    if(flag_creating_map_ == true)
    {
        long long index = 0;
        for (int i:ptr_thread_ros_msg_->map_data_.data)
        {

            int pix = i;
            if(pix == -1)
            {
                painter.setPen(pen_background);
            }
            else if(pix == 100)
            {
                painter.setPen(pen_obstacle);
            }
            else if(pix == 0)
            {
                painter.setPen(pen_road);
            }

            painter.drawPoint(QPoint(index%map_width_ , -(index / map_width_ - map_height_)));
            ++index;
        }
    }
    else
    {
        QPixmap pixmap(using_map_);
        map_width_ = pixmap.width();
        map_height_ = pixmap.height();
        painter.drawPixmap(0,0,map_width_,map_height_,pixmap);
    }

    QPen pen_laser;
    pen_laser.setColor(Qt::red);
    pen_laser.setWidth(2);
    painter.setPen(pen_laser);

    //painter.translate(100,100);

    for (QPoint point:ptr_thread_ros_msg_->laser_points_)
    {
        if(abs(point.x()) > 100000)
        {
            continue;
        }

        if(ptr_thread_ros_msg_->work_status_ == S_MANUAL_LOCATING)
        {
            auto target_x = point.x();
            auto target_y = 2*ptr_thread_ros_msg_->manual_locate_y_ - point.y();
            painter.drawPoint(target_x,target_y);
        }
        if(ptr_thread_ros_msg_->work_status_ == S_STANDING_BY || ptr_thread_ros_msg_->work_status_ == S_CREATING_MAP)
        {
            auto target_x = point.x();
            auto target_y = ptr_thread_ros_msg_->map_data_.info.height - point.y();
            painter.drawPoint(target_x,target_y);
        }

    }


    flag_udpate_map = false;

    painter.setRenderHints(QPainter::Antialiasing|QPainter::SmoothPixmapTransform);
    if(ptr_thread_ros_msg_->flag_path_updating == false)
    {
        pen_goal.setColor(Qt::darkCyan);
        pen_goal.setWidth(1);
        painter.setPen(pen_goal);
        ui->label_mouse->setText(QString::number(ptr_thread_ros_msg_->nav_path_data_.size(),10));
        painter.drawLines(ptr_thread_ros_msg_->nav_path_data_);

    }

    QPixmap pix_locate("loc.png");
    if(status_locating_ == true)
    {
        double angle =ptr_thread_ros_msg_->compensate_yaw_;
        painter.translate((ptr_thread_ros_msg_->manual_locate_x_ + ptr_thread_ros_msg_->compensate_x_), (ptr_thread_ros_msg_->manual_locate_y_ - ptr_thread_ros_msg_->compensate_y_));
        painter.rotate(-angle/PI*180);
        painter.drawPixmap( - pix_locate.width() / 2 ,  - pix_locate.height() / 2 , pix_locate.width(), pix_locate.height(),pix_locate);
        painter.rotate(angle/PI*180);
        painter.translate(-(ptr_thread_ros_msg_->manual_locate_x_ + ptr_thread_ros_msg_->compensate_x_), -(ptr_thread_ros_msg_->manual_locate_y_ - ptr_thread_ros_msg_->compensate_y_));
    }
    else if(ptr_thread_ros_msg_->work_status_ == S_STANDING_BY)
    {
        double angle = tf::getYaw(ptr_thread_ros_msg_->amcl_pose_.pose.pose.orientation);

        double col = ptr_thread_ros_msg_->scaled_ * ptr_thread_ros_msg_->amcl_pose_.pose.pose.position.x;
        double row = ptr_thread_ros_msg_->scaled_ * ptr_thread_ros_msg_->amcl_pose_.pose.pose.position.y;
        col -= ptr_thread_ros_msg_->map_data_.info.origin.position.x * ptr_thread_ros_msg_->scaled_;
        row -= ptr_thread_ros_msg_->map_data_.info.origin.position.y * ptr_thread_ros_msg_->scaled_;
        painter.translate(col,ptr_thread_ros_msg_->map_data_.info.height - row);
        painter.rotate(-angle/PI*180);
        painter.drawPixmap( - pix_locate.width() / 2 ,  - pix_locate.height() / 2 , pix_locate.width(), pix_locate.height(),pix_locate);
        painter.rotate(angle/PI*180);
        painter.translate(-col,-(ptr_thread_ros_msg_->map_data_.info.height - row));
    }
    else if(ptr_thread_ros_msg_->work_status_ == S_CREATING_MAP)
    {

    }


    if(flag_draw_arrow_ == true)
    {
        pen_goal.setColor(Qt::red);
        pen_goal.setWidth(3);
        painter.setPen(pen_goal);

        qreal arrowSize = 30;
         QLineF line(goal_x_,goal_y_,dir_x_,dir_y_);
        double angle_0 = std::atan2(-line.dy(), line.dx());
        QPointF arrowP1 = line.p2() - QPointF(sin(angle_0 + M_PI/2.15)*arrowSize,cos(angle_0+M_PI/2.15)*arrowSize);
        QPointF arrowP2 = line.p2() - QPointF(sin(angle_0 + M_PI - M_PI/2.15)*arrowSize,cos(angle_0+M_PI -M_PI/2.15)*arrowSize);
        QPolygonF arrowHead;
        arrowHead.clear();
        arrowHead<<line.p2()<<arrowP1<<arrowP2;
        painter.drawLine(line);
        painter.drawPolygon(arrowHead);

    }

    ptr_thread_ros_msg_->flag_draw_finished = true;
}

void Widget::updateMap()
{
    std::cout << "updata Map" << std::endl;
    flag_udpate_map = true;
    if(ptr_thread_ros_msg_->flag_draw_finished == true)
        update();
}

void Widget::updateLaser()
{
    if(flag_udpate_map == true)
        return ;
    flag_udpate_map = true;
    if(ptr_thread_ros_msg_->flag_draw_finished == true)
        update();
}




void Widget::on_button_manual_locate_clicked()
{
    if(status_locating_ == false)
    {
        ptr_thread_ros_msg_->work_status_ = S_MANUAL_LOCATING;
        status_ = "manual locating...";
        status_locating_ = true;
        ui->button_manual_locate->setText("结束定位");
        this->locating_panel_.show();

        ptr_thread_ros_msg_->compensate_x_ = 0;
        ptr_thread_ros_msg_->compensate_y_ = 0;
        ptr_thread_ros_msg_->compensate_yaw_ = 0;
        locating_panel_.compensate_x_ = 0;
        locating_panel_.compensate_y_ = 0;
        locating_panel_.compensate_yaw_ = 0;

    }
    else
    {
        ptr_thread_ros_msg_->work_status_ = S_STANDING_BY;
        status_ = "standing by...";
        status_locating_ = false;
        ui->button_manual_locate->setText("手动定位");
        this->locating_panel_.close();

        ptr_thread_ros_msg_->publishInitPosed();
    }

}

void Widget::updateXY()
{
    ptr_thread_ros_msg_->compensate_x_ = locating_panel_.compensate_x_;
    ptr_thread_ros_msg_->compensate_y_ = locating_panel_.compensate_y_;
}

void Widget::updateYaw()
{
    ptr_thread_ros_msg_->compensate_yaw_ = locating_panel_.compensate_yaw_*3.1415926 / 180.0;
}

void Widget::on_button_navigation_clicked()
{
    setCursor(Qt::CrossCursor);
    flag_setting_goal_ = true;
}

void Widget::mousePressEvent(QMouseEvent *event)
{
    if(flag_setting_goal_ == true)
    {
        dir_x_ = goal_x_ = event->x();
        dir_y_ = goal_y_ = event->y();

        map_goal_x_ = goal_x_ / ptr_thread_ros_msg_->scaled_ + ptr_thread_ros_msg_->map_data_.info.origin.position.x;

        map_goal_y_ = (ptr_thread_ros_msg_->map_data_.info.height - goal_y_) / ptr_thread_ros_msg_->scaled_ + ptr_thread_ros_msg_->map_data_.info.origin.position.y;

        auto tmp = QString::number(map_goal_x_,'c',2) + QString(" ") + QString::number(map_goal_y_,'c',2);
        ui->label_mouse->setText(tmp);
    }
}

void Widget::mouseMoveEvent(QMouseEvent *event)
{
    if(flag_setting_goal_ == true)
    {
        dir_x_  = event->x();
        dir_y_  = event->y();
        auto tmp = QString::number(dir_x_,'c',2) + QString(" ") + QString::number(dir_y_,'c',2);
        ui->label_mouse->setText(tmp);
        flag_draw_arrow_ = true;
    }
}

void Widget::mouseReleaseEvent(QMouseEvent *event)
{
    std::cout << event->x() << " , " << event->y() << std::endl;

    if(status_locating_ == true)
    {
        ptr_thread_ros_msg_->manual_locate_x_ = event->x();
        ptr_thread_ros_msg_->manual_locate_y_ = event->y();
        ptr_thread_ros_msg_->compensate_x_ = 0;
        ptr_thread_ros_msg_->compensate_y_ = 0;
        locating_panel_.compensate_x_ = 0;
        locating_panel_.compensate_y_ = 0;
    }

    if(flag_setting_goal_ == true)
    {
        flag_setting_goal_ = false;
        flag_draw_arrow_ = false;
        setCursor(Qt::ArrowCursor);

        goal_yaw_ = atan((event->y() - goal_y_) / (event->x() - goal_x_));

        move_base_msgs::MoveBaseActionGoal msg_goal;
        msg_goal.goal.target_pose.header.frame_id = "map";
        msg_goal.header.stamp = ros::Time::now();
        msg_goal.goal.target_pose.pose.position.x = map_goal_x_;
        msg_goal.goal.target_pose.pose.position.y = map_goal_y_;

        msg_goal.goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw_);
        ptr_thread_ros_msg_->goal_pose_pub_.publish(msg_goal);
        ptr_thread_ros_msg_->nav_path_data_.clear();
    }
}


void Widget::on_button_create_map_clicked()
{
    if(flag_creating_map_ == false)
    {
        flag_creating_map_ = true;
        ui->button_create_map->setText("结束建图");
        ptr_thread_ros_msg_->work_status_ = S_CREATING_MAP;
        ptr_thread_ros_msg_->transform_x_ = ptr_thread_ros_msg_->odom_.pose.pose.position.x;
        ptr_thread_ros_msg_->transform_y_ = ptr_thread_ros_msg_->odom_.pose.pose.position.y;
    }
    else
    {
        flag_creating_map_ = false;
        ui->button_create_map->setText("建图");

        QString file_name =  QFileDialog::getSaveFileName(NULL, "选择保存路径", "/home/byd/" , ".pgm");

        if(file_name != "")
        {
            QString instruct = QString("rosrun map_server map_saver -f ");
            instruct += file_name;
            system(instruct.toStdString().c_str());
            QString yaml_name = file_name + QString(".yaml");
            file_name += QString(".pgm");

            using_map_ = file_name;
            instruct = QString("rosrun map_server map_server ");
            instruct += yaml_name;
            system(instruct.toStdString().c_str());
        }
        ptr_thread_ros_msg_->work_status_ = S_STANDING_BY;
    }
}

void Widget::on_button_mapping_clicked()
{
    //QString file_name =  QFileDialog::getOpenFileName(0, "pgm", "/home/byd/" , "*.pgm");
    QString file_name =  QInputDialog::getText(0, "pgm", "/home/byd/");
    using_map_ = file_name;
//  using_map_ = QString("/home/byd/byd_ws/src/clean_ct06a/map/compelete.pgm");
    update();
}
