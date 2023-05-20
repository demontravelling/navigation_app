#ifndef THREADROSMSG_H
#define THREADROSMSG_H
//Qt header:
#include <QThread>
#include <QPoint>
#include <QLine>
#include <QVector>

//Ros related header:
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include "tf/transform_listener.h"

#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#define S_STANDING_BY     0
#define S_MANUAL_LOCATING 1
#define S_CREATING_MAP 2
#define PI 3.1415926
//信号是QT对象之间通信的机制，因此必须加上Q_OBJECT说明该线程类是QT对象
class ThreadRosMsg : public QThread
{
 Q_OBJECT

public:
    explicit ThreadRosMsg(QObject *parent = nullptr);
signals:
    void MapUpdateSignal();
    void LaserUpdateSignal();
public :
//members:
    ros::Subscriber map_sub_;
    ros::Subscriber laser_sub_;
    ros::Subscriber amcl_pose_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher inital_pose_pub_;
    ros::Publisher goal_pose_pub_;
    ros::Subscriber nav_path_sub_;

    std::shared_ptr<ros::NodeHandle> ptr_nh_;
    static nav_msgs::OccupancyGrid map_data_;
    static sensor_msgs::LaserScan laser_data_;
    static std::vector<QPoint> laser_points_;
    static geometry_msgs::PoseWithCovarianceStamped amcl_pose_;
    static QVector<QLine> nav_path_data_;

    static bool flag_update_map_;
    static bool flag_update_laser_;
    bool flag_draw_finished = true;
    static bool flag_path_updating;

    static int manual_locate_x_;
    static int manual_locate_y_;
    static int manual_locate_yaw_;

    static double compensate_x_;
    static double compensate_y_;
    static double compensate_yaw_;

    static nav_msgs::Odometry odom_;
    static double transform_x_;
    static double transform_y_;
    static double transform_yaw_;


    static tf::TransformListener* tf_listener_;
    static tf::StampedTransform transform_s2m_;
    static tf::StampedTransform transform_o2m_;

    geometry_msgs::PoseWithCovarianceStamped init_posed_;

    static double scaled_;
    static unsigned char work_status_;

//member function:
//ros1中，回调函数作为c++类成员必须是静态函数，所以所有回调函数必须加上static关键字
//静态的回调函数只能访问类中的静态成员，不能访问非静态成员
    void run();
    static void mapCallBack(const nav_msgs::OccupancyGrid& msg);
    static void laserCallBack(const sensor_msgs::LaserScan& msg);
    static void amclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped& msg);
    static void odomCallBack(const nav_msgs::Odometry& msg);
    static void transformScan2Map();
    static void navPathCallBack(const nav_msgs::Path& msg);
    void publishInitPosed();

};

#endif // THREADROSMSG_H
