#include"ThreadRosMsg.h"


ThreadRosMsg::ThreadRosMsg(QObject *parent) :
    QThread(parent)
{
    ptr_nh_ = std::make_shared<ros::NodeHandle>();
    map_sub_ = ptr_nh_->subscribe("/map",1000,mapCallBack);
    laser_sub_ = ptr_nh_->subscribe("/scan",1000,laserCallBack);
    amcl_pose_sub_ = ptr_nh_->subscribe("/amcl_pose",1000,amclPoseCallBack);
    odom_sub_ = ptr_nh_->subscribe("/odom",1000,odomCallBack);
    nav_path_sub_ = ptr_nh_->subscribe("/move_base/NavfnROS/plan" , 1000 , navPathCallBack);

    inital_pose_pub_ = ptr_nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
    goal_pose_pub_ = ptr_nh_->advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal",1000);


    tf_listener_ = new tf::TransformListener ;

    flag_path_updating = false;

    static ros::AsyncSpinner spinner(1);
    spinner.start();
}

nav_msgs::OccupancyGrid ThreadRosMsg::map_data_;
bool ThreadRosMsg::flag_update_map_ = false;
bool ThreadRosMsg::flag_update_laser_ = false;

sensor_msgs::LaserScan ThreadRosMsg::laser_data_;
std::vector<QPoint> ThreadRosMsg::laser_points_;

int ThreadRosMsg::manual_locate_x_;
int ThreadRosMsg::manual_locate_y_;
int ThreadRosMsg::manual_locate_yaw_;

double ThreadRosMsg::compensate_x_ = 0;
double ThreadRosMsg::compensate_y_ = 0;
double ThreadRosMsg::compensate_yaw_ = 0;
double ThreadRosMsg::scaled_ = 1.0;

double ThreadRosMsg::transform_x_;
double ThreadRosMsg::transform_y_;
double ThreadRosMsg::transform_yaw_;

unsigned char ThreadRosMsg::work_status_ = S_STANDING_BY;
geometry_msgs::PoseWithCovarianceStamped ThreadRosMsg::amcl_pose_;
nav_msgs::Odometry ThreadRosMsg::odom_;
tf::TransformListener* ThreadRosMsg::tf_listener_;
tf::StampedTransform ThreadRosMsg::transform_s2m_;
tf::StampedTransform ThreadRosMsg::transform_o2m_;
bool ThreadRosMsg::flag_path_updating;
QVector<QLine> ThreadRosMsg::nav_path_data_;

void ThreadRosMsg::transformScan2Map()
{
    try {
        tf_listener_->waitForTransform("/laser", "/map", ros::Time(0), ros::Duration(2.0));
        tf_listener_->lookupTransform("/laser", "/map", ros::Time(0), transform_s2m_);

    } catch (...) {
        std::cout << "can not find tf laser to map... \n";
    }

}

void ThreadRosMsg::run()
{


    while(true)
    {
        if(flag_update_map_)
        {
            flag_update_map_ = false;
            emit MapUpdateSignal();
        }

        if(flag_update_laser_)
        {
            flag_update_map_ = false;
            emit LaserUpdateSignal();
        }
    }
}

void ThreadRosMsg::odomCallBack(const nav_msgs::Odometry& msg)
{
    odom_ = std::move(msg);
}


void ThreadRosMsg::mapCallBack(const nav_msgs::OccupancyGrid& msg)
{
    std::cout << msg.data.size() << std::endl;
    std::cout << msg.info.width << "x" << msg.info.height << std::endl;
    int cnt = 0;

    flag_update_map_ = true;
    map_data_ = std::move(msg);
    scaled_ = 1.0 / msg.info.resolution;

    std::cout << "scaled_: " << scaled_ << std::endl;
}

void ThreadRosMsg::laserCallBack(const sensor_msgs::LaserScan& msg)
{
//    std::cout << msg.ranges.size() << std::endl;
//    std::cout << (msg.angle_max - msg.angle_min)/msg.angle_increment << std::endl;
//    std::cout << msg.angle_min << " ~ " << msg.angle_max << std::endl;
//    std::cout << msg.angle_increment << std::endl << std::endl;


    laser_points_.clear();
    int index = 0;
    for (double distance:msg.ranges)
    {
          double angle;
          double col;
          double row;
          if(work_status_ == S_STANDING_BY)
          {
//              tf::Quaternion quat;
//              tf::quaternionMsgToTF(amcl_pose_.pose.pose.orientation, quat);
//              double yaw = tf::getYaw(quat);

//              angle = msg.angle_min + msg.angle_increment * index +yaw;
//              angle += tf::getYaw(odom_.pose.pose.orientation) - transform_yaw_;
//              double tr_x = odom_.pose.pose.position.x - transform_x_;
//              double tr_y = odom_.pose.pose.position.y - transform_y_;


//              col = scaled_ * (distance * cos(angle) + amcl_pose_.pose.pose.position.x + tr_x - map_data_.info.origin.position.x);
//              row = scaled_ * (distance * sin(angle) + amcl_pose_.pose.pose.position.y + tr_y - map_data_.info.origin.position.y);
//              std::cout << amcl_pose_.pose.pose.position.y << " | " <<  map_data_.info.origin.position.y << std::endl;

              double angle = msg.angle_min + msg.angle_increment * index;
              transformScan2Map();

              double result_yaw = tf::getYaw(transform_s2m_.getRotation());


              angle -= result_yaw;
              double x = distance  * cos(angle);
              double y = distance  * sin(angle);



//              col = scaled_ * (x - transform_s2m_.getOrigin().x());
//              row = scaled_ * (y - transform_s2m_.getOrigin().y());

              col = scaled_ * amcl_pose_.pose.pose.position.x + scaled_ * x;
              row = scaled_ * amcl_pose_.pose.pose.position.y + scaled_ * y;
              col -= map_data_.info.origin.position.x * scaled_;
              row -= map_data_.info.origin.position.y * scaled_;
          }
          if(work_status_ == S_MANUAL_LOCATING)
          {
              angle = msg.angle_min + msg.angle_increment * index + compensate_yaw_;
              col = scaled_ * distance * cos(angle) + manual_locate_x_ + compensate_x_;
              row = scaled_ * distance * sin(angle) + manual_locate_y_ + compensate_y_;
          }

          if(work_status_ == S_CREATING_MAP)
          {
              double angle = msg.angle_min + msg.angle_increment * index;
              transformScan2Map();


              double result_yaw = tf::getYaw(transform_s2m_.getRotation());


              angle -= result_yaw;
              double x = distance  * cos(angle);
              double y = distance  * sin(angle);

              col = (odom_.pose.pose.position.x   )* scaled_;
              row = (odom_.pose.pose.position.y  )* scaled_;


              col += x*scaled_;
              row += y*scaled_;


              col -= map_data_.info.origin.position.x * scaled_;
              row -= map_data_.info.origin.position.y * scaled_;


          }

          laser_points_.push_back(QPoint(col,row));
          ++index;
    }

    flag_update_laser_ = true;
    laser_data_ = std::move(msg);


}

void ThreadRosMsg::navPathCallBack(const nav_msgs::Path& msg)
{
   flag_path_updating = true;

   nav_path_data_.clear();
   for(int i = 1 ; i < msg.poses.size() ; i ++)
   {
        double x1,y1,x2,y2;
        x1 = msg.poses.at(i - 1).pose.position.x;
        y1 = msg.poses.at(i - 1).pose.position.y;
        x2 = msg.poses.at(i).pose.position.x;
        y2 = msg.poses.at(i).pose.position.y;
        x1 = x1 * scaled_ - map_data_.info.origin.position.x * scaled_;
        y1 = y1 * scaled_ - map_data_.info.origin.position.y * scaled_;
        x2 = x2 * scaled_ - map_data_.info.origin.position.x * scaled_;
        y2 = y2 * scaled_ - map_data_.info.origin.position.y * scaled_;
        y1 = map_data_.info.height - y1;
        y2 = map_data_.info.height - y2;

        nav_path_data_.push_back(QLine(x1,y1,x2,y2));
   }

   flag_path_updating = false;
}

void ThreadRosMsg::publishInitPosed()
{

    init_posed_.header.stamp = ros::Time::now();
    init_posed_.header.frame_id = "map";
    //要先设置header再去设置pose

    init_posed_.pose.pose.position.x = (manual_locate_x_ + compensate_x_) / scaled_ + map_data_.info.origin.position.x;
    init_posed_.pose.pose.position.y = -((manual_locate_y_ - compensate_y_) - map_data_.info.height)/ scaled_ + map_data_.info.origin.position.y;


//    init_posed_.pose.pose.position.x -= (0.484* cos(compensate_yaw_))/scaled_;
//    init_posed_.pose.pose.position.y -= (0.484* sin(compensate_yaw_))/scaled_;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(compensate_yaw_);
    init_posed_.pose.pose.orientation = quat;
    inital_pose_pub_.publish(init_posed_);

}

void ThreadRosMsg::amclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    amcl_pose_ = std::move(msg);
}
