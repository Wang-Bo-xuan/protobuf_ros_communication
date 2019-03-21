#include "client/client.datapackage.h"

ClientDataPackage::ClientDataPackage() : data_package_thread_(NULL)
{
    if (!nh_.getParam("/client_node/bufferPackageSize", bufferPackageSize_))
        bufferPackageSize_ = 3;
    buffer_package_size_ = bufferPackageSize_ * 1024 * 1024;
    if(!nh_.getParam("/client_node/mode", mode_)){
        ROS_ERROR("datapackage can not get mode!");
        return;
    }
}

ClientDataPackage::~ClientDataPackage()
{
    if (data_package_thread_)
    {
        data_package_thread_->join();
        delete data_package_thread_;
    }
}

void ClientDataPackage::start(DataShare *datashare)
{
    ds_ = datashare;
    data_package_thread_ = new boost::thread([&] { runThread(); });
}

void ClientDataPackage::runThread()
{

    cout << "enter client data package!" << endl;
    ros::Subscriber lidar_sub = nh_.subscribe("/scan", 10, &ClientDataPackage::lidarCallback, this);
    ros::Subscriber map_sub = nh_.subscribe("/map", 1, &ClientDataPackage::mapCallback, this);
    ros::Subscriber path_sub = nh_.subscribe("/move_base/NavfnROS/plan", 10, &ClientDataPackage::pathCallback, this);
    ros::Subscriber tf_sub = nh_.subscribe("/tf", 10, &ClientDataPackage::TFCallback, this);
    ros::Subscriber tf_static_sub = nh_.subscribe("/tf_static", 10, &ClientDataPackage::TFStaticCallback, this);
    ros::Subscriber odo_sub = nh_.subscribe("/odom", 10, &ClientDataPackage::OdometryCallback, this);

    ros::spin();
}

void ClientDataPackage::pack(const string &name, const string &content)
{
    SensorData buff;
    buff.name = name;
    buff.content = content;
    buff.total_len.resize(8);
    buff.name_len = to_string(buff.name.size());
    buff.total_len = to_string(buff.start.size() + buff.name_len.size() + buff.total_len.size() + buff.name.size() + buff.content.size() + buff.terminator.size());
    buff.total_len.resize(8);
    buff.all = buff.start + buff.name_len + buff.total_len + buff.name + buff.content + buff.terminator;

    // if(ds_->client_data_package_.size() > buffer_package_size_) return;
    if (ds_->client_data_package_.size() > 1024)
        return;
    ds_->client_data_package_mtx_.lock();
    ds_->client_data_package_.append(buff.all);
    ds_->client_data_package_mtx_.unlock();

    cout << endl
         << "------------------------" << endl;
    cout << buff.start << endl;
    cout << buff.name << endl;
    cout << buff.total_len << " " << buff.all.size() << endl;
    cout << buff.terminator << endl;
    cout << "client_data_package_ size : " << ds_->client_data_package_.size() << endl;
    cout << "------------------------" << endl
         << endl;
}

void ClientDataPackage::lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    if (ds_->client_control_command_.compare("start"))
        return;
    string name = "proto_msg.LaserScan";
    string content;
    proto_msg::LaserScan proto_Lidar;

    proto_Lidar.set_protocol_type(name);
    proto_Lidar.set_publish_stamp(scan->header.stamp.toSec());
    proto_Lidar.set_frame_id(scan->header.frame_id);
    proto_Lidar.set_angle_min(scan->angle_min);
    proto_Lidar.set_angle_max(scan->angle_max);
    proto_Lidar.set_angle_increment(scan->angle_increment);
    proto_Lidar.set_time_increment(scan->time_increment);
    proto_Lidar.set_scan_time(scan->scan_time);
    proto_Lidar.set_range_min(scan->range_min);
    proto_Lidar.set_range_max(scan->range_max);
    for (double x : scan->ranges)
        proto_Lidar.add_ranges(x);
    for (double x : scan->intensities)
        proto_Lidar.add_intensitys(x);
    proto_Lidar.SerializeToString(&content);
    proto_Lidar.Clear();

    pack(name, content);
}

void ClientDataPackage::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if (ds_->client_control_command_.compare("start"))
        return;
    string name = "proto_msg.OccupancyGrid";
    string content;
    proto_msg::OccupancyGrid proto;

    proto.set_protocol_type(name);
    proto.set_publish_stamp(msg->header.stamp.toSec());
    proto.set_frame_id(msg->header.frame_id);
    proto.set_load_time(msg->info.map_load_time.toSec());
    proto.set_resolution(msg->info.resolution);
    proto.set_width(msg->info.width);
    proto.set_height(msg->info.height);
    proto.set_x(msg->info.origin.position.x);
    proto.set_y(msg->info.origin.position.y);
    proto.set_z(msg->info.origin.position.z);
    proto.set_q_x(msg->info.origin.orientation.x);
    proto.set_q_y(msg->info.origin.orientation.y);
    proto.set_q_z(msg->info.origin.orientation.z);
    proto.set_q_w(msg->info.origin.orientation.w);
    std::string m;
    for (int i = 0; i < msg->data.size(); i++)
        m.push_back(msg->data.at(i));
    proto.set_map_data(m);
    bool re = proto.SerializeToString(&content);
    proto.Clear();
    if (!re)
    {
        cout << "fail!!!!!" << endl;
        return;
    }

    pack(name, content);

    if(mode_ == "nav") {
        ds_->client_backup_map_mtx_.lock();
        ds_->client_backup_map_ = content;
        ds_->client_backup_map_mtx_.unlock();
        ROS_INFO("ds_->client_backup_map_ size = %d", static_cast<int>(ds_->client_backup_map_.size()));
    }
}

void ClientDataPackage::pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    if (ds_->client_control_command_.compare("start"))
        return;
    string name = "proto_msg.Path";
    string content;
    proto_msg::Path proto;

    proto.set_protocol_type(name);
    proto.set_publish_stamp(msg->header.stamp.toSec());
    proto.set_frame_id(msg->header.frame_id);
    proto_msg::PathPoint *pp;
    for (auto pose : msg->poses)
    {
        pp = proto.add_poses();
        pp->set_publish_stamp(pose.header.stamp.toSec());
        pp->set_frame_id(pose.header.frame_id);
        pp->set_x(pose.pose.position.x);
        pp->set_y(pose.pose.position.y);
        pp->set_z(pose.pose.position.z);
        double roll, pitch, yaw;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(pose.pose.orientation, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        pp->set_roll(roll);
        pp->set_pitch(pitch);
        pp->set_yaw(yaw);
    }

    bool re = proto.SerializeToString(&content);
    if (!re)
    {
        cout << "fail!!!!!" << endl;
        return;
    }
    proto.Clear();

    pack(name, content);
}

void ClientDataPackage::TFCallback(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    if (ds_->client_control_command_.compare("start"))
        return;
    string name = "proto_msg.TFMessage";
    string content;
    proto_msg::TFMessage proto;

    proto.set_protocol_type(name);
    proto_msg::TF *tf;
    for (auto transform : msg->transforms)
    {
        tf = proto.add_tfs();
        tf->set_publish_stamp(transform.header.stamp.toSec());
        tf->set_frame_id(transform.header.frame_id);
        tf->set_child_frame_id(transform.child_frame_id);
        tf->set_x(transform.transform.translation.x);
        tf->set_y(transform.transform.translation.y);
        tf->set_z(transform.transform.translation.z);
        double roll, pitch, yaw;
        tf::Quaternion qua;
        tf::quaternionMsgToTF(transform.transform.rotation, qua);
        tf::Matrix3x3(qua).getRPY(roll, pitch, yaw);
        tf->set_roll(roll);
        tf->set_pitch(pitch);
        tf->set_yaw(yaw);
    }

    if (!proto.SerializeToString(&content))
    {
        cout << "fail!!!!!" << endl;
        return;
    }
    proto.Clear();

    pack(name, content);
}

void ClientDataPackage::TFStaticCallback(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    if (ds_->client_control_command_.compare("start"))
        return;
    string name = "proto_msg.TFStaticMessage";
    string content;
    proto_msg::TFStaticMessage proto;

    proto.set_protocol_type(name);
    proto_msg::TF *tf;
    for (auto transform : msg->transforms)
    {
        tf = proto.add_tfs();
        tf->set_publish_stamp(transform.header.stamp.toSec());
        tf->set_frame_id(transform.header.frame_id);
        tf->set_child_frame_id(transform.child_frame_id);
        tf->set_x(transform.transform.translation.x);
        tf->set_y(transform.transform.translation.y);
        tf->set_z(transform.transform.translation.z);
        double roll, pitch, yaw;
        tf::Quaternion qua;
        tf::quaternionMsgToTF(transform.transform.rotation, qua);
        tf::Matrix3x3(qua).getRPY(roll, pitch, yaw);
        tf->set_roll(roll);
        tf->set_pitch(pitch);
        tf->set_yaw(yaw);
    }

    if (!proto.SerializeToString(&content))
    {
        cout << "fail!!!!!" << endl;
        return;
    }
    proto.Clear();

    pack(name, content);

    ds_->client_backup_tf_static_mtx_.lock();
    ds_->client_backup_tf_static_ = content;
    ds_->client_backup_tf_static_mtx_.unlock();
    ROS_INFO("ds_->client_backup_tf_static_ size = %d", static_cast<int>(ds_->client_backup_tf_static_.size()));

}

void ClientDataPackage::OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (ds_->client_control_command_.compare("start"))
        return;
    string name = "proto_msg.Odometry";
    string content;
    proto_msg::Odometry proto;

    proto.set_protocol_type(name);
    proto.set_publish_stamp(msg->header.stamp.toSec());
    proto.set_frame_id(msg->header.frame_id);
    proto.set_child_frame_id(msg->child_frame_id);
    proto.set_pose_x(msg->pose.pose.position.x);
    proto.set_pose_y(msg->pose.pose.position.y);
    proto.set_pose_z(msg->pose.pose.position.z);

    double roll, pitch, yaw;
    tf::Quaternion qua;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, qua);
    tf::Matrix3x3(qua).getRPY(roll, pitch, yaw);
    proto.set_pose_roll(roll);
    proto.set_pose_pitch(pitch);
    proto.set_pose_yaw(yaw);
    for (auto pcov : msg->pose.covariance)
        proto.add_pose_cova(pcov);
    proto.set_twist_x(msg->twist.twist.linear.x);
    proto.set_twist_y(msg->twist.twist.linear.y);
    proto.set_twist_z(msg->twist.twist.linear.z);
    proto.set_twist_roll(msg->twist.twist.angular.x);
    proto.set_twist_pitch(msg->twist.twist.angular.x);
    proto.set_twist_yaw(msg->twist.twist.angular.z);
    for (auto tcov : msg->twist.covariance)
        proto.add_twist_cova(tcov);

    if (!proto.SerializeToString(&content))
    {
        cout << "fail!!!!!" << endl;
        return;
    }
    proto.Clear();

    pack(name, content);
}
