#include "server/server.dataparse.h"

ServerDataParse::ServerDataParse() : data_parse_thread_(NULL)
{
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/protobuf_test_lidar", 10);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/protobuf_test_map", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/protobuf_test_path", 10);
    tf_pub_ = nh_.advertise<tf2_msgs::TFMessage>("/tf", 10);
    tf_static_pub_ = nh_.advertise<tf2_msgs::TFMessage>("/tf_static", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/protobuf_test_odom", 10);
}
ServerDataParse::~ServerDataParse()
{
    if (data_parse_thread_)
    {
        data_parse_thread_->join();
        delete data_parse_thread_;
    }
}

void ServerDataParse::start(DataShare *datashare)
{
    ds_ = datashare;
    try
    {
        data_parse_thread_ = new boost::thread([&] { runThread(); });
    }
    catch (exception &e)
    {
    }
}

void ServerDataParse::runThread()
{
    string laser{"proto_msg.LaserScan"};
    string map{"proto_msg.OccupancyGrid"};
    string path{"proto_msg.Path"};
    string tf_message{"proto_msg.TFMessage"};
    string tf_static_message{"proto_msg.TFStaticMessage"};
    string odom{"proto_msg.Odometry"};
    string buffer;
    string content;
    ros::Rate l(50.0);
    while (ros::ok())
    {
//      l.sleep();
        if (ds_->server_data_parse_.size() > 0)
        {
            size_t total_len = stoi(ds_->server_data_parse_.substr(8, 8));
            buffer.assign(ds_->server_data_parse_.substr(0, total_len));

            ds_->server_data_parse_mtx_.lock();
            ds_->server_data_parse_.erase(0, total_len);
            ds_->server_data_parse_mtx_.unlock();

            size_t name_len;
            try
            {
                name_len = stoi(buffer.substr(6, 2));
            }
            catch (exception &e)
            {
                cout << e.what() << endl;
                cout << "server parse throw exceptiong !!!!!!!!" << endl;
            }
            string name_str = buffer.substr(16, name_len);
            size_t content_start_pos = 6 + 2 + 8 + name_len;
            size_t content_length = total_len - content_start_pos - 4;

            content.assign(buffer.substr(content_start_pos, content_length));

            cout << endl
                 << "================" << endl;
            cout << buffer.substr(0, 6) << endl;
            cout << "head_len = " << name_len << endl;
            cout << "name = " << name_str << endl;
            cout << "total_len = " << total_len << endl;
            cout << buffer.substr(total_len - 4, 4) << endl;
            cout << "server_data_parse_ size = " << ds_->server_data_parse_.size() << endl;
            cout << "==================" << endl
                 << endl;

            if (!name_str.compare(laser))
                scanPublish(content);
            if (!name_str.compare(map))
                mapPublish(content);
            if (!name_str.compare(path))
                pathPublish(content);
            if (!name_str.compare(tf_message))
                tfPublish(content);
            if (!name_str.compare(tf_static_message))
                tfStaticPublish(content);
            if (!name_str.compare(odom))
                odomPublish(content);
        }
    }
}

void ServerDataParse::scanPublish(const string &scan)
{
    sensor_msgs::LaserScan ls;
    proto_msg::LaserScan pls;

    bool re = pls.ParseFromString(scan);
    if (!re)
    {
        cout << "parse false!!!!!!!!!!!!!!!" << endl;
        return;
    }
    ls.header.frame_id = pls.frame_id();
    ls.header.stamp = static_cast<ros::Time>(pls.publish_stamp());
    ls.range_min = pls.range_min();
    ls.range_max = pls.range_max();
    ls.angle_min = pls.angle_min();
    ls.angle_max = pls.angle_max();
    ls.scan_time = pls.scan_time();
    ls.time_increment = pls.time_increment();
    ls.angle_increment = pls.angle_increment();
    for (int i = 0; i < pls.ranges_size(); ++i)
        ls.ranges.push_back(pls.ranges(i));
    for (int i = 0; i < pls.intensitys_size(); ++i)
        ls.intensities.push_back(pls.intensitys(i));

    scan_pub_.publish(ls);
    cout << "publish lidar succeed!" << endl;
}

void ServerDataParse::mapPublish(const string &content)
{
    nav_msgs::OccupancyGrid og;
    proto_msg::OccupancyGrid proto_map;

    bool re = proto_map.ParseFromString(content);
    if (!re)
    {
        cout << "parse map false!!!!!!!!!!!!!!!" << endl;
        return;
    }

    og.header.frame_id = proto_map.frame_id();
    og.header.stamp = static_cast<ros::Time>(proto_map.publish_stamp());
    og.info.map_load_time = static_cast<ros::Time>(proto_map.load_time());
    og.info.resolution = proto_map.resolution();
    og.info.width = proto_map.width();
    og.info.height = proto_map.height();
    og.info.origin.position.x = proto_map.x();
    og.info.origin.position.y = proto_map.y();
    og.info.origin.position.z = proto_map.z();
    og.info.origin.orientation.x = proto_map.q_x();
    og.info.origin.orientation.y = proto_map.q_y();
    og.info.origin.orientation.z = proto_map.q_z();
    og.info.origin.orientation.w = proto_map.q_w();
    std::string m = proto_map.map_data();
    og.data.assign(m.begin(), m.end());

    map_pub_.publish(og);
    cout << "publish map succeed!" << endl;
}

void ServerDataParse::pathPublish(const string &content)
{
    nav_msgs::Path p;
    geometry_msgs::PoseStamped ps;
    proto_msg::Path proto_path;

    if (!proto_path.ParseFromString(content))
    {
        cout << "parse false!!!!!!!!!!!!!!!" << endl;
        return;
    }

    p.header.stamp = static_cast<ros::Time>(proto_path.publish_stamp());
    p.header.frame_id = proto_path.frame_id();

    for (int i = 0; i < proto_path.poses_size(); i++)
    {
        ps.header.stamp = static_cast<ros::Time>(proto_path.poses(i).publish_stamp());
        ps.header.frame_id = proto_path.poses(i).frame_id();
        ps.pose.position.x = proto_path.poses(i).x();
        ps.pose.position.y = proto_path.poses(i).y();
        ps.pose.position.z = proto_path.poses(i).z();
        tf::Quaternion qua;
        qua.setRPY(proto_path.poses(i).roll(), proto_path.poses(i).pitch(), proto_path.poses(i).yaw());
        ps.pose.orientation.x = qua.x();
        ps.pose.orientation.y = qua.y();
        ps.pose.orientation.z = qua.z();
        ps.pose.orientation.w = qua.w();

        p.poses.push_back(ps);
    }

    path_pub_.publish(p);
    cout << "publish path succeed!" << endl;
}

void ServerDataParse::tfPublish(const string &content)
{
    tf2_msgs::TFMessage tf;
    geometry_msgs::TransformStamped ts;
    proto_msg::TFMessage proto_tf;

    if (!proto_tf.ParseFromString(content))
    {
        cout << "parse false!!!!!!!!!!!!!!!" << endl;
        return;
    }

    for (int i = 0; i < proto_tf.tfs_size(); i++)
    {
        ts.header.stamp = static_cast<ros::Time>(proto_tf.tfs(i).publish_stamp());
        ts.header.frame_id = proto_tf.tfs(i).frame_id();
        ts.child_frame_id = proto_tf.tfs(i).child_frame_id();
        ts.transform.translation.x = proto_tf.tfs(i).x();
        ts.transform.translation.y = proto_tf.tfs(i).y();
        ts.transform.translation.z = proto_tf.tfs(i).z();
        tf::Quaternion qua;
        qua.setRPY(proto_tf.tfs(i).roll(), proto_tf.tfs(i).pitch(), proto_tf.tfs(i).yaw());
        ts.transform.rotation.x = qua.x();
        ts.transform.rotation.y = qua.y();
        ts.transform.rotation.z = qua.z();
        ts.transform.rotation.w = qua.w();

        tf.transforms.push_back(ts);
    }

    tf_pub_.publish(tf);
    cout << "publish tf succeed!" << endl;
}

void ServerDataParse::tfStaticPublish(const string &content)
{
    tf2_msgs::TFMessage tf;
    geometry_msgs::TransformStamped ts;
    proto_msg::TFMessage proto_tf;

    if (!proto_tf.ParseFromString(content))
    {
        cout << "parse false!!!!!!!!!!!!!!!" << endl;
        return;
    }

    for (int i = 0; i < proto_tf.tfs_size(); i++)
    {
        ts.header.stamp = static_cast<ros::Time>(proto_tf.tfs(i).publish_stamp());
        ts.header.frame_id = proto_tf.tfs(i).frame_id();
        ts.child_frame_id = proto_tf.tfs(i).child_frame_id();
        ts.transform.translation.x = proto_tf.tfs(i).x();
        ts.transform.translation.y = proto_tf.tfs(i).y();
        ts.transform.translation.z = proto_tf.tfs(i).z();
        tf::Quaternion qua;
        qua.setRPY(proto_tf.tfs(i).roll(), proto_tf.tfs(i).pitch(), proto_tf.tfs(i).yaw());
        ts.transform.rotation.x = qua.x();
        ts.transform.rotation.y = qua.y();
        ts.transform.rotation.z = qua.z();
        ts.transform.rotation.w = qua.w();

        tf.transforms.push_back(ts);
    }

    tf_static_pub_.publish(tf);
    cout << "publish tf_static succeed!" << endl;

    ds_->server_tf_static_state_ = true;
}

void ServerDataParse::odomPublish(const string &content)
{
    nav_msgs::Odometry odom;
    proto_msg::Odometry proto_odom;

    bool re = proto_odom.ParseFromString(content);
    if (!re)
    {
        cout << "parse false!!!!!!!!!!!!!!!" << endl;
        return;
    }
    odom.header.stamp = static_cast<ros::Time>(proto_odom.publish_stamp());

    odom.child_frame_id = proto_odom.child_frame_id();
    odom.header.frame_id = proto_odom.frame_id();
    odom.pose.pose.position.x = proto_odom.pose_x();
    odom.pose.pose.position.y = proto_odom.pose_y();
    odom.pose.pose.position.z = proto_odom.pose_z();
    tf::Quaternion qua;
    qua.setRPY(proto_odom.pose_roll(), proto_odom.pose_pitch(), proto_odom.pose_yaw());
    odom.pose.pose.orientation.x = qua.x();
    odom.pose.pose.orientation.y = qua.y();
    odom.pose.pose.orientation.z = qua.z();
    odom.pose.pose.orientation.w = qua.w();
    for (int i = 0; i < proto_odom.pose_cova_size(); i++)
    {
        odom.pose.covariance.assign(proto_odom.pose_cova(i));
    }

    odom.twist.twist.linear.x = proto_odom.twist_x();
    odom.twist.twist.linear.y = proto_odom.twist_y();
    odom.twist.twist.linear.z = proto_odom.twist_z();
    odom.twist.twist.angular.x = proto_odom.twist_roll();
    odom.twist.twist.angular.y = proto_odom.twist_pitch();
    odom.twist.twist.angular.z = proto_odom.twist_yaw();
    for (int i = 0; i < proto_odom.twist_cova_size(); i++)
    {
        odom.twist.covariance.assign(proto_odom.twist_cova(i));
    }

    odom_pub_.publish(odom);
    cout << "publish odom_pub succeed!" << endl;

}
