#pragma once
#include <iostream>
#include <algorithm>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tf.h>

#include "utils/datashare.h"
#include "proto_msgs.LaserScan.pb.h"
#include "proto_msgs.OccupancyGrid.pb.h"
#include "proto_msgs.Path.pb.h"
#include "proto_msgs.TFMessage.pb.h"
#include "proto_msgs.Odometry.pb.h"

using namespace std;

class ClientDataPackage{
public:
    ClientDataPackage();
    ~ClientDataPackage();
    void start(DataShare* datashare);
private:
    struct SensorData
    {
        string start{"$START"};
        string name_len;
        string total_len;
        string name;
        string content;
        string terminator{"$END"};
        string all;
    };

    DataShare *ds_;

    boost::thread* data_package_thread_;
    void runThread();

    ros::NodeHandle nh_;

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void pathCallback(const nav_msgs::Path::ConstPtr &msg);
    void TFCallback(const tf2_msgs::TFMessage::ConstPtr &msg);
    void TFStaticCallback(const tf2_msgs::TFMessage::ConstPtr &msg);
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg);

    void pack(const string &name, const string &content);

    int bufferPackageSize_;
    size_t buffer_package_size_;

    string mode_;
};
