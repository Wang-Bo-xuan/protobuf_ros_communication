#pragma once
#include <iostream>
#include <string>
#include <boost/thread.hpp>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
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

class ServerDataParse{
public:
    ServerDataParse();
    ~ServerDataParse();
    void start(DataShare* datashare);
private:
    ros::NodeHandle nh_;
    ros::Publisher scan_pub_;
    ros::Publisher map_pub_;
    ros::Publisher path_pub_;
    ros::Publisher tf_pub_;
    ros::Publisher tf_static_pub_;
    ros::Publisher odom_pub_;

    boost::thread* data_parse_thread_;
    void runThread();

    DataShare* ds_;

    void scanPublish(const string& scan);
    void mapPublish(const string& content);
    void pathPublish(const string& content);
    void tfPublish(const string& content);
    void tfStaticPublish(const string& content);
    void odomPublish(const string& content);

};

