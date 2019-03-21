#pragma once
#include <iostream>
#include <algorithm>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

#include "utils/datashare.h"
#include "proto_msgs.MoveBaseActionGoal.pb.h"
#include "proto_msgs.PoseStamped.pb.h"

using namespace std;

class ServerDataPackage{
public:
    ServerDataPackage();
    ~ServerDataPackage();
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

    ros::NodeHandle nh_;

    boost::thread* data_package_thread_;
    void runThread();

    DataShare* ds_;

    void moveBaseActionGoalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr &msg);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void controlCallback(const std_msgs::String::ConstPtr &msg);

    void pack(const string& name, const string& content);

    int bufferPackageSize_;
    size_t buffer_package_size_;

};

