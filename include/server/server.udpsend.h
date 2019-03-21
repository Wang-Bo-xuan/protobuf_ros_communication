#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>
#include <algorithm>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include "datashare.h"

using namespace std;

class ServerUdpSend{
public:
    ServerUdpSend();
    ~ServerUdpSend();

    void start(DataShare* datashare);

private:
    ros::NodeHandle nh_;
    DataShare* ds_;

    boost::thread* data_send_thread_;
    void runThread();

    string targetIP_;
    int targetPort_;
    int sendSize_;

    size_t send_size_;

};

