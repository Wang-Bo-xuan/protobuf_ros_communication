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

#include "utils/datashare.h"

using namespace std;

class ClientUdpSend{
public:
    ClientUdpSend();
    ~ClientUdpSend();

    void start(DataShare* datashare);

private:
    ros::NodeHandle nh_;
    DataShare* ds_;

    boost::thread* data_send_thread_;
    void runThread();

    size_t send_size_;
    int sendSize_;

    string targetIP_;
    int targetPort_;

};
