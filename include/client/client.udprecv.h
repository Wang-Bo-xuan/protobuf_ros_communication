#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>


#include <iostream>
#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include "utils/datashare.h"


using namespace std;

class ClientUdpRecv{
public:
    ClientUdpRecv();
    ~ClientUdpRecv();

    void start(DataShare* datashare);

private:
    ros::NodeHandle nh_;
    DataShare* ds_;

    boost::thread* data_recv_thread_;
    void runThread();


    int localPort_;
    int recvSize_;
    int bufferParseSize_;

    size_t recv_size_;
    size_t buffer_parse_size_;
};
