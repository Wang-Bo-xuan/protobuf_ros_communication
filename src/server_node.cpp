#include "utils/datashare.h"
#include "server.datapackage.h"
#include "server.dataparse.h"
#include "server.udpsend.h"
#include "server.udprecv.h"
#include "server.monitor.h"


#include <iostream>

int main(int argc, char** argv){
    ros::init(argc, argv, "server_node");

    DataShare* ds = DataShare::getInstance();

    ServerDataPackage  data_package;
    ServerUdpSend      udp_send;
    ServerUdpRecv      udp_recv;
    ServerDataParse    data_parse;
    ServerMonitor      monitor;

    data_package.start(ds);
    udp_send.start(ds);
    udp_recv.start(ds);
    data_parse.start(ds);
    monitor.start(ds);
}