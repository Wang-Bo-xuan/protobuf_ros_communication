#include "utils/datashare.h"
#include "client/client.datapackage.h"
#include "client/client.dataparse.h"
#include "client/client.udprecv.h"
#include "client/client.udpsend.h"

#include <iostream>

int main(int argc, char** argv){
    ros::init(argc, argv, "client_node");

    DataShare* ds = DataShare::getInstance();

    ClientDataPackage  data_package;
    ClientUdpSend      udp_send;
    ClientUdpRecv      udp_recv;
    ClientDataParse    data_parse;

    data_package.start(ds);
    udp_send.start(ds);
    udp_recv.start(ds);
    data_parse.start(ds);
}