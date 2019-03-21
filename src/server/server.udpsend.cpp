#include "server/server.udpsend.h"

ServerUdpSend::ServerUdpSend():data_send_thread_(NULL)
{
    if(!nh_.getParam("/server_node/targetIP", targetIP_))
        targetIP_ = "127.0.0.1";
    if(!nh_.getParam("/server_node/targetPort", targetPort_))
        targetPort_ = 8001;
    if(!nh_.getParam("/server_node/sendSize", sendSize_))
        sendSize_ = 5;
    send_size_ = sendSize_*1024;

}

ServerUdpSend::~ServerUdpSend() {
    if(data_send_thread_){
        data_send_thread_->join();
        delete data_send_thread_;
    }
}

void ServerUdpSend::start(DataShare *datashare) {
    ds_ = datashare;
    data_send_thread_ = new boost::thread([&]{runThread();});
}

void ServerUdpSend::runThread() {
    struct sockaddr_in targetAddr;
    bzero(&targetAddr, sizeof(targetAddr));
    targetAddr.sin_family = AF_INET;
    targetAddr.sin_addr.s_addr = inet_addr(targetIP_.c_str());
    targetAddr.sin_port = htons(targetPort_);
    int targetAddrLen = sizeof(targetAddr);

    int sockdf = socket(AF_INET, SOCK_DGRAM, 0);

    int n = 0;
    int send_size;
    string buf;
    while (ros::ok())
    {
        if (ds_->server_data_package_.size() > 0)
        {
            ds_->server_data_package_mtx_.lock();
            buf.assign(ds_->server_data_package_);
            ds_->server_data_package_.clear();
            ds_->server_data_package_mtx_.unlock();
            while (!buf.empty())
            {
                send_size = std::min(static_cast<int>(buf.size()), static_cast<int>(send_size_));
                try {
                    n = sendto(sockdf, buf.data(), send_size, 0, (struct sockaddr *) &targetAddr, sizeof(targetAddr));
                }catch (exception& e){
                    ROS_ERROR("send catch exception: %s", e.what());
                }
                if (n == -1)
                    perror("sendto error");
                cout << "send size: " <<  n << endl;
                buf.erase(0, n);
            }

            cout << "----------------" << endl;
        }
    }

    close(sockdf);
}
