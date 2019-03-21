#include "client/client.udprecv.h"


ClientUdpRecv::ClientUdpRecv():data_recv_thread_(NULL) {
    if(!nh_.getParam("/client_node/localPort", localPort_))
        localPort_ = 8001;
    if(!nh_.getParam("/client_node/recvSize", recvSize_))
        recvSize_ = 5;
    if(!nh_.getParam("/client_node/bufferParseSize", bufferParseSize_))
        bufferParseSize_ = 1;
    recv_size_ = recvSize_*1024;
    buffer_parse_size_ = bufferParseSize_*1024*1024;
}
ClientUdpRecv::~ClientUdpRecv() {
    if(data_recv_thread_){
        data_recv_thread_->join();
        delete data_recv_thread_;
    }
}

void ClientUdpRecv::start(DataShare* datashare) {
    ds_ = datashare;
    data_recv_thread_ = new boost::thread([&]{runThread();});
}
void ClientUdpRecv::runThread() {

    struct sockaddr_in srvAddr;
    bzero(&srvAddr, sizeof(srvAddr));
    srvAddr.sin_family = AF_INET;
    srvAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    srvAddr.sin_port = htons(localPort_);
    int srvAddrLen = sizeof(srvAddr);

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        perror("socket failed:");
    }

    if(bind(sockfd, (struct sockaddr *)&srvAddr, sizeof(srvAddr)) == -1)
        perror("bind failed:");

    struct sockaddr_in cliAddr;
    bzero(&cliAddr, sizeof(cliAddr));
    cliAddr.sin_family = AF_INET;
    int cliAddrLen = sizeof(cliAddr);

    char buff[recv_size_ + 1];
    int n = 0;
    string buffer;
    char header[7] = "$START";
    size_t head_n = 0;
    while (ros::ok())
    {
        bzero(&buff, sizeof(buff));

        try {
            n = recvfrom(sockfd, buff, recv_size_, 0, (struct sockaddr *) &cliAddr, (socklen_t *) &cliAddrLen);
        }catch (exception &e){
            ROS_ERROR("recv catch exception %s", e.what());
        }

        if (n == -1)
            perror("recvfrom error");
        cout << "recv_size=" << n << endl;

        if (!strncmp(reinterpret_cast<const char *>(&buff), reinterpret_cast<const char *>(&header), 6)) {
            buffer.assign(buff, n);
        }else{
            buffer.append(buff, n);
        }
        if (buff[n-4] == '$' && buff[n-3] == 'E' && buff[n-2] == 'N' && buff[n-1] == 'D'){
            cout << "13" << buffer << endl;
            size_t name_len = stoi(buffer.substr(6, 2));
            size_t total_len = stoi(buffer.substr(8, 8));
            string name_str = buffer.substr(16, name_len);

            cout << "buffer.size = " << buffer.size() << endl;
            cout << "total_len = " << total_len << endl;

//            if (buffer.size() != total_len)
//            {
//                buffer.clear();
//                cout << "package loss or out of order!!!!!!!!!";
//            }

            if(ds_->client_data_parse_.size() < buffer_parse_size_) {
                ds_->client_data_parse_mtx_.lock();
                ds_->client_data_parse_.append(buffer);
                ds_->client_data_parse_mtx_.unlock();


                cout << endl
                     << "================" << endl;
                cout << "recv_n = " << head_n << endl;
                cout << buffer.substr(0, 6) << endl;
                cout << "head_len = " << name_len << endl;
                cout << "name = " << name_str << endl;
                cout << "total_len = " << total_len << endl;
                cout << buffer.substr(total_len - 4, 4) << endl;
                cout << "client_data_package_ size = " << ds_->client_data_package_.size() << endl;
                cout << "==================" << endl
                     << endl;
            }

        }
    }
}
