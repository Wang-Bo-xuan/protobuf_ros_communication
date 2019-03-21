#include "server/server.udprecv.h"

ServerUdpRecv::ServerUdpRecv() : data_recv_thread_(NULL)
{
    if (!nh_.getParam("/server_node/localPort", localPort_))
        localPort_ = 8002;
    if (!nh_.getParam("/server_node/recvSize", recvSize_))
        recvSize_ = 5;
    if (!nh_.getParam("/client_node/bufferParseSize", bufferParseSize_))
        bufferParseSize_ = 3;
    recv_size_ = recvSize_ * 1024;
    buffer_parse_size_ = bufferParseSize_ * 1024 * 1024;
}
ServerUdpRecv::~ServerUdpRecv()
{
    if (data_recv_thread_)
    {
        data_recv_thread_->join();
        delete data_recv_thread_;
    }
}

void ServerUdpRecv::start(DataShare *datashare)
{
    ds_ = datashare;
    data_recv_thread_ = new boost::thread([&] { runThread(); });
}
void ServerUdpRecv::runThread()
{
    struct sockaddr_in srvAddr;
    bzero(&srvAddr, sizeof(srvAddr));
    srvAddr.sin_family = AF_INET;
    srvAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    srvAddr.sin_port = htons(localPort_);
    int srvAddrLen = sizeof(srvAddr);

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1)
    {
        perror("socket failed:");
    }

    //    struct timeval timeout;
    //    timeout.tv_sec = 1;
    //    timeout.tv_usec = 0;
    //    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
    //        perror("setsockopt failed:");
    //    }

    if (bind(sockfd, (struct sockaddr *)&srvAddr, sizeof(srvAddr)) == -1)
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
    size_t i = 0;
    while (ros::ok())
    {
        bzero(&buff, sizeof(buff));

        n = recvfrom(sockfd, buff, recv_size_, 0, (struct sockaddr *) &cliAddr, (socklen_t *) &cliAddrLen);

//        try {
//            n = recvfrom(sockfd, buff, recv_size_, 0, (struct sockaddr *) &cliAddr, (socklen_t *) &cliAddrLen);
//        }catch (exception& e){
//            ROS_ERROR("recv catch exception: %s", e.what());
//        }

        if (n == -1)
            perror("recvfrom error");

        cout << "recv_size=" << n << endl;
        for (size_t i = 2; i < n; ++i)
        {
            buff[i - 2] = buff[i];
        }
//        buff[n - 1] = '\0';
        n -= 2;
        if (!strncmp(reinterpret_cast<const char *>(&buff), reinterpret_cast<const char *>(&header), 6))
        {
            buffer.assign(buff, n);

        }
        else
        {
            buffer.append(buff, n);
        }

//        cout << "buffer size = " << buffer.size() << endl;
//        cout << "buffer  = " << buffer.substr(0,30) << endl;
//        cout << "buffer  = " << buffer.substr(buffer.size() -20,20) << endl;
        if (buff[n - 4] == '$' && buff[n - 3] == 'E' && buff[n - 2] == 'N' && buff[n - 1] == 'D')
        {
            size_t name_len;
            size_t total_len;

            try
            {
                name_len = stoi(buffer.substr(6, 2));
                total_len = stoi(buffer.substr(8, 8));
            }
            catch (exception &e)
            {
                cout << e.what() << endl;
                cout << "server recv throw exceptiong !!!!!!!!" << endl;
            }
            //            size_t name_len = stoi(buffer.substr(6, 2));
            //            size_t total_len = stoi(buffer.substr(8, 8));
            string name_str = buffer.substr(16, name_len);
            if (buffer.size() != total_len)
            {
                //                buffer.clear();
                cout << "package loss or out of order!!!!!!!!!";
            }
            else if (ds_->server_data_parse_.size() < buffer_parse_size_)
            {
                ds_->server_data_parse_mtx_.lock();
                ds_->server_data_parse_.append(buffer);
                ds_->server_data_parse_mtx_.unlock();

                 cout << endl
                      << "================" << endl;
                 cout << "recv_n = " << head_n << endl;
                 cout << buffer.substr(0, 6) << endl;
                 cout << "head_len = " << name_len << endl;
                 cout << "name = " << name_str << endl;
                 cout << "total_len = " << total_len << endl;
                 cout << buffer.substr(total_len - 4, 4) << endl;
                 cout << "server_data_package_ size = " << ds_->server_data_package_.size() << endl;
                 cout << "==================" << endl
                      << endl;
            }
        }
    }
}
