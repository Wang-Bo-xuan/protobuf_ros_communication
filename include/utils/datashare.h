#pragma once

#include <string>
#include <boost/thread/mutex.hpp>
#include <atomic>
//#include <nav_msgs/OccupancyGrid.h>

using namespace std;
class DataShare{
public:
    DataShare(){
        server_tf_static_state_ = false;
    }
    static DataShare* getInstance(){
        static DataShare* instance = new DataShare();
        return instance;
    }



    //client
    string client_data_package_;
    boost::mutex client_data_package_mtx_;

    string client_data_parse_;
    boost::mutex client_data_parse_mtx_;

    string client_control_command_;
    boost::mutex client_control_command_mtx_;

    string client_backup_map_;
    boost::mutex client_backup_map_mtx_;

    string client_backup_tf_static_;
    boost::mutex client_backup_tf_static_mtx_;

    //server
    string server_data_package_;
    boost::mutex server_data_package_mtx_;

    string server_data_parse_;
    boost::mutex server_data_parse_mtx_;

    string server_control_command_;
    boost::mutex server_control_command_mtx_;

    std::atomic_bool server_tf_static_state_;

};