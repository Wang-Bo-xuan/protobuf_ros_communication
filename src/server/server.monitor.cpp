#include "server/server.monitor.h"


ServerMonitor::ServerMonitor():monitor_thread_(NULL){
    map_state_ = false;
    tf_static_state_ = false;
    if(!nh_.getParam("server_node/mode", mode_)){
        cout << "can not get mode!!!" << endl;
        return;
    }
}

ServerMonitor::~ServerMonitor() {
    if(monitor_thread_){
        monitor_thread_->join();
        delete(monitor_thread_);
    }
}

void ServerMonitor::start(DataShare *datashare) {
    ds_ = datashare;
    monitor_thread_ = new boost::thread([&]{runThread();});
}

void ServerMonitor::runThread()
{
    if(mode_ == "nav")
    {
        ros::Subscriber map_sub = nh_.subscribe("/protobuf_test_map", 1, &ServerMonitor::mapCb, this);
        ros::Rate loop_rate(20.0);

        while (ros::ok())
        {
          loop_rate.sleep();

            if (ds_->server_control_command_.compare("start"))
              continue;

            if (!map_state_)
            {
                ROS_ERROR("Get map fail!!!");
                string name{"proto_msg.MapRequest"};
                string content{""};
                // if(ds_->client_data_package_.size() > buffer_package_size_) return;
                if (ds_->client_data_package_.size() > 1024)
                    return;

                ds_->server_data_package_mtx_.lock();
                ds_->server_data_package_.append(common.dataPack(name, content));
                ds_->server_data_package_mtx_.unlock();
            }

            if (!ds_->server_tf_static_state_)
            {
                ROS_ERROR("Get tf_static fail!!!");
                string name{"proto_msg.TFStaticRequest"};
                string content{""};
                if (ds_->client_data_package_.size() > 1024)
                    return;

                ds_->server_data_package_mtx_.lock();
                ds_->server_data_package_.append(common.dataPack(name, content));
                ds_->server_data_package_mtx_.unlock();
            }
            ros::spinOnce();
        }
    }

    if(mode_ == "map")
    {
        ros::Rate loop_rate(20.0);
        while(ros::ok())
        {
            loop_rate.sleep();

            if (ds_->server_control_command_.compare("start"))
              continue;

            if(!ds_->server_tf_static_state_)
            {
                ROS_ERROR("Get tf_static fail!!!");
                string name{"proto_msg.TFStaticRequest"};
                string content{""};
                if (ds_->client_data_package_.size() > 1024)
                    return;
                ds_->server_data_package_mtx_.lock();
                ds_->server_data_package_.append(common.dataPack(name, content));
                ds_->server_data_package_mtx_.unlock();
            }
            ros::spinOnce();
        }
    }
}

void ServerMonitor::mapCb(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    map_state_ = true;
}






