#include "server/server.datapackage.h"

ServerDataPackage::ServerDataPackage():data_package_thread_(NULL) {
    if(!nh_.getParam("/server_node/bufferPackageSize", bufferPackageSize_))
        bufferPackageSize_ = 1;
    buffer_package_size_ = bufferPackageSize_*1024*1024;
}

ServerDataPackage::~ServerDataPackage() {
    if(data_package_thread_){
        data_package_thread_->join();
        delete data_package_thread_;
    }
}

void ServerDataPackage::start(DataShare* datashare) {
    ds_ = datashare;
    data_package_thread_ = new boost::thread([&]{runThread();});
}

void ServerDataPackage::runThread() {
    ros::Subscriber goal_sub = nh_.subscribe("/move_base/goal", 1, &ServerDataPackage::moveBaseActionGoalCallback, this);
    ros::Subscriber pose_stamped_sub = nh_.subscribe("/move_base_simple/goal", 1, &ServerDataPackage::poseStampedCallback, this);
    ros::Subscriber laser_control_sub = nh_.subscribe("/control", 10, &ServerDataPackage::controlCallback, this);



    //    ros::Subscriber lidar_sub = nh_.subscribe("/scan", 10, &ServerDataPackage::lidarCallback, this);

    ros::spin();
}

void ServerDataPackage::pack(const string &name, const string &content) {
    SensorData buff;
    buff.name = name;
    buff.content = content;
    buff.total_len.resize(8);
    buff.name_len = to_string(buff.name.size());
    buff.total_len = to_string(buff.start.size() + buff.name_len.size() + buff.total_len.size() + buff.name.size() + buff.content.size() + buff.terminator.size());
    buff.total_len.resize(8);
    buff.all = buff.start + buff.name_len + buff.total_len + buff.name + buff.content + buff.terminator;

    if(ds_->server_data_package_.size() > buffer_package_size_) return;
    ds_->server_data_package_mtx_.lock();
    ds_->server_data_package_.append(buff.all);
    ds_->server_data_package_mtx_.unlock();

    cout << endl << "------------------------" << endl;
    cout << buff.start << endl;
    cout << buff.name << endl;
    cout << buff.total_len << " " << buff.all.size() << endl;
    cout << buff.terminator << endl;
    cout << "server_data_package_ size : " << ds_->server_data_package_.size() << endl;
    cout << "------------------------" << endl << endl;
}


void ServerDataPackage::moveBaseActionGoalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr &msg) {
    cout << "enter goal callback" << endl;
}

void ServerDataPackage::poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
//        cout << " enter poseStamped callback" << endl;
    proto_msg::PoseStamped proto_ps;
    string name{"proto_msg.PoseStamped"};
    string content;

    proto_ps.set_stamp(msg->header.stamp.toSec());
    proto_ps.set_frame_id(msg->header.frame_id);
    proto_ps.set_x(msg->pose.position.x);
    proto_ps.set_y(msg->pose.position.y);
    proto_ps.set_z(msg->pose.position.z);
    proto_ps.set_q_x(msg->pose.orientation.x);
    proto_ps.set_q_y(msg->pose.orientation.y);
    proto_ps.set_q_z(msg->pose.orientation.z);
    proto_ps.set_q_w(msg->pose.orientation.w);
    proto_ps.SerializeToString(&content);
    proto_ps.Clear();

    pack(name, content);

}

void ServerDataPackage::controlCallback(const std_msgs::String::ConstPtr &msg) {
    ds_->server_control_command_mtx_.lock();
    ds_->server_control_command_ = msg->data;
    cout << "msg->data = " << ds_->server_control_command_<< endl;
    ds_->server_control_command_mtx_.unlock();

    string name{"proto_msg.control"};
    string content{msg->data};

    pack(name, content);
}

