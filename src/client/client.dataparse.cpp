#include "client/client.dataparse.h"

ClientDataParse::ClientDataParse() : data_parse_thread_(NULL)
{
    if(!nh_.getParam("client_node/mode", mode_)){
        ROS_ERROR("data parse not get mode!!");
        return;
    }
    //    pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/protobuf_pose_stamped", 10);
    pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
}
ClientDataParse::~ClientDataParse()
{
    if (data_parse_thread_)
    {
        data_parse_thread_->join();
        delete data_parse_thread_;
    }
}

void ClientDataParse::start(DataShare *datashare)
{
    ds_ = datashare;
    data_parse_thread_ = new boost::thread([&] { runThread(); });
}

void ClientDataParse::runThread()
{
    string pose{"proto_msg.PoseStamped"};
    string control{"proto_msg.control"};
    string tf_static_request{"proto_msg.TFStaticRequest"};
    string buffer;
    string content;
    while (ros::ok())
    {
        if (ds_->client_data_parse_.size() > 0)
        {
            size_t total_len = stoi(ds_->client_data_parse_.substr(8, 8));
            buffer.assign(ds_->client_data_parse_.substr(0, total_len));

            ds_->client_data_parse_mtx_.lock();
            ds_->client_data_parse_.erase(0, total_len);
            ds_->client_data_parse_mtx_.unlock();

            size_t name_len = stoi(buffer.substr(6, 2));
            string name_str = buffer.substr(16, name_len);
            size_t content_start_pos = 6 + 2 + 8 + name_len;
            size_t content_length = total_len - content_start_pos - 4;

            content.assign(buffer.substr(content_start_pos, content_length));

            cout << endl
                 << "================" << endl;
            cout << buffer.substr(0, 6) << endl;
            cout << "head_len = " << name_len << endl;
            cout << "name = " << name_str << endl;
            cout << "total_len = " << total_len << endl;
            cout << buffer.substr(total_len - 4, 4) << endl;
            cout << "server_data_parse_ size = " << ds_->server_data_parse_.size() << endl;
            cout << "==================" << endl
                 << endl;

            if (!name_str.compare(pose))
                poseStampedPublish(content);
            if (!name_str.compare(control))
                controlPublish(content);

            if(!name_str.compare(tf_static_request))
                sendTFStaticAgain();

            if(mode_ == "nav"){
                string map_request{"proto_msg.MapRequest"};
                if(!name_str.compare(map_request))
                    sendMapAgain();
            }

        }
    }
}

void ClientDataParse::poseStampedPublish(const string &content)
{
    //    cout << "enter pose stamped publish" << endl;
    if (ds_->client_control_command_.compare("start"))
        return;
    geometry_msgs::PoseStamped publ;
    proto_msg::PoseStamped proto;

    bool re = proto.ParseFromString(content);
    if (!re)
    {
        cout << "parse false!!!!!!!!!!!!!!!" << endl;
        return;
    }

    publ.header.stamp = static_cast<ros::Time>(proto.stamp());
    publ.header.frame_id = proto.frame_id();
    publ.pose.position.x = proto.x();
    publ.pose.position.y = proto.y();
    publ.pose.position.z = proto.z();
    publ.pose.orientation.x = proto.q_x();
    publ.pose.orientation.y = proto.q_y();
    publ.pose.orientation.z = proto.q_z();
    publ.pose.orientation.w = proto.q_w();

    pose_stamped_pub_.publish(publ);
    cout << "publish posestamped succeed!" << endl;

    //    ds_->client_control_command_ = "start";
}

void ClientDataParse::controlPublish(const string &msg)
{
    cout << "enter control" << endl;
    cout << msg << endl;
    ds_->client_control_command_ = msg;
}

void ClientDataParse::sendMapAgain() {
    cout << "ds_->client_backup_map_.size() = " << ds_->client_backup_map_.size() << endl;
    if (ds_->client_control_command_.compare("start"))
        return;
    if(ds_->client_backup_map_.size() == 0){
        ROS_ERROR("map buffer is empty!");
        return;
    }
    string name{"proto_msg.OccupancyGrid"};
    ds_->client_backup_map_mtx_.lock();
    string content = ds_->client_backup_map_;
    ds_->client_backup_map_mtx_.unlock();
    string package = common.dataPack(name, content);
    ds_->client_data_package_mtx_.lock();
    ds_->client_data_package_.append(package);
    ds_->client_data_package_mtx_.unlock();
}

void ClientDataParse::sendTFStaticAgain() {
    if (ds_->client_control_command_.compare("start"))
        return;
    if(ds_->client_backup_tf_static_.size() == 0){
        ROS_ERROR("tf_static buffer is empty!");
        return;
    }
    string name{"proto_msg.TFStaticMessage"};
    ds_->client_backup_tf_static_mtx_.lock();
    string content = ds_->client_backup_tf_static_;
    ds_->client_backup_tf_static_mtx_.unlock();
    string package = common.dataPack(name, content);
    ds_->client_data_package_mtx_.lock();
    ds_->client_data_package_.append(package);
    ds_->client_data_package_mtx_.unlock();
}
