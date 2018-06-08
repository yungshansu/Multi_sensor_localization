#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <stdio.h>
#include <math.h>

class TangoVisualization{
public:
    TangoVisualization();

private:
    ros::NodeHandle nh;
    ros::Publisher path_vis_publisher;
    ros::Subscriber tango_pose_subscriber;
    visualization_msgs::Marker path_list;
    geometry_msgs::PoseStamped last_pose;
    void marker_setup();
    void tangoCallback(const geometry_msgs::PoseStamped& pose);
    void draw_tango_map(geometry_msgs::PoseStamped recent_pose);   
};

TangoVisualization::TangoVisualization()
{
    path_vis_publisher = nh.advertise<visualization_msgs::Marker>("tango_path", 0);
    tango_pose_subscriber = nh.subscribe("/tango_sensors/tango_pose", 1, &TangoVisualization::tangoCallback, this);
    marker_setup();

}

void TangoVisualization::marker_setup(){
    path_list.header.stamp = ros::Time::now();
    path_list.header.frame_id = "/map";
    path_list.action = visualization_msgs::Marker::ADD;
    path_list.pose.orientation.w = 1.0;
    path_list.id = 2;
    path_list.type = visualization_msgs::Marker::LINE_LIST;    
    path_list.scale.x = 0.06; 
    path_list.scale.y = 0.06;
    path_list.color.r=1.0;
    path_list.color.a=1.0;
    last_pose.pose.position.x =0;
    last_pose.pose.position.y =0;
    last_pose.pose.position.z =0;
}

void TangoVisualization::tangoCallback(const geometry_msgs::PoseStamped& pose){
    geometry_msgs::PoseStamped recent_pose = pose;
    draw_tango_map(recent_pose);
    last_pose = recent_pose;
}

void TangoVisualization::draw_tango_map(geometry_msgs::PoseStamped recent_pose){
    geometry_msgs::Point p1; 
    geometry_msgs::Point p2;
    p1.x = last_pose.pose.position.x;
    p1.y = last_pose.pose.position.y;
    p1.z = last_pose.pose.position.z;
    path_list.points.push_back(p1);
    p2.x = recent_pose.pose.position.x;
    p2.y = recent_pose.pose.position.y;
    p2.z = recent_pose.pose.position.z;
    path_list.points.push_back(p2);
    path_vis_publisher.publish(path_list);
}


int   main (int argc, char** argv)
{
    ros::init(argc, argv, "apriltag_localization");
    TangoVisualization tango_visualize;
    ros::spin();
}

