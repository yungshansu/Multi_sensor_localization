#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_msgs/TFMessage.h>

using namespace std;

class AprilTagLocalization{
public:
    AprilTagLocalization();

private:
    ros::NodeHandle nh;
    ros::Subscriber tag_sub;
    ros::Publisher vis_pub;
    ros::Publisher marker_pub;
    visualization_msgs::Marker marker;
    visualization_msgs::Marker map;
    void LocalCallback(const tf2_msgs::TFMessage& tags);
    void draw_the_map();
    geometry_msgs::Point tag_1, tag_2, tag_3, tag_4, tag_5, tag_6, tag_7, tag_8, tag_9, tag_10, tag_11, tag_12, tag_13, tag_14, tag_15, tag_16, tag_17;
};

AprilTagLocalization::AprilTagLocalization()
{
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_map", 1);
    tag_sub = nh.subscribe("tf", 1, &AprilTagLocalization::LocalCallback, this);
    //draw_the_map();
}

void AprilTagLocalization::draw_the_map()
{
   // visualization_msgs::Marker map;
    map.header.frame_id = "/map";
    map.header.stamp = ros::Time();
    map.type = visualization_msgs::Marker::POINTS;
    map.action = visualization_msgs::Marker::ADD;
    map.id = 0;
    map.scale.x = 0.1;
    map.scale.y = 0.1;
    map.scale.z = 0.5;
    map.color.a = 1.0;
    map.color.r = 1.0;
    map.color.g = 1.0;
    map.color.b = 1.0;

   // geometry_msgs::Point tag_1, tag_2, tag_3, tag_4, tag_5, tag_6, tag_7, tag_8, tag_9;

    tag_1.x = 0.0;tag_1.y = 0.0;tag_1.z = 1.5;
    tag_2.x = 0.0;tag_2.y = 8.0;tag_2.z = 1.5;
    tag_3.x = 0.0;tag_3.y = 16.0;tag_3.z = 1.5;
    tag_4.x = 0.0;tag_4.y = 24.0;tag_4.z = 1.5;
    tag_5.x = 0.0;tag_5.y = 32.0;tag_5.z = 1.5;
    tag_6.x = 0.0;tag_6.y = 40.0;tag_6.z = 1.5;
    tag_7.x = 0.0;tag_7.y = 48.0;tag_7.z = 1.5;
    tag_8.x = 0.0;tag_8.y = 55.6;tag_8.z = 1.5;
    tag_9.x = -7.8;tag_9.y = 55.6;tag_9.z = 1.5;
    tag_10.x = -7.8;tag_10.y = 51.6;tag_10.z = 1.5;
    tag_11.x = -15.4;tag_11.y = 51.6;tag_11.z = 1.5;
    tag_12.x = -23.4;tag_12.y = 51.6;tag_12.z = 1.5;
    tag_13.x = -31.4;tag_13.y = 51.6;tag_13.z = 1.5;
    tag_14.x = -39.4;tag_14.y = 51.6;tag_14.z = 1.5;
    tag_15.x = -47.4;tag_15.y = 51.6;tag_15.z = 1.5;
    tag_16.x = -55.4;tag_16.y = 51.6;tag_16.z = 1.5;
    tag_17.x = -67.2;tag_17.y = 51.6;tag_17.z = 1.5;

    map.points.push_back(tag_1);
    map.points.push_back(tag_2);
    map.points.push_back(tag_3);
    map.points.push_back(tag_4);
    map.points.push_back(tag_5);
    map.points.push_back(tag_6);
    map.points.push_back(tag_7);
    map.points.push_back(tag_8);
    map.points.push_back(tag_9);
    map.points.push_back(tag_10);
    map.points.push_back(tag_11);
    map.points.push_back(tag_12);
    map.points.push_back(tag_13);
    map.points.push_back(tag_14);
    map.points.push_back(tag_15);
    map.points.push_back(tag_16);
    map.points.push_back(tag_17);

    //while (ros::ok()){
    vis_pub.publish(map);
    //}
}

void AprilTagLocalization::LocalCallback(const tf2_msgs::TFMessage& tags)
{
    draw_the_map();
    //AprilTagDetection tag_detection
    //visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 1;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    int r = 0;    
    float tx = 1, ty = 1;  
    //std::vector<geometry_msgs::Point> my_points;

    for(int i=0; i<tags.transforms.size(); i++){
	geometry_msgs::Point p;
        

        if(tags.transforms[i].child_frame_id == "tag_1") {
            tx = tag_1.x;ty = tag_1.y;r = 0;}
        else if(tags.transforms[i].child_frame_id == "tag_2") {
            tx = tag_2.x;ty = tag_2.y;r = 0;}
        else if(tags.transforms[i].child_frame_id == "tag_3") {
            tx = tag_3.x;ty = tag_3.y;r=0;}
        else if(tags.transforms[i].child_frame_id == "tag_4") {
            tx = tag_4.x;ty = tag_4.y;r=0;}
        else if(tags.transforms[i].child_frame_id == "tag_5") {
            tx = tag_5.x;ty = tag_5.y;r=0;}
        else if(tags.transforms[i].child_frame_id == "tag_6") {
            tx = tag_6.x;ty = tag_6.y;r=0;}
        else if(tags.transforms[i].child_frame_id == "tag_7") {
            tx = tag_7.x;ty = tag_7.y;r=0;}
        else if(tags.transforms[i].child_frame_id == "tag_8") {
            tx = tag_8.x;ty = tag_8.y;r=1;}
        else if(tags.transforms[i].child_frame_id == "tag_9") {
            tx = tag_9.x;ty = tag_9.y;r=2;}
        else if(tags.transforms[i].child_frame_id == "tag_10") {
            tx = tag_10.x;ty = tag_10.y;r=2;}
        else if(tags.transforms[i].child_frame_id == "tag_11") {
            tx = tag_11.x;ty = tag_11.y;r=1;}
        else if(tags.transforms[i].child_frame_id == "tag_12") {
            tx = tag_12.x;ty = tag_12.y;r=1;}
        else if(tags.transforms[i].child_frame_id == "tag_13") {
            tx = tag_13.x;ty = tag_13.y;r=1;}
        else if(tags.transforms[i].child_frame_id == "tag_14") {
            tx = tag_14.x;ty = tag_14.y;r=1;}
        else if(tags.transforms[i].child_frame_id == "tag_15") {
            tx = tag_15.x;ty = tag_15.y;r=1;}
        else if(tags.transforms[i].child_frame_id == "tag_16") {
            tx = tag_16.x;ty = tag_16.y;r=1;}
        else if(tags.transforms[i].child_frame_id == "tag_17") {
            tx = tag_17.x;ty = tag_17.y;r=1;}
        //p.x = tags.detections[i].pose.pose.position.x;   
        //p.y = tags.detections[i].pose.pose.position.y;   
        //p.z = tags.detections[i].pose.pose.position.z;
        if (r == 0){
            p.x = -1 * tags.transforms[i].transform.translation.z +tx;
            p.y = tags.transforms[i].transform.translation.x +ty;
            p.z = tags.transforms[i].transform.translation.y + 1.5;}
        else if (r == 1){
            p.x = -1 * tags.transforms[i].transform.translation.x +tx;
            p.y = -1 * tags.transforms[i].transform.translation.z +ty;
            p.z = tags.transforms[i].transform.translation.y + 1.5;}
        else if (r == 2){
            p.x = tags.transforms[i].transform.translation.z +tx;
            p.y = -1 * tags.transforms[i].transform.translation.x +ty;
            p.z = tags.transforms[i].transform.translation.y + 1.5;}
	
        marker.points.push_back(p);
    }
    marker_pub.publish(marker);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "apriltag_localization");
    AprilTagLocalization apriltag_localization;
    ros::spin();
}
