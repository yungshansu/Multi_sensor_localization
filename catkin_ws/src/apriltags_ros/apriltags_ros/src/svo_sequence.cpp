#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <rosgraph_msgs/Log.h>
#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <string>
#include <math.h>
using namespace Eigen; 
using namespace std; 
class Svo_sequence{
  public:
    Svo_sequence();
  private:
    MatrixXf start_pose;//(3,1);
    MatrixXf raw_pose;//(3,1);
    MatrixXf rectified_pose;//(3,1);
    geometry_msgs::PoseStamped pose_out;
    visualization_msgs::Marker path_list;
    ros::Subscriber vo_pose_sub;
    ros::Subscriber rosout_sub;
    ros::Publisher  rectified_pose_pub;
    ros::Publisher  path_vis_publisher ;

    void  pose_cb (const geometry_msgs::PoseStamped::ConstPtr& input);
    void  rosout_cb(const rosgraph_msgs::Log::ConstPtr& input);
    void  path_vis (geometry_msgs::Pose);
    MatrixXf pose_to_transform_matrix(const geometry_msgs::PoseStamped::ConstPtr& input);
    geometry_msgs::Pose transform_matrix_to_pose(MatrixXf tf_matrix);
    //Eigen::Quaterniond q_init;
    //double yaw_init, pitch_init, roll_init;
};

Svo_sequence::Svo_sequence(){
    
    ros::NodeHandle nh;   
    start_pose = raw_pose = rectified_pose = MatrixXf::Identity(4,4);
    vo_pose_sub = nh.subscribe<geometry_msgs::PoseStamped> ("/svo/pose_cam/0", 10, &Svo_sequence::pose_cb,this);
    rosout_sub = nh.subscribe<rosgraph_msgs::Log> ("/rosout",10,&Svo_sequence::rosout_cb,this);
    rectified_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/svo/rectified_pose_cam", 10);
    path_vis_publisher = nh.advertise<visualization_msgs::Marker>("/svo_path", 10);
    //Initial marker
    path_list.header.frame_id = "world";
    path_list.header.stamp = ros::Time::now();
    path_list.action = visualization_msgs::Marker::ADD;
    path_list.pose.orientation.w = 1.0;
    path_list.id = 2;
    path_list.type = visualization_msgs::Marker::LINE_STRIP;
    path_list.scale.x = 0.03; 
    path_list.scale.y = 0.03;
    path_list.color.r=1.0;
    path_list.color.a=1.0;
    printf("Finish initialization\n");
}
void Svo_sequence::pose_cb (const geometry_msgs::PoseStamped::ConstPtr& input){
    raw_pose = pose_to_transform_matrix(input);
    rectified_pose = start_pose * raw_pose;
    //Transform to ros pose
    pose_out.header = input->header;
    pose_out.header.frame_id = "world";
    pose_out.pose = transform_matrix_to_pose(rectified_pose);
    //Publish
    rectified_pose_pub.publish(pose_out);
    path_vis(pose_out.pose);
}

void  Svo_sequence::rosout_cb(const rosgraph_msgs::Log::ConstPtr& input){
    string reset_string = input->msg;
    if (reset_string == "DepthFilter: RESET.")
       start_pose = rectified_pose;
    printf("DepthFilter: RESET\n");
    return;
}

MatrixXf Svo_sequence::pose_to_transform_matrix(const geometry_msgs::PoseStamped::ConstPtr& input){
    Eigen::Quaterniond q;
    q.x() = input->pose.orientation.x;
    q.y() = input->pose.orientation.y;
    q.z() = input->pose.orientation.z;
    q.w() = input->pose.orientation.w;
    Eigen::Matrix3d rot = q.toRotationMatrix();
    Vector3d translation(input->pose.position.x, input->pose.position.y, input->pose.position.z);
    MatrixXf tf_matrix = MatrixXf::Identity(4,4); 
    for (int row=0; row<3; row++){
      for (int column=0; column<3; column++){
        tf_matrix(row,column) = rot(row,column);
      }
      tf_matrix(row,3) = translation(row);
    }
    return tf_matrix;
}

geometry_msgs::Pose Svo_sequence::transform_matrix_to_pose(MatrixXf tf_matrix){
    //Assign transition
    geometry_msgs::Pose pose_trans;
    pose_trans.position.x = tf_matrix(0,3);
    pose_trans.position.y = tf_matrix(1,3);
    pose_trans.position.z = tf_matrix(2,3);
    //Assign transition
    Eigen::Matrix3d rot;
    for (int row=0; row<3; row++){
        for (int column=0; column<3; column++){
            rot(row,column) = tf_matrix(row,column);
        }
    }
    Eigen::Quaterniond q(rot);
    pose_trans.orientation.x = q.x(); 
    pose_trans.orientation.y = q.y(); 
    pose_trans.orientation.z = q.z(); 
    pose_trans.orientation.w = q.w(); 
    return pose_trans;
}

void  Svo_sequence::path_vis (geometry_msgs::Pose input){      
    geometry_msgs::Point p1; 
    p1.x = input.position.x;
    p1.y = input.position.y;
    p1.z = input.position.z;
    path_list.points.push_back(p1);
    path_vis_publisher.publish(path_list);
}

int main (int argc, char** argv){
     ros::init (argc, argv, "svo_sequence");
     // Initialize ROS
     Svo_sequence svo_sequence;
     ros::spin ();
     return 0;
}

