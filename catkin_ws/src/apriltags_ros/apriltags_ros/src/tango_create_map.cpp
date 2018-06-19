#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>
#include <tf2_msgs/TFMessage.h>

class ApriltagPosition{
public:
	ApriltagPosition(){x=0, y=0, z=0, n=0; qx=0; qy=0; qz=0; qw=0;}
	float x, y, z;
	float qx,qy,qz,qw;
	int n;
};


class TangoMapCreation{
public:
    TangoMapCreation();
    ApriltagPosition* apriltagposition;

private:

    
    ros::NodeHandle nh;
    ros::Publisher path_vis_publisher;
    ros::Publisher apriltag_vis_publisher;
    ros::Subscriber tango_pose_subscriber;
    ros::Subscriber apriltag_pose_subscriber;

    visualization_msgs::Marker path_list;
    visualization_msgs::Marker apriltag_list;
    geometry_msgs::PoseStamped last_pose;
    void marker_setup();
    void tangoCallback(const geometry_msgs::PoseStamped& pose);
    void apriltagCallback(const apriltags_ros::AprilTagDetectionArray& tags);
    //void apriltagCallback(const tf2_msgs::TFMessage& tags);
    void draw_tango_map(geometry_msgs::PoseStamped recent_pose);   
};

TangoMapCreation::TangoMapCreation()
{
    path_vis_publisher = nh.advertise<visualization_msgs::Marker>("tango_path", 0);
    apriltag_vis_publisher = nh.advertise<visualization_msgs::Marker>("apriltag_position", 0);
    tango_pose_subscriber = nh.subscribe("/tango_sensors/tango_pose", 1, &TangoMapCreation::tangoCallback, this);
    apriltag_pose_subscriber = nh.subscribe("/tag_detections", 1, &TangoMapCreation::apriltagCallback, this);
    marker_setup();
    apriltagposition = new ApriltagPosition[80];

}

void TangoMapCreation::marker_setup(){
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

    apriltag_list.header.stamp = ros::Time::now();
    apriltag_list.header.frame_id = "/map";
    apriltag_list.action = visualization_msgs::Marker::ADD;
    apriltag_list.id = 1;
    apriltag_list.type = visualization_msgs::Marker::POINTS;
    apriltag_list.scale.x = 0.1; 
    apriltag_list.scale.y = 0.1;
    apriltag_list.scale.z = 0.1;
    apriltag_list.color.a = 1.0;
    apriltag_list.color.g = 1.0;

}

void TangoMapCreation::tangoCallback(const geometry_msgs::PoseStamped& pose){
    geometry_msgs::PoseStamped recent_pose = pose;
    draw_tango_map(recent_pose);
    last_pose = recent_pose;

}

void TangoMapCreation::draw_tango_map(geometry_msgs::PoseStamped recent_pose){
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

void TangoMapCreation::apriltagCallback(const apriltags_ros::AprilTagDetectionArray& tags){
	
	//last_pose;
	Eigen::Quaterniond q, qt;
	Eigen::Matrix4d tf_tango, tf_tag, tf_rot, tf_apriltag;
    q.x() = last_pose.pose.orientation.x;
	q.y() = last_pose.pose.orientation.y;
	q.z() = last_pose.pose.orientation.z;
	q.w() = last_pose.pose.orientation.w;    // Half of the rotation angle must be specified, even IDK why
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    tf_tango << R(0,0), R(0,1), R(0,2), last_pose.pose.position.x,
                R(1,0), R(1,1), R(1,2), last_pose.pose.position.y,
                R(2,0), R(2,1), R(2,2), last_pose.pose.position.z,
                0, 0, 0, 1;
    //R = R.inverse().eval();
    

	for(int i=0; i<tags.detections.size(); i++){
		//a <<  tags.transforms[i].transform.translation.x, tags.transforms[i].transform.translation.y, tags.transforms[i].transform.translation.z;
    	
    	qt.x() = tags.detections[i].pose.pose.orientation.x;
	    qt.y() = tags.detections[i].pose.pose.orientation.y;
	    qt.z() = tags.detections[i].pose.pose.orientation.z;
	    qt.w() = tags.detections[i].pose.pose.orientation.w;
        Eigen::Matrix3d Rt = qt.normalized().toRotationMatrix();
        tf_tag << Rt(0,0), Rt(0,1), Rt(0,2), tags.detections[i].pose.pose.position.z,
                  Rt(1,0), Rt(1,1), Rt(1,2), tags.detections[i].pose.pose.position.x,
                  Rt(2,0), Rt(2,1), Rt(2,2), tags.detections[i].pose.pose.position.y,
                  0, 0, 0, 1;
        

    	tf_apriltag = tf_tango* tf_tag;
		geometry_msgs::Point p;
		//p.x = tags.detections[i].pose.pose.position.y + last_pose.pose.position.x;
		//p.y = - tags.detections[i].pose.pose.position.x + last_pose.pose.position.y;
		//p.z = tags.detections[i].pose.pose.position.z + last_pose.pose.position.z;
		p.x = tf_apriltag(0,3);
		p.y = tf_apriltag(1,3);
		p.z = tf_apriltag(2,3);
		Eigen::Matrix3f mat;
		mat << tf_apriltag(0,0), tf_apriltag(0,1), tf_apriltag(0,2),
		       tf_apriltag(1,0), tf_apriltag(1,1), tf_apriltag(1,2),
		       tf_apriltag(2,0), tf_apriltag(2,1), tf_apriltag(2,2);

  		Eigen::Quaternionf qq(mat);

		apriltag_list.points.push_back(p);
		for (int j=0; j<80; j++){

			if (tags.detections[i].id == (100-j)){
				apriltagposition[j].n += 1;
				apriltagposition[j].x += p.x;
				apriltagposition[j].y += p.y;
				apriltagposition[j].z += p.z;
				apriltagposition[j].qx = qq.x();
				apriltagposition[j].qy = qq.y();
				apriltagposition[j].qz = qq.z();
				apriltagposition[j].qw = qq.w();
			} 
		}
		//std::cout<<tags.transforms[i].header.stamp<<std::endl;
		//std::cout<<last_pose.header.stamp<<"\n"<<std::endl;
	}
	apriltag_vis_publisher.publish(apriltag_list);
}

int   main (int argc, char** argv)
{
    ros::init(argc, argv, "tango_map_creation");
    TangoMapCreation tango_map_creation;
    while (ros::ok()){
    	ros::spin();
    }
    for(int i=0; i<80; i++){
    	if(tango_map_creation.apriltagposition[i].n != 0){
    		tango_map_creation.apriltagposition[i].x /= (float)tango_map_creation.apriltagposition[i].n;
    		tango_map_creation.apriltagposition[i].y /= (float)tango_map_creation.apriltagposition[i].n;
    		tango_map_creation.apriltagposition[i].z /= (float)tango_map_creation.apriltagposition[i].n;
    		tango_map_creation.apriltagposition[i].qx /= (float)tango_map_creation.apriltagposition[i].n;
    		tango_map_creation.apriltagposition[i].qy /= (float)tango_map_creation.apriltagposition[i].n;
    		tango_map_creation.apriltagposition[i].qz /= (float)tango_map_creation.apriltagposition[i].n;
    		tango_map_creation.apriltagposition[i].qw /= (float)tango_map_creation.apriltagposition[i].n;
    	}
        std::cout<<i<<" "<<tango_map_creation.apriltagposition[i].x<<" "<<tango_map_creation.apriltagposition[i].y<<" "<<tango_map_creation.apriltagposition[i].z<<" "<<tango_map_creation.apriltagposition[i].qx<<" "<<tango_map_creation.apriltagposition[i].qy<<" "<<tango_map_creation.apriltagposition[i].qz<<" "<<tango_map_creation.apriltagposition[i].qw<<std::endl;
    }
}

