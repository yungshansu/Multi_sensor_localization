#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <math.h>
using namespace Eigen; 





class Tango_imu_fuse{
  public:
    Tango_imu_fuse();
  private:
    double time_difference(ros::Time now, ros::Time last_time);
    void  path_vis (MatrixXf current_pos);
    void  imu_cb (const sensor_msgs::Imu::ConstPtr& input);
    void  tango_cb (const geometry_msgs::PoseStamped& pose);
    ros::Publisher path_vis_publisher;
    ros::Subscriber model_subscriber;
    ros::Subscriber tango_pose_subscriber ;
    ros::Time current_timestamp;
    ros::Time last_timestamp;
    MatrixXf last_position;
    MatrixXf position;
    sensor_msgs::Imu::ConstPtr initial_measure;
    geometry_msgs::PoseStamped tango_last_pose ;
    geometry_msgs::PoseStamped tango_recent_pose ;
    MatrixXf velocity;
    Matrix3f angular_ac = Matrix3f::Identity(); 
    Matrix3f I = Matrix3f::Identity();
    MatrixXf initial_linear_ac; 
    MatrixXf linear_ac;  
    MatrixXf rectified_linear_ac;  
    visualization_msgs::Marker line_list;
};



Tango_imu_fuse::Tango_imu_fuse(){
    ros::NodeHandle nh;   
    last_position = position = velocity = initial_linear_ac = linear_ac = MatrixXf::Zero(3,1); 
    angular_ac = Matrix3f::Identity();
    I = Matrix3f::Identity();

    initial_measure = ros::topic::waitForMessage<sensor_msgs::Imu>("/imu/data", ros::Duration(10));
    geometry_msgs::PoseStamped::ConstPtr reg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/tango_sensors/tango_pose", ros::Duration(10));
    (tango_last_pose) = (*reg);
    last_timestamp = initial_measure->header.stamp;
    position(0,0)=0; position(1,0)=0; position(2,0)=0;
    velocity(0,0)=0; velocity(1,0)=0; velocity(2,0)=0;
    initial_linear_ac(0,0) = initial_measure->linear_acceleration.x; initial_linear_ac(1,0) = initial_measure->linear_acceleration.y; initial_linear_ac(2,0) = initial_measure->linear_acceleration.z;
    // Create a ROS subscriber for the input point cloud
    model_subscriber = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 10, &Tango_imu_fuse::imu_cb,this);
    tango_pose_subscriber = nh.subscribe("/tango_sensors/tango_pose", 1, &Tango_imu_fuse::tango_cb, this);
    
    path_vis_publisher = nh.advertise<visualization_msgs::Marker>("/map/imu_path", 10);
    //Set up line list
    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.scale.x = 0.01; 
    line_list.scale.y = 0.01;
    line_list.color.r=1.0;
    line_list.color.a=1.0;
}


void  Tango_imu_fuse::imu_cb (const sensor_msgs::Imu::ConstPtr& input)
{
  printf("IMU START\n");
	double time_diff ;
  double theta;
  Matrix3f B;

  current_timestamp = input->header.stamp;
  time_diff = time_difference(current_timestamp,last_timestamp );
  //printf("Time:%lf,Augular Velocity:%lf,%lf,%lf Linear acceleration: %lf %lf %lf",time_diff,input->angular_velocity.x,input->angular_velocity.y,input->angular_velocity.z,input->linear_acceleration.x,input->linear_acceleration.y,input->linear_acceleration.z);
  if(time_diff>0.002){
    time_diff=0.002;
    printf("Strange");
  }
  //Renew orientation
  B(0,0)=0; B(0,1) = -1.0*time_diff * input->angular_velocity.z; B(0,2) = 1.0*time_diff * input->angular_velocity.y;
  B(1,0)=1.0*time_diff * input->angular_velocity.z; B(1,1) = 0; B(1,2) = -1.0*time_diff * input->angular_velocity.x;
  B(2,0)=-1.0*time_diff * input->angular_velocity.y; B(2,1) = 1.0*time_diff * input->angular_velocity.x; B(2,2) = 0;
  //printf("Finish B\n");
  theta = sqrt( (input->angular_velocity.x)*(input->angular_velocity.x) + (input->angular_velocity.y)*(input->angular_velocity.y) + (input->angular_velocity.z)*(input->angular_velocity.z))*time_diff;
  //printf("sine(theta): %lf,cos(theta):%lf",sin(theta),cos(theta));
  angular_ac = angular_ac * (I + sin(theta)/theta * B + (1-cos(theta))/(theta*theta) * B *B);
  linear_ac(0,0)= input->linear_acceleration.x;linear_ac(1,0)= input->linear_acceleration.y;linear_ac(2,0)= input->linear_acceleration.z;
  linear_ac =  angular_ac * linear_ac;
  //printf("Linear ac : %lf %lf %lf\n",linear_ac(0,0),linear_ac(1,0),linear_ac(2,0));
  //printf("Finish linear_ac\n");
  rectified_linear_ac = linear_ac - initial_linear_ac;
  velocity = velocity + time_diff * rectified_linear_ac ;
  position = position + time_diff * velocity;
  printf("rectified_linear_ac:%lf %lf %lf\n",rectified_linear_ac(0,0),rectified_linear_ac(1,0),rectified_linear_ac(2,0));
  printf("Position:%lf %lf %lf\n",position(0,0),position(1,0),position(2,0));
  //printf("Finish position\n");
  path_vis(position);
   printf("IMU Finish\n");
}
void  Tango_imu_fuse::path_vis (MatrixXf path){
    
    
    geometry_msgs::Point p1; 
    p1.x = path(0,0);
    p1.y = path(1,0);
    p1.z = path(2,0);
    if(p1.x<200 && p1.x>-200 && p1.y<200 && p1.y>-200 && p1.z<200 && p1.z>-200)
      line_list.points.push_back(p1);
    last_timestamp = current_timestamp;
    path_vis_publisher.publish(line_list);
  

}

double Tango_imu_fuse::time_difference(ros::Time now, ros::Time last_time){
    double difference =0;
    difference = now.sec-last_time.sec + (now.nsec-last_time.nsec)/1000000000.0;

    return difference;
}


void Tango_imu_fuse::tango_cb(const geometry_msgs::PoseStamped& pose){
    printf("Tango start\n");
    tango_recent_pose = pose;
    double delta_t = time_difference(tango_recent_pose.header.stamp,tango_last_pose.header.stamp);
    position(0,0) =  tango_recent_pose.pose.position.x; ;position(1,0) = tango_recent_pose.pose.position.y ;position(2,0) = tango_recent_pose.pose.position.z;
    path_vis(position);
    velocity(0,0) =  (tango_recent_pose.pose.position.x - tango_last_pose.pose.position.x)/delta_t;
    velocity(1,0) =  (tango_recent_pose.pose.position.y - tango_last_pose.pose.position.y)/delta_t;
    velocity(2,0) =  (tango_recent_pose.pose.position.z - tango_last_pose.pose.position.z)/delta_t;
    tango_last_pose = tango_recent_pose;
  printf("Tango finish\n");
} 
int   main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "Tango_imu_fuse");
     Tango_imu_fuse tango_imu_fuse;
     // Spin
     ros::spin ();
}

