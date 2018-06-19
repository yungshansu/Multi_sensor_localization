#define _GLIBCXX_USE_C99 1

#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

using namespace std;

class AprilTagLocalization{
public:
    AprilTagLocalization();
    geometry_msgs::Point *tag_pos;
    Eigen::Quaterniond *qq;
private:
    ros::NodeHandle nh;
    ros::Subscriber tag_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber odo_sub;
    ros::Publisher vis_pub;
    ros::Publisher marker_pub;
    ros::Publisher marker_pub2;
    ros::Publisher pose_pub;
    visualization_msgs::Marker marker;
    visualization_msgs::Marker map;
    visualization_msgs::Marker fusion_marker;
    void LocalCallback(const apriltags_ros::AprilTagDetectionArray& tags);
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu);
    void OdoCallback(const nav_msgs::Odometry::ConstPtr& odo);
    
    void assign_the_point();
    void draw_the_map();
    geometry_msgs::Point last_p;
    Eigen::Quaterniond q, qt;
    Eigen::Matrix4d tf_imu, tf_tag, tf_apriltag, tf_world;
     
};

void AprilTagLocalization::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu){
    q.x() = imu->orientation.x;
    q.y() = imu->orientation.y;
    q.z() = imu->orientation.z;
    q.w() = imu->orientation.w;
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    tf_imu << R(0,0), R(0,1), R(0,2), 0,
              R(1,0), R(1,1), R(1,2), 0,
              R(2,0), R(2,1), R(2,2), 0,
              0, 0, 0, 1;

}

AprilTagLocalization::AprilTagLocalization()
{
    assign_the_point();
    last_p.x =0;
    last_p.y =0;
    last_p.z =0;
    //draw_the_map();
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    marker_pub2 = nh.advertise<visualization_msgs::Marker>("visualization_fusion_marker", 0);
    vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_map", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("tag_pose", 1);
    tag_sub = nh.subscribe("/tag_detections", 1, &AprilTagLocalization::LocalCallback, this);
    imu_sub = nh.subscribe("/imu", 1, &AprilTagLocalization::ImuCallback, this);
    odo_sub = nh.subscribe("/odometry/filtered", 1, &AprilTagLocalization::OdoCallback, this);
    //draw_the_map();
}
void AprilTagLocalization::assign_the_point()
{
    tag_pos = new geometry_msgs::Point[72];
    qq = new Eigen::Quaterniond[72];
    tag_pos[0].x=3.35002 ;tag_pos[0].y=-0.0845511 ;tag_pos[0].z=-0.561389;
    tag_pos[1].x=7.90899 ;tag_pos[1].y=3.28763 ;tag_pos[1].z=-0.478181;
    tag_pos[2].x=12.0536 ;tag_pos[2].y=2.90333 ;tag_pos[2].z=-0.389003;
    tag_pos[3].x=16.1519 ;tag_pos[3].y=2.48528 ;tag_pos[3].z=-0.392722;
    tag_pos[4].x=20.2125 ;tag_pos[4].y=2.12903 ;tag_pos[4].z=-0.478796;
    tag_pos[5].x=24.6012 ;tag_pos[5].y=1.74879 ;tag_pos[5].z=-0.56725;
    tag_pos[6].x=28.7158 ;tag_pos[6].y=1.30843 ;tag_pos[6].z=-0.742378;
    tag_pos[7].x=32.8887 ;tag_pos[7].y=0.963858 ;tag_pos[7].z=-0.806969;
    tag_pos[8].x=37.2323 ;tag_pos[8].y=0.617934 ;tag_pos[8].z=-0.720548;
    tag_pos[9].x=41.4877 ;tag_pos[9].y=0.239182 ;tag_pos[9].z=-0.689611;
    tag_pos[10].x=45.5162 ;tag_pos[10].y=-0.207529 ;tag_pos[10].z=-0.825037;
    tag_pos[11].x=49.6288 ;tag_pos[11].y=-0.52882 ;tag_pos[11].z=-0.716211;
    tag_pos[12].x=53.8557 ;tag_pos[12].y=-1.0289 ;tag_pos[12].z=-0.637238;
    tag_pos[13].x=58.1664 ;tag_pos[13].y=-1.18984 ;tag_pos[13].z=-0.653709;
    tag_pos[14].x=58.5862 ;tag_pos[14].y=2.67265 ;tag_pos[14].z=-0.531835;
    tag_pos[15].x=59.0829 ;tag_pos[15].y=6.57189 ;tag_pos[15].z=-0.716896;
    tag_pos[16].x=59.6192 ;tag_pos[16].y=10.6813 ;tag_pos[16].z=-0.516279;
    tag_pos[17].x=60.0295 ;tag_pos[17].y=14.9305 ;tag_pos[17].z=-0.445291;
    tag_pos[18].x=60.5769 ;tag_pos[18].y=18.9783 ;tag_pos[18].z=-0.578577;
    tag_pos[19].x=61.012 ;tag_pos[19].y=23.1239 ;tag_pos[19].z=-0.43713;
    tag_pos[20].x=61.5435 ;tag_pos[20].y=27.1103 ;tag_pos[20].z=-0.48043;
    tag_pos[21].x=62.0175 ;tag_pos[21].y=31.0966 ;tag_pos[21].z=-0.523729;
    tag_pos[22].x=62.5248 ;tag_pos[22].y=35.0261 ;tag_pos[22].z=-0.503711;
    tag_pos[23].x=62.9388 ;tag_pos[23].y=39.124 ;tag_pos[23].z=-0.497665;
    tag_pos[24].x=63.4799 ;tag_pos[24].y=43.2816 ;tag_pos[24].z=-0.362474;
    tag_pos[25].x=63.9685 ;tag_pos[25].y=47.2593 ;tag_pos[25].z=-0.498761;
    tag_pos[26].x=0 ;tag_pos[26].y=0 ;tag_pos[26].z=0;
    tag_pos[27].x=66.1294 ;tag_pos[27].y=50.5636 ;tag_pos[27].z=-2.50713;
    tag_pos[28].x=68.2357 ;tag_pos[28].y=50.2449 ;tag_pos[28].z=-3.64201;
    tag_pos[29].x=68.5247 ;tag_pos[29].y=51.0242 ;tag_pos[29].z=-3.46485;
    tag_pos[30].x=66.6808 ;tag_pos[30].y=51.5424 ;tag_pos[30].z=-4.68517;
    tag_pos[31].x=64.6679 ;tag_pos[31].y=51.7755 ;tag_pos[31].z=-5.65791;
    tag_pos[32].x=60.7107 ;tag_pos[32].y=51.8399 ;tag_pos[32].z=-4.24214;
    tag_pos[33].x=56.4903 ;tag_pos[33].y=52.6675 ;tag_pos[33].z=-4.26397;
    tag_pos[34].x= 52.99315 ;tag_pos[34].y=53.270175 ;tag_pos[34].z=-4.26397;
    tag_pos[35].x= 49.496 ;tag_pos[35].y=53.87285 ;tag_pos[35].z=-4.26397;
    tag_pos[36].x= 45.99885 ;tag_pos[36].y=54.475525 ;tag_pos[36].z=-4.26397;
    tag_pos[37].x= 42.5017 ;tag_pos[37].y=55.0782 ;tag_pos[37].z=-6.33055;
    tag_pos[38].x= 39.5312 ;tag_pos[38].y=55.4633 ;tag_pos[38].z=-7.07513;
    tag_pos[39].x=37.968 ;tag_pos[39].y=55.6796 ;tag_pos[39].z=-7.85138;
    tag_pos[40].x= 34.8624 ;tag_pos[40].y= 56.0157 ;tag_pos[40].z=-8.5752;
    tag_pos[41].x= 32.1546 ;tag_pos[41].y= 56.3881 ;tag_pos[41].z=-9.78287;
    tag_pos[42].x= 28.8173 ;tag_pos[42].y=56.7285 ;tag_pos[42].z=-9.63666;
    tag_pos[43].x= 25.0446 ;tag_pos[43].y=56.5568 ;tag_pos[43].z=-8.38307;
    tag_pos[44].x= 20.74 ;tag_pos[44].y=57.3482 ;tag_pos[44].z=-9.66587;
    tag_pos[45].x= 16.9059 ;tag_pos[45].y=57.801 ;tag_pos[45].z=-8.50338;
    tag_pos[46].x= 12.518 ;tag_pos[46].y=58.3777 ;tag_pos[46].z=-9.64308;
    tag_pos[47].x= 9.10026 ;tag_pos[47].y=58.9867 ;tag_pos[47].z=-8.70798;
    tag_pos[48].x= 7.56055 ;tag_pos[48].y=62.2109 ;tag_pos[48].z=-8.97983;
    tag_pos[49].x= 6.33825 ;tag_pos[49].y=64.2522 ;tag_pos[49].z=-8.73962;
    tag_pos[50].x= 1.81078 ;tag_pos[50].y= 63.5652 ;tag_pos[50].z=-8.6603;
    tag_pos[51].x= -1.53373 ;tag_pos[51].y=62.4261 ;tag_pos[51].z=-8.74538;
    tag_pos[52].x= -6.83099 ;tag_pos[52].y=61.3881 ;tag_pos[52].z=-8.95391;
    tag_pos[53].x=-10.5694 ;tag_pos[53].y=58.9544 ;tag_pos[53].z=-8.9638;
    tag_pos[54].x= 0 ;tag_pos[54].y=0 ;tag_pos[54].z=0;
    tag_pos[55].x= -16.3048 ;tag_pos[55].y=54.769 ;tag_pos[55].z=-7.59288;
    tag_pos[56].x= -17.415 ;tag_pos[56].y=49.685 ;tag_pos[56].z=-7.35572;
    tag_pos[57].x= -18.7836 ;tag_pos[57].y=46.8272 ;tag_pos[57].z=-7.39146;
    tag_pos[58].x= -19.8148 ;tag_pos[58].y=44.2174 ;tag_pos[58].z=-6.23712;
    tag_pos[59].x= -21.0702 ;tag_pos[59].y=40.0925 ;tag_pos[59].z=-6.38548;
    tag_pos[60].x= -21.61 ;tag_pos[60].y=36.6579; tag_pos[60].z=-5.78026;
    tag_pos[61].x= -21.688 ;tag_pos[61].y=32.1102 ; tag_pos[61].z=-5.79329;
    tag_pos[62].x= -21.1039 ;tag_pos[62].y=28.5554 ; tag_pos[62].z=-4.70017;
    tag_pos[63].x= -21.3107 ;tag_pos[63].y=23.3908 ; tag_pos[63].z=-4.04827;
    tag_pos[64].x= -20.9436 ;tag_pos[64].y=19.7301 ; tag_pos[64].z=-4.13143;
    tag_pos[65].x= -18.9204 ;tag_pos[65].y=15.9061 ; tag_pos[65].z=-2.84531;
    tag_pos[66].x= -18.0734 ;tag_pos[66].y=14.2446 ; tag_pos[66].z=-2.66632;
    tag_pos[67].x= -16.1516 ;tag_pos[67].y=11.7722 ; tag_pos[67].z=-2.3659;
    tag_pos[68].x= -14.1782 ;tag_pos[68].y=9.77703 ; tag_pos[68].z=-1.20819;
    tag_pos[69].x= -11.6385 ;tag_pos[69].y=7.90546 ; tag_pos[69].z=-2.40664;
    tag_pos[70].x= -7.72354 ;tag_pos[70].y=5.54511 ; tag_pos[70].z=-1.01647;
    tag_pos[71].x=0.166575 ;tag_pos[71].y=2.23877 ; tag_pos[71].z=-1.07611;
    qq[0].x() =-0.0168879;qq[0].y() = -0.00659387 ;qq[0].z()=0.0119609 ;qq[0].w()=0.123099;
    qq[1].x() =0.0016516;qq[1].y() = -0.0136105 ;qq[1].z()=0.0233024 ;qq[1].w()=0.248534;
    qq[2].x() =-0.00149347;qq[2].y() = 0.00102314 ;qq[2].z()=-0.00137497 ;qq[2].w()=0.0713924;
    qq[3].x() =-0.00189148;qq[3].y() = 0.00579138 ;qq[3].z()=-0.00173228 ;qq[3].w()=0.0830923;
    qq[4].x() =0.00633831;qq[4].y() =0.0137335;qq[4].z() = -0.00600229 ;qq[4].w()=0.109913 ;
    qq[5].x() =0.00169633;qq[5].y() = 0.0076315 ;qq[5].z()=-0.00320221 ;qq[5].w()=0.110789;
    qq[6].x() =-0.0019947;qq[6].y() = 0.0148623 ;qq[6].z()=-0.00518106 ;qq[6].w()=0.0987334;
    qq[7].x() =0.00204344;qq[7].y() = 0.0121528 ;qq[7].z()=-0.00684719 ;qq[7].w()=0.0898093;
    qq[8].x() =0.00204696;qq[8].y() = 0.00830549 ;qq[8].z()=-0.00706296 ;qq[8].w()=0.124507;
    qq[9].x() =-0.000522861;qq[9].y() = 0.00556963 ;qq[9].z()=-0.00299018 ;qq[9].w()=0.0906875;
    qq[10].x() =0.0163181;qq[10].y() = 0.0225944 ;qq[10].z()=-0.0330409 ;qq[10].w()=0.498128;
    qq[11].x() =-0.0200316;qq[11].y() = 0.0116636 ;qq[11].z()=-0.00533509 ;qq[11].w()=0.248866;
    qq[12].x() =-0.00733091 ;qq[12].y() =0.00543561 ;qq[12].z()=-0.00468247 ;qq[12].w()=0.166351;
    qq[13].x() =0.0112392 ;qq[13].y() =-0.0221911 ;qq[13].z()=0.0412021 ;qq[13].w()=0.159566;
    qq[14].x() =-0.0119972;qq[14].y() = 0.00989258 ;qq[14].z()=0.0626874 ;qq[14].w()=0.0763446;
    qq[15].x() =0.00400027;qq[15].y() = 0.00247632 ;qq[15].z()=0.0771261 ;qq[15].w()=0.0798443;
    qq[16].x() =-0.00758331;qq[16].y() = 0.010372 ;qq[16].z()=0.0677941 ;qq[16].w()=0.0723801;
    qq[17].x() =-0.00428399;qq[17].y() = 0.00918181 ;qq[17].z()=0.0870386 ;qq[17].w()=0.0891439;
    qq[18].x() =-0.00390444;qq[18].y() = 0.0112464 ;qq[18].z()=0.113545 ;qq[18].w()=0.121423;
    qq[19].x() =-0.0025106;qq[19].y() = 0.00826176 ;qq[19].z()=0.0873365 ;qq[19].w()=0.0890099;
    qq[20].x() =-0.0025106;qq[20].y() = 0.00826176 ;qq[20].z()=0.0873365 ;qq[20].w()=0.0890099;
    qq[21].x() =-0.0102185;qq[21].y()= 0.0176614 ;qq[21].z()=0.142403 ;qq[21].w()=0.138943;
    qq[22].x() =-0.00703642;qq[22].y()= 0.00552301 ;qq[22].z()=0.0762874 ;qq[22].w()=0.0802864;
    qq[23].x() =-0.00750093;qq[23].y()= 0.0112468 ;qq[23].z()=0.073503 ;qq[23].w()=0.0822207;
    qq[24].x() =-0.0349368;qq[24].y()= 0.0365863 ;qq[24].z()=0.68509 ;qq[24].w()=0.7267;
    qq[25].x() =-0.00759062;qq[25].y()= 0.0163089 ;qq[25].z()=0.0855554 ;qq[25].w()=0.0893402;
    qq[26].x() =0;qq[26].y()= 0 ;qq[26].z()=0 ;qq[26].w()=1;
    qq[27].x() =-0.213362;qq[27].y() =0.433806 ;qq[27].z()=0.13226 ;qq[27].w()=0.86533;
    qq[28].x() =-0.048185;qq[28].y()= 0.0481911 ;qq[28].z()=0.0135928 ;qq[28].w()=0.103904;
    qq[29].x() =-0.19037;qq[29].y()= 0.0775728 ;qq[29].z()=0.246509 ;qq[29].w()=-0.0899229;
    qq[30].x() =-0.116329;qq[30].y()= -0.068285 ;qq[30].z()=0.208783 ;qq[30].w()=-0.0267294;
    qq[31].x() =-0.08254;qq[31].y()= -0.0910014 ;qq[31].z()=0.21613 ;qq[31].w()=-0.0263371;
    qq[32].x() =-0.0424789 ;qq[32].y()=-0.00327065 ;qq[32].z()=0.498142 ;qq[32].w()=0.00629197;
    qq[33].x() =-0.112665;qq[33].y()= -0.0192295 ;qq[33].z()=0.991108 ;qq[33].w()=0.0681312;
    qq[34].x() =-0.112665;qq[34].y()= -0.0192295 ;qq[34].z()=0.991108 ;qq[34].w()=0.0681312;
    qq[35].x() =-0.112665;qq[35].y()= -0.0192295 ;qq[35].z()=0.991108 ;qq[35].w()=0.0681312;
    qq[36].x() =-0.112665;qq[36].y()= -0.0192295 ;qq[36].z()=0.991108 ;qq[36].w()=0.0681312;
    qq[37].x() =-0.0973602;qq[37].y()= -0.0521341 ;qq[37].z()=0.164636 ;qq[37].w()=-0.0264172;
    qq[38].x() =-0.0463653;qq[38].y()= -0.040575 ;qq[38].z()=0.0908199 ;qq[38].w()=-0.0173598;
    qq[39].x() =-0.0456666;qq[39].y()= -0.0259764 ;qq[39].z()=0.0841982 ;qq[39].w()=-0.0122655;
    qq[40].x() =-0.0416591;qq[40].y()= -0.0254622 ;qq[40].z()=0.0755087 ;qq[40].w()=-0.0133824;
    qq[41].x() =-0.0420674;qq[41].y()= -0.02824 ;qq[41].z()=0.0854193 ;qq[41].w()=-0.0116781;
    qq[42].x() =-0.0739722 ;qq[42].y()=-0.065084 ;qq[42].z()=0.172269 ;qq[42].w()=-0.0248127;
    qq[43].x() =-0.0591693 ;qq[43].y()=0.00534909 ;qq[43].z()=0.242831 ;qq[43].w()=-0.00191228;
    qq[44].x() =-0.117978 ;qq[44].y()=-0.236394 ;qq[44].z()=0.418865 ;qq[44].w()=-0.0689329;
    qq[45].x() =-0.171848 ;qq[45].y()=0.00392328 ;qq[45].z()=0.285226 ;qq[45].w()=0.0144912;
    qq[46].x() =-0.10433 ;qq[46].y()=-0.242709 ;qq[46].z()=0.419162 ;qq[46].w()=-0.0671605;
    qq[47].x() =-0.0541179 ;qq[47].y()=0.0137644 ;qq[47].z()=0.188284 ;qq[47].w()=0.0378256;
    qq[48].x() =-0.101028 ;qq[48].y()=0.061386 ;qq[48].z()=0.194656 ;qq[48].w()=0.103123;
    qq[49].x() =-0.0735841 ;qq[49].y()=-0.0273209 ;qq[49].z()=0.132098 ;qq[49].w()=-0.0645503;
    qq[50].x() =-0.0767197 ;qq[50].y()=-0.0457482 ;qq[50].z()=0.149491 ;qq[50].w()=-0.0983544;
    qq[51].x() =0.0308859 ;qq[51].y()=0.0257237 ;qq[51].z()=-0.0863081 ;qq[51].w()=0.0809954;
    qq[52].x() =0.000982554 ;qq[52].y()=0.000578057 ;qq[52].z()=0.0532628 ;qq[52].w()=-0.0249395;
    qq[53].x() =0.0129356 ;qq[53].y()=-0.136131 ;qq[53].z()=0.182253 ;qq[53].w()=-0.102883;
    qq[54].x() =0 ;qq[54].y()=0 ;qq[54].z()=0 ;qq[54].w()=1;
    qq[55].x() =-0.029574 ;qq[55].y()=-0.0117924 ;qq[55].z()=0.0825834 ;qq[55].w()=-0.0465436;
    qq[56].x() =-0.0487805 ;qq[56].y()=0.0729804 ;qq[56].z()=-0.105683 ;qq[56].w()=0.0943571;
    qq[57].x() =-0.0704238 ;qq[57].y()=0.13314 ;qq[57].z()=-0.211207 ;qq[57].w()=0.209324;
    qq[58].x() =-0.0871115 ;qq[58].y()=0.0702375 ;qq[58].z()=-0.10635 ;qq[58].w()=0.127153;
    qq[59].x() =-0.127361 ;qq[59].y()=0.138241 ;qq[59].z()=-0.177073 ;qq[59].w()=0.210773;
    qq[60].x() =-0.119582 ;qq[60].y()=0.137536 ;qq[60].z()=-0.163641 ;qq[60].w()=0.22609;
    qq[61].x() =-0.134068 ;qq[61].y()=0.139027 ;qq[61].z()=-0.147497 ;qq[61].w()=0.228151;
    qq[62].x() =-0.0734322 ;qq[62].y()=0.0750416 ;qq[62].z()=-0.0738492 ;qq[62].w()=0.153371;
    qq[63].x() =0.0215849 ;qq[63].y()=0.00907638 ;qq[63].z()=-0.0621963 ;qq[63].w()=0.0747217;
    qq[64].x() =-0.264693 ;qq[64].y()=0.51157 ;qq[64].z()=-0.252956 ;qq[64].w()=0.777333;
    qq[65].x() =0.0146826 ;qq[65].y()=0.00722242 ;qq[65].z()=-0.0563038 ;qq[65].w()=0.156013;
    qq[66].x() =0.0415727 ;qq[66].y()=0.182073 ;qq[66].z()=-0.0305698 ;qq[66].w()=0.274404;
    qq[67].x() =-0.00313287 ;qq[67].y()=0.0478938 ;qq[67].z()=0.00252657 ;qq[67].w()=0.100178;
    qq[68].x() =-0.00828831 ;qq[68].y()=0.0388403 ;qq[68].z()=0.0171423 ;qq[68].w()=0.102345;
    qq[69].x() =-0.112849 ;qq[69].y()=0.152399 ;qq[69].z()=0.0364195 ;qq[69].w()=0.271706;
    qq[70].x() =0.00934457 ;qq[70].y()=0.0166514 ;qq[70].z()=-0.0163571 ;qq[70].w()=0.122445;
    qq[71].x() =-0.00689639 ;qq[71].y()=0.0300014 ;qq[71].z()=-0.0190214 ;qq[71].w()=0.162691;
}

void AprilTagLocalization::draw_the_map()
{
   // visualization_msgs::Marker map;
    map.header.frame_id = "/map";
    map.header.stamp = ros::Time();
    map.type = visualization_msgs::Marker::POINTS;
    map.action = visualization_msgs::Marker::ADD;
    map.id = 3;
    map.scale.x = 0.5;
    map.scale.y = 0.5;
    map.scale.z = 0.5;
    map.color.a = 1.0;
    map.color.r = 1.0;
    map.color.g = 1.0;
    map.color.b = 1.0;
    for(int i =0; i< 72; i++){
        if(i == 26 || i == 54){
            continue;
        }
        map.points.push_back(tag_pos[i]);
    }
   // geometry_msgs::Point tag_1, tag_2, tag_3, tag_4, tag_5, tag_6, tag_7, tag_8, tag_9;

    /*tag_1.x = 0.0;tag_1.y = 0.0;tag_1.z = 1.5;
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
    map.points.push_back(tag_17);*/

    //while (ros::ok()){
    vis_pub.publish(map);
    //}
}

void AprilTagLocalization::LocalCallback(const apriltags_ros::AprilTagDetectionArray& tags)
{
    draw_the_map();
    //AprilTagDetection tag_detection
    //visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::LINE_LIST;
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
//tags.detections.size()
    geometry_msgs::Point p;
    int max = 0;
    int k = 100;
    for(int i=0; i<tags.detections.size(); i++){

        if(tags.detections[i].id > max){
            max = tags.detections[i].id;
            k = i;
        }
    }
    if(k == 100||max == 74||max == 46)
        return;
    qt.x() = tags.detections[k].pose.pose.orientation.x;
    qt.y() = tags.detections[k].pose.pose.orientation.y;
    qt.z() = tags.detections[k].pose.pose.orientation.z;
    qt.w() = tags.detections[k].pose.pose.orientation.w;
    Eigen::Matrix3d Rt = qt.normalized().toRotationMatrix();
    tf_tag << Rt(0,0), Rt(0,1), Rt(0,2), -tags.detections[k].pose.pose.position.z,
              Rt(1,0), Rt(1,1), Rt(1,2), tags.detections[k].pose.pose.position.x,
              Rt(2,0), Rt(2,1), Rt(2,2), tags.detections[k].pose.pose.position.y,
              0, 0, 0, 1;
        
    for(int j=0; j<72; j++){
        if(tags.detections[k].id == (100-j)){
            Eigen::Matrix3d Rq = qq[j].normalized().toRotationMatrix();
            tf_apriltag << Rq(0,0), Rq(0,1), Rq(0,2), tag_pos[j].x,
                           Rq(1,0), Rq(1,1), Rq(1,2), tag_pos[j].y,
                           Rq(2,0), Rq(2,1), Rq(2,2), tag_pos[j].z,
                           0, 0, 0, 1;
            tf_apriltag.inverse().eval();
            //tf_tag.inverse();

            tf_world = tf_apriltag* tf_tag;

            p.x = tf_world(0,3);
            p.y = tf_world(1,3);
            p.z = tf_world(2,3);
        }
    }
      /*  if(tags.transforms[i].child_frame_id == "tag_1") {
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
            p.z = tags.transforms[i].transform.translation.y + 1.5;}*/
	
    marker.points.push_back(last_p);
    marker.points.push_back(p);
    marker_pub.publish(marker);
    last_p = p;

    geometry_msgs::PoseWithCovarianceStamped mypose;
    mypose.header.stamp =  ros::Time::now();
    mypose.header.frame_id = "map";
    mypose.pose.pose.position.x = p.x;
    mypose.pose.pose.position.y = p.y;
    mypose.pose.pose.position.z = p.z;
    Eigen::Matrix3d rot;
    for (int row=0;row<3;row++)
        for(int column=0;column<3;column++)
            rot(row,column) = tf_world(row,column);
    Eigen::Quaterniond q(rot);
    mypose.pose.pose.orientation.x = q.x(); 
    mypose.pose.pose.orientation.y = q.y(); 
    mypose.pose.pose.orientation.z = q.z(); 
    mypose.pose.pose.orientation.w = q.w(); 
    mypose.pose.covariance[0] = 0.005;
    mypose.pose.covariance[7] = 0.005;
    mypose.pose.covariance[14] = 0.005;
    mypose.pose.covariance[21] = 0.001;
    mypose.pose.covariance[28] = 0.001;
    mypose.pose.covariance[35] = 0.001;
    
    pose_pub.publish(mypose);
}

void AprilTagLocalization::OdoCallback(const nav_msgs::Odometry::ConstPtr& odo){
    fusion_marker.header.frame_id = "base_imu_link";
    fusion_marker.header.stamp = ros::Time();
    fusion_marker.type = visualization_msgs::Marker::LINE_STRIP;
    fusion_marker.action = visualization_msgs::Marker::ADD;
    fusion_marker.id = 1;
    fusion_marker.scale.x = 0.1;
    fusion_marker.scale.y = 0.1;
    fusion_marker.scale.z = 0.1;
    fusion_marker.color.a = 1.0;
    fusion_marker.color.r = 1.0;
    fusion_marker.color.g = 0.0;
    fusion_marker.color.b = 0.0;
    geometry_msgs::Point p;
    p.x =  odo->pose.pose.position.x;
    p.y =  odo->pose.pose.position.y;
    p.z =  odo->pose.pose.position.z;
    fusion_marker.points.push_back(p);
    marker_pub2.publish(fusion_marker);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "apriltag_localization");
    AprilTagLocalization apriltag_localization;
    ros::spin();
}
