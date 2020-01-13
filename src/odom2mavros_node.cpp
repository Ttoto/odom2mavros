#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "yamlRead.h"
#include "stdio.h"
#include <sstream>
#include "../3rdPartLib/Sophus/sophus/se3.h"
#include "../3rdPartLib/Sophus/sophus/so3.h"

using namespace std;
using namespace Sophus;

double time_gap;
bool vo_has_pose;
geometry_msgs::PoseStamped latest_pose;
ros::Time lastMsgTime;
bool first_msg;
ros::Publisher  mavros_pub;

SE3 T_imu_odom;
SE3 T_odom_imu;

void vo_callback(const nav_msgs::OdometryConstPtr msg)
{
    if(first_msg)
    {
        lastMsgTime = msg->header.stamp;
        latest_pose.header.frame_id = "map";
        latest_pose.header.stamp=msg->header.stamp;
        Quaterniond q=Quaterniond(msg->pose.pose.orientation.w,
                                  msg->pose.pose.orientation.x,
                                  msg->pose.pose.orientation.y,
                                  msg->pose.pose.orientation.z);
        Vec3 t = Vec3(msg->pose.pose.position.x,
                      msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
        SE3 T_w_odom = SE3(q,t);
        SE3 T_w_i=T_w_odom*T_odom_imu;
        Quaterniond q_w_i = T_w_i.unit_quaternion();
        Vec3 t_w_i = T_w_i.translation();
        latest_pose.pose.orientation.w = q_w_i.w();
        latest_pose.pose.orientation.x = q_w_i.x();
        latest_pose.pose.orientation.y = q_w_i.y();
        latest_pose.pose.orientation.z = q_w_i.z();
        latest_pose.pose.position.x = t_w_i(0);
        latest_pose.pose.position.y = t_w_i(1);
        latest_pose.pose.position.z = t_w_i(2);
        mavros_pub.publish(latest_pose);
        first_msg = false;
    }
    else
    {
        if((msg->header.stamp.toSec()-lastMsgTime.toSec())>time_gap)
        {
            lastMsgTime = msg->header.stamp;
            latest_pose.header.frame_id = "map";
            latest_pose.header.stamp=msg->header.stamp;
            Quaterniond q=Quaterniond(msg->pose.pose.orientation.w,
                                      msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y,
                                      msg->pose.pose.orientation.z);
            Vec3 t = Vec3(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z);
            SE3 T_w_odom = SE3(q,t);
            SE3 T_w_i=T_w_odom*T_odom_imu;
            Quaterniond q_w_i = T_w_i.unit_quaternion();
            Vec3 t_w_i = T_w_i.translation();
            latest_pose.pose.orientation.w = q_w_i.w();
            latest_pose.pose.orientation.x = q_w_i.x();
            latest_pose.pose.orientation.y = q_w_i.y();
            latest_pose.pose.orientation.z = q_w_i.z();
            latest_pose.pose.position.x = t_w_i(0);
            latest_pose.pose.position.y = t_w_i(1);
            latest_pose.pose.position.z = t_w_i(2);
            mavros_pub.publish(latest_pose);
        }
    }
    return;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "odom2mavros");
    ros::NodeHandle nh;
    string configFilePath;
    string mavros_topic;
    string odom_topic;
    int frequency=0;

    cout  <<"get parameter" << endl;
    nh.getParam("odom_topic", odom_topic);
    nh.getParam("mavros_topic", mavros_topic);
    nh.getParam("config_file_path", configFilePath);
    nh.getParam("pub_frequency", frequency);

    cout << odom_topic << endl;
    cout << mavros_topic << endl;
    cout << configFilePath << endl;
    cout << frequency << endl;

    Mat4x4 mat_imu_odom;//odom to imu frame
    mat_imu_odom = Mat44FromYaml(configFilePath,"T_imu_odom");


    cout << "Transformation From odom to imu is:" << endl << mat_imu_odom << endl;
    T_imu_odom = SE3(SO3(mat_imu_odom.topLeftCorner(3,3)),mat_imu_odom.topRightCorner(3,1));
    T_odom_imu = T_imu_odom.inverse();
    cout << mat_imu_odom.topLeftCorner(3,3) << endl;
    cout << mat_imu_odom.topRightCorner(3,1) << endl;

    time_gap = 0.1;//default frequency is 10hz
    if(frequency)
    {
        time_gap = 1.0/((double)frequency);
    }
    first_msg = true;

    mavros_pub = nh.advertise<geometry_msgs::PoseStamped>(mavros_topic, 2);
    ros::Subscriber sub = nh.subscribe(odom_topic, 1, vo_callback);

    ros::spin();

    return 0;
}
