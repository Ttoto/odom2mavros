#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

#include <sstream>

bool vo_has_pose;
geometry_msgs::PoseStamped latest_pose;

void vo_callback(const nav_msgs::OdometryConstPtr msg)
{
  latest_pose.header.frame_id = "map";
  latest_pose.header.stamp=msg->header.stamp;
  latest_pose.pose = msg->pose.pose;
  vo_has_pose = true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom2mavros");

  ros::NodeHandle n;

  vo_has_pose = false;

  ros::Publisher  pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 2);
  ros::Subscriber sub = n.subscribe("/camera/odom/sample", 1, vo_callback);

  ros::Rate loop_rate(20);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    if(vo_has_pose)
    {
      pub.publish(latest_pose);
    }

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
