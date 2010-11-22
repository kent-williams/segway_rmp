#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "segway_apox.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "segway_apox/SegwayOdom.h"
//#include "robot_msgs/PoseDot.h"
//#include "robot_msgs/PoseStamped.h"
//#include "deprecated_msgs/RobotBase2DOdom.h"
#include "tf/transform_broadcaster.h"

using namespace ros;
using std::string;

static const float MAX_X_VEL = 1.2;
static const float MAX_YAW_RATE = 0.4;
static geometry_msgs::Twist g_last_cmd_vel;
static double g_last_req_time = 0;
boost::mutex cmd_mutex;

void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
  cmd_mutex.lock();
  g_last_cmd_vel = *cmd_vel;
  cmd_mutex.unlock();
  g_last_req_time = ros::Time::now().toSec();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "segway_apox");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
//  std::string port("/dev/ttyUSB2");
//  nh_private.getParam("port", port);
//  ROS_DEBUG("opening canbus on %s\n", port.c_str());
  SegwayApox segway;
	tf::TransformBroadcaster tf;
  double last_send_time = ros::Time::now().toSec();
  //robot_msgs::PoseStamped odom;
  //deprecated_msgs::RobotBase2DOdom odom;
  //odom.header.frame_id = "odom";
  //ros::Publisher odom_pub = nh.advertise<robot_msgs::PoseStamped>("odom", 1);
  //ros::Publisher odom_pub = nh.advertise<deprecated_msgs::RobotBase2DOdom>("odom", 1);
  ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 10, cmd_vel_cb);
  ros::Publisher odom_pub = nh.advertise<segway_apox::SegwayOdom>("odom", 1);
  bool req_timeout = true;

  while(ros::ok())
  {
    ros::spinOnce();
    if (ros::Time::now().toSec() - last_send_time > 0.01)
    {
      double time_since_last_cmd = ros::Time::now().toSec() - g_last_req_time;
      if (time_since_last_cmd > 0.15)
        req_timeout = true;
      else
        req_timeout = false;

      if (!req_timeout)
      {
        //printf("sending %f %f\n", g_last_cmd_vel.vel.vx, g_last_cmd_vel.ang_vel.vz);
        cmd_mutex.lock();
        segway.send_vel_cmd(g_last_cmd_vel.linear.x, g_last_cmd_vel.angular.z);
        cmd_mutex.unlock();
      }
      last_send_time = ros::Time::now().toSec();
    }
    if (segway.poll(0.5))
    {
      float x = 0, y = 0, yaw = 0;
      uint32_t left, right;
      segway_apox::SegwayOdom odom;
      odom.header.stamp = ros::Time::now();
      segway.get_last_odometry(x, y, yaw, left, right);
      odom.left = left;
      odom.right = right;
      odom.x = x;
      odom.y = y;
      odom.yaw = yaw;
      odom_pub.publish(odom);
      /*
      odom.pose.position.x  = x;
      odom.pose.position.y  = y;
      odom.pose.position.z  = 0;
      */
      //tf::Quaternion rot(yaw, 0,0);
      //tf::QuaternionTFToMsg(rot, odom.pose.orientation);
      /*
      odom.pos.x  = x;
      odom.pos.y  = y;
      odom.pos.th = yaw;
      odom.header.stamp = ros::Time::now();
      odom_pub.publish(odom);
      */
      /*
      tf.sendTransform(tf::Stamped<tf::Transform>(
                                              tf::Transform(rot, 
                                              tf::Point(x, y, 0.0)),
                       ros::Time::now(), string("base_footprint"), string("odom")));
      */
    }
	  usleep(500);
  }
  return 0;
}

