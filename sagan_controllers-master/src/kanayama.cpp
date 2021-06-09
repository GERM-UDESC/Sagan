#include "ros/ros.h"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/JointState.h"
/**
 * This is a kanayma controller with messages for the ROS system.
 */
geometry_msgs::Twist vel;
float pc[3], pr[3], pe[3]; // Declare variables pc-(current posture), pr-(reference posture), pe-(error posture)
const int x = 0, y = 1, theta = 2; // Define consts
ros::Publisher topic; // Declare publisher called topic
int n=0; float t=0, ts=0.01;/* Define time for trajectory generation
 t = real time; ts = step time; n = increment;
*/
float v,w; //Define Velocities
float Vr, Wr;
const float Kx = 2.5, Ky = 4 , Kth = 2;
void chatterCallback(const sensor_msgs::JointState::ConstPtr& current_pose);
void publish(const ros::Publisher& topic_pub);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kanayama");
  ros::NodeHandle n;
  ros::Publisher topic_pub = n.advertise<geometry_msgs::Twist>("/sagan_gazebo/vel_cmd", 1000);
  topic = topic_pub;
  ros::Subscriber sub = n.subscribe("/sagan_gazebo/state", 1000, chatterCallback);
  ros::spin();

  return 0;
}
void chatterCallback(const sensor_msgs::JointState::ConstPtr& current_pose)
{

  // print all the remaining numbers
  pc[x] = current_pose->position[0];
  pc[y] = current_pose->position[1];
  pc[theta] = current_pose->position[5];

      ROS_INFO("x = %.4f, y = %.4f theta = %.4f", pc[x],pc[y],pc[theta]);



  //reference trajectory
  t=n*ts; //real time calculation
  pr[x]=0.5*t;
  pr[y]=0.5*t;
  pr[theta]=0.785;
  Vr = 0.5; Wr = 0;  // reference Velocities
  n++;   // increment

  //error calculator
  pe[x]=cos(pc[theta])*(pr[x]-pc[x])+sin(pc[theta])*(pr[y]-pc[y]);
  pe[y]=-sin(pc[theta])*(pr[x]-pc[x])+cos(pc[theta])*(pr[y]-pc[y]);
  pe[theta]=pr[theta]-pc[theta];

  // Velocities calculator
v = (Vr)*cos(pe[theta])+Kx*pe[x];
w = Wr + (Vr)*(Ky*pe[y]+ Kth*sin(pe[theta]));

    vel.linear.x = v;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = w;
    publish(topic);

  return;
}
void publish(const ros::Publisher& topic_pub)
{
    topic_pub.publish(vel);
    return;
}
