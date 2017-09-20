#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char **argv){

ros::init(argc,argv,"talker");
ros::NodeHandle r;
ros::Publisher chatty=r.advertise<geometry_msgs::Twist>("robot_1/cmd_vel",1);
ros::Rate loop_rate(1000);
geometry_msgs::Twist velocity;
while(ros::ok())
{
  velocity.linear.x=-1.5;
  velocity.linear.y=-0.5;
  //velocity.angular.z=0.55;
  chatty.publish(velocity);
  ros::spinOnce();
}
 return 0;
}
