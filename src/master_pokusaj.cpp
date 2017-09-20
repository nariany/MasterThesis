#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>
#include <tf/transform_datatypes.h>
#include <acado_toolkit.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;
using namespace std;
USING_NAMESPACE_ACADO;
DifferentialState x,y,theta;
Control v,omega;
const double Ts=0.01;
DiscretizedDifferentialEquation f(Ts);
//Control v,omega;
// f<<next(x)==x+Ts*v*cos(theta);
// f<<next(y)==y+Ts*v*sin(theta);
// f<<next(theta)==theta+Ts*omega;
 Function h;
// h<<x;
// h<<y;
// h<<theta;
 DMatrix P(3,3);
// P(0,0)=0.1;
// P(1,1)=0.1;
// P(2,2)=0.1;
 DMatrix Q(3,3);
// Q(0,0)=0.1;
// Q(1,1)=0.1;
// Q(2,2)=0.1;
 DVector r(3);
 //Problem evaluacije ugla theta
// r(0)=5.0;
// r(1)=5.0;
// r(2)=1.047;
//std::vector<std::vector<float> >Pretvaranje;
//std::vector<float> odometrija;
 ros::Publisher upravljanje_objava;
void odometrycallback(const nav_msgs::Odometry::ConstPtr& msg,const sensor_msgs::LaserScan &scan){
       //std::vector<float> povratniOdometrijskiVektor;
      std::vector<std::vector<float> >Pretvaranje;
      Pretvaranje.resize(2);
      float angular_resolution=(scan.angle_max-scan.angle_min)/scan.ranges.size();
      float pola_ugla=(scan.angle_max-scan.angle_min)/2;
      for(int i=0; i<Pretvaranje.size();i++)Pretvaranje[i].resize(360);
       for (int i=0;scan.ranges.size();i++){
         float phi=(i*angular_resolution)-pola_ugla;
         float x=scan.ranges[i]*cos(phi);
         float y=scan.ranges[i]*sin(phi);
         Pretvaranje[0][i]=x;
         Pretvaranje[1][i]=y;
      }
       std::vector<float> odometrija;
       odometrija.resize(3);
       odometrija[0]=msg->pose.pose.orientation.x;
       odometrija[1]=msg->pose.pose.orientation.y;
       double quatx=msg->pose.pose.orientation.x;
       double quaty=msg->pose.pose.orientation.y;
       double quatz=msg->pose.pose.orientation.z;
       double quatw=msg->pose.pose.orientation.w;
       tf::Quaternion q(quatx,quaty,quatz,quatw);
       tf::Matrix3x3 m(q);
       double roll,pitch,yaw;
       m.getRPY(roll,pitch,yaw);
       odometrija[2]=yaw;
      // USING_NAMESPACE_ACADO
        //std::vector<std::vector<float> >LaserskiPodaci=Pretvaranje;
       // std::vector<float> OdometrijskiPodaci=odometrija;
      // DifferentialState x,y,theta;
      // Control v,omega;
      // const double Ts=0.01;
       const double t_start=0.0;
       const double N=30.0;
      // DiscretizedDifferentialEquation f(Ts);
      // f<<next(x)==x+Ts*v*cos(theta);
      // f<<next(y)==y+Ts*v*sin(theta);
      // f<<next(theta)==theta+Ts*omega;
      // Function h;
      // h<<x;
      // h<<y;
      // h<<theta;
      // DMatrix P(3,3);
      // P(0,0)=0.1;
      // P(1,1)=0.1;
      // P(2,2)=0.1;
      // DMatrix Q(3,3);
      // Q(0,0)=0.1;
      // Q(1,1)=0.1;
      // Q(2,2)=0.1;
      // DVector r(3);
      //Problem evaluacije ugla theta
      // r(0)=5.0;
      // r(1)=5.0;
      // r(2)=1.047;
       OCP ocp(t_start,N*Ts,N);
       ocp.minimizeLSQ(Q,h,r);
       ocp.minimizeLSQEndTerm(P,h,r);
       ocp.subjectTo(f);
       ocp.subjectTo(AT_START,x==odometrija[0]);
       ocp.subjectTo(AT_START,y==odometrija[1]);
       ocp.subjectTo(AT_START,theta==odometrija[2]);
       ocp.subjectTo(AT_START,v==0);
       ocp.subjectTo(AT_START,omega==0);
       ocp.subjectTo(-0.7<=v<=0.7);
       ocp.subjectTo(-4.2<=omega<=4.2);
       OptimizationAlgorithm algorithm(ocp);
       algorithm.solve();
       VariablesGrid upravljanje;
       // std::vector<std::vector<float> > upravljanje;
       algorithm.getControls(upravljanje);
       geometry_msgs::Twist objavaUpravljanja;
       objavaUpravljanja.linear.x=upravljanje(1,0);
       objavaUpravljanja.linear.y=upravljanje(1,0);
       objavaUpravljanja.angular.z=upravljanje(2,0);
      // return odometrija;
       upravljanje_objava.publish(objavaUpravljanja);
       //Control dummy1;
       //DifferentialState dummy2;
       //Parameter dummy3;
       //dummy1.clearStaticCounters();
       //dummy2.clearStaticCounters();
       //dummy3.clearStaticCounters();
       clearAllStaticCounters();
 }

int main(int argc, char **argv){
    //DifferentialState x,y,theta;
    //Control v,omega;
    //const double Ts=0.01;
    //DiscretizedDifferentialEquation f(Ts);
   //Control v,omega;
    f<<next(x)==x+Ts*v*cos(theta);
    f<<next(y)==y+Ts*v*sin(theta);
    f<<next(theta)==theta+Ts*omega;
    //Function h;
    h<<x;
    h<<y;
    h<<theta;
  //  DMatrix P(3,3);
    P(0,0)=0.1;
    P(1,1)=0.1;
    P(2,2)=0.1;
   // DMatrix Q(3,3);
    Q(0,0)=0.1;
    Q(1,1)=0.1;
    Q(2,2)=0.1;
   // DVector r(3);
   //Problem evaluacije ugla theta
    r(0)=5.0;
    r(1)=5.0;
    r(2)=1.047;
    ros::init(argc,argv,"master_pokusaj");
    ros::NodeHandle n;
    //Odavde, pokusaj ide
    message_filters::Subscriber<Odometry> odom_sub(n,"base_pose_groud_truth",10);
    message_filters::Subscriber<LaserScan> laser_sub(n,"base_scan",10);
    typedef sync_policies::ApproximateTime<Odometry,LaserScan> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),odom_sub,laser_sub);
    sync.registerCallback(boost::bind(&odometrycallback,_1,_2));
    //ros::Subscriber sub=n.subscribe("base_pose_ground_truth",10,odometrycallback);
    //ros::NodeHandle n;
    ros::Rate loop_rate(1);
    //ros::Publisher upravljanje_objava;
    upravljanje_objava=n.advertise<geometry_msgs::Twist>("cmd_vel",10);
    ros::spin();
   return 0;
 }
