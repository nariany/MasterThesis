#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>                       //BIBLIOTEKE
#include <vector>
#include <tf/transform_datatypes.h>
#include <acado_toolkit.hpp>

using namespace std;
//std::vector<std::vector<float> >Pretvaranje;
std::vector<float> odometrija;
// void callback(const sensor_msgs::LaserScan &scan){
     // std::vector<std::vector<float> >Pretvaranje;
//      Pretvaranje.resize(2);
//      float angular_resolution=(scan.angle_max-scan.angle_min)/scan.ranges.size();
//      float pola_ugla=(scan.angle_max-scan.angle_min)/2;
//      for(int i=0; i<Pretvaranje.size();i++)Pretvaranje[i].resize(360);
//       for (int i=0;scan.ranges.size();i++){
//         float phi=(i*angular_resolution)-pola_ugla;
//         float x=scan.ranges[i]*cos(phi);
//         float y=scan.ranges[i]*sin(phi);
//         Pretvaranje[0][i]=x;
//         Pretvaranje[1][i]=y;
//      }
// }
 //Funkcija koja vraca (x,y,theta) (primarno se pretvara theta) odometrijske koordinate
  //ros::Publisher control;
  void odometrycallback(const nav_msgs::Odometry::ConstPtr& msg){
       //std::vector<float> povratniOdometrijskiVektor;
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
      // return odometrija;
 }
 
//Glavna funkcija koja radi optimizaciju
//ros::Publisher control;
//void MPC(const sensor_msgs::LaserScan &scan,const nav_msgs::Odometry::ConstPtr &msg){
    //  USING_NAMESPACE_ACADO
    //  std::vector<std::vector<float> >LaserskiPodaci=Pretvaranje;
    //  std::vector<float> OdometrijskiPodaci=odometrija;
    //  DifferentialState x,y,theta;
    //  Control v,omega;
    //  const double Ts=0.01;
    //  const double t_start=0.0;
    //  const double N=30.0;
    //  DiscretizedDifferentialEquation f(Ts);
    //  f<<next(x)==x+Ts*v*cos(theta);
    //  f<<next(y)==y+Ts*v*sin(theta);
    //  f<<next(theta)==theta+Ts*omega;
    //  Function h;
    //  h<<x;
    //  h<<y;
    //  h<<theta;
    //  DMatrix P(3,3);
    //  P(0,0)=0.1;
    //  P(1,1)=0.1;
    //  P(2,2)=0.1;
    //  DMatrix Q(3,3);
    //  Q(0,0)=0.1;
    //  Q(1,1)=0.1;
    //  Q(2,2)=0.1;
    //  DVector r(3);
      //Problem evaluacije ugla theta
    //  r(0)=5.0;
    //  r(1)=5.0;
    //  r(2)=1.047;
    //  OCP ocp(t_start,N*Ts,N);
    //  ocp.minimizeLSQ(Q,h,r);
    //  ocp.minimizeLSQEndTerm(P,h,r);
    //  ocp.subjectTo(f);
    //  ocp.subjectTo(AT_START,x==OdometrijskiPodaci[0]);
    //  ocp.subjectTo(AT_START,y==OdometrijskiPodaci[1]);
    //  ocp.subjectTo(AT_START,theta==OdometrijskiPodaci[2]);
    //  ocp.subjectTo(AT_START,v==0);
    //  ocp.subjectTo(AT_START,omega==0);
    //  ocp.subjectTo(-0.7<=v<=0.7);
    //  ocp.subjectTo(-4.2<=omega<=4.2);
    //  OptimizationAlgorithm algorithm(ocp);
    //  algorithm.solve();
    //  VariablesGrid upravljanje;
    //  algorithm.getControls(upravljanje);
    //  geometry_msgs::Twist objavaUpravljanja;
    //  objavaUpravljanja.linear.x=upravljanje(1,0);
    //  objavaUpravljanja.linear.y=upravljanje(1,0);
    //  objavaUpravljanja.angular.z=upravljanje(2,0);
    //  control.publish(objavaUpravljanja);
       
 // }

//Pretplacivanje na teme i odbavljivanje poruka tipa cmd_vel
int main(int argc, char **argv) {
       ros::init(argc,argv,"thesis");
       ros::NodeHandle n;
       //ros::Subscriber sub=n.subscribe("/base_scan",10,callback);
       //ros::NodeHandle m;
        ros::Subscriber sub1=n.subscribe("/base_pose_ground_truth",10,odometrycallback);
        ros::Publisher control=n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
        USING_NAMESPACE_ACADO
        //std::vector<std::vector<float> >LaserskiPodaci=Pretvaranje;
        std::vector<float> OdometrijskiPodaci=odometrija;
        DifferentialState x,y,theta;
        Control v,omega;
        const double Ts=0.01;
        const double t_start=0.0;
        const double N=30.0;
        DiscretizedDifferentialEquation f(Ts);
        f<<next(x)==x+Ts*v*cos(theta);
        f<<next(y)==y+Ts*v*sin(theta);
        f<<next(theta)==theta+Ts*omega;
        Function h;
        h<<x;
        h<<y;
        h<<theta;
        DMatrix P(3,3);
        P(0,0)=0.1;
        P(1,1)=0.1;
        P(2,2)=0.1;
        DMatrix Q(3,3);
        Q(0,0)=0.1;
        Q(1,1)=0.1;
        Q(2,2)=0.1;
        DVector r(3);
      //Problem evaluacije ugla theta
        r(0)=5.0;
        r(1)=5.0;
        r(2)=1.047;
        OCP ocp(t_start,N*Ts,N);
        ocp.minimizeLSQ(Q,h,r);
        ocp.minimizeLSQEndTerm(P,h,r);
        ocp.subjectTo(f);
        ocp.subjectTo(AT_START,x==OdometrijskiPodaci[0]);
        ocp.subjectTo(AT_START,y==OdometrijskiPodaci[1]);
        ocp.subjectTo(AT_START,theta==OdometrijskiPodaci[2]);
        ocp.subjectTo(AT_START,v==0);
        ocp.subjectTo(AT_START,omega==0);
      ocp.subjectTo(-0.7<=v<=0.7);
      ocp.subjectTo(-4.2<=omega<=4.2);
      OptimizationAlgorithm algorithm(ocp);
      algorithm.solve();
      VariablesGrid upravljanje;
      algorithm.getControls(upravljanje);
      geometry_msgs::Twist objavaUpravljanja;
      objavaUpravljanja.linear.x=upravljanje(1,0);
      objavaUpravljanja.linear.y=upravljanje(1,0);
      objavaUpravljanja.angular.z=upravljanje(2,0);
      control.publish(objavaUpravljanja);
     //  ros::NodeHandle n;
       ros::Rate loop_rate(10);
     //  control=n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
       ros::spin();
       return 0;
  }

