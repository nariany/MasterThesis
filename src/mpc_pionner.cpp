#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/Pose.h>
#include <vector>
#include <tf/transform_datatypes.h>
#include <acado_toolkit.hpp>
#include <math.h>
#include <acado_gnuplot.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
//#include <Vector3.h>
using namespace std;
USING_NAMESPACE_ACADO
class MPC_Control_Mobile_Robot
{    
     ros::NodeHandle n_;
     ros::Subscriber sub_;
     ros::Subscriber sub1_;
     ros::Subscriber sub2_;
     ros::Publisher pub_;
     tf::TransformListener listener;
    // tf::TransformBroadcaster
    // tf::StampedTransform transform;
    // std::vector<std::vector<float> > Pomocni_Vektor;
     std::vector<float> odometry;
     double linear_velocity,angular_velocity;
     std::vector<std::vector<float> > LaserScanToXY;
     std::vector<float> laserDistanceVector;
 public:
     MPC_Control_Mobile_Robot();
     ~MPC_Control_Mobile_Robot();
    void callback_odometry(const nav_msgs::Odometry::ConstPtr &msg);
    void callback_laser_scan(const sensor_msgs::LaserScan &scan);
    void callback_velocity(const geometry_msgs::Twist::ConstPtr& vel);
    void optimizationfunction();
};
int main(int argc,char **argv){
    //ROS_INFO("I heard prije cvora");
    std::cout<<"I heard prije cvora"<<"\n";
    ros::init(argc,argv,"MPC_Control_Mobile_Robot_node");
    ros::NodeHandle r;
    ros::Rate loop_rate(100); 
    MPC_Control_Mobile_Robot pionner;
    while(ros::ok())
    {
    //std::cout<<"I heard prije instancije klase"<<"\n";
    //ros::Rate r(10);
    //ROS_INFO("I heard poslije instancije klase");
    std::cout<<"I heard poslije instancije klase"<<"\n";
    //pionner.optimizationfunction(); 
    ros::spinOnce();
    pionner.optimizationfunction();
    loop_rate.sleep();
    }
   std::cout<<"Kraj"<<"\n";
   return 0;
 }
MPC_Control_Mobile_Robot::MPC_Control_Mobile_Robot()
 {
    //ROS_INFO("I heard na pocetku konstruktora");
    std::cout<<"I heard na pocetku konstruktora"<<"\n";
    linear_velocity=0.0;
    angular_velocity=0.0;
    odometry.resize(3);
    odometry[0]=0;
    odometry[1]=0;
    odometry[2]=0;
    geometry_msgs::Twist pub_msg;
    pub_msg.linear.x=0.0;
    pub_msg.linear.y=0.0;
    pub_msg.linear.z=0.0;
    pub_msg.angular.x=0.0;
    pub_msg.angular.y=0.0;
    pub_msg.angular.z=0.0;
    laserDistanceVector.resize(15);
    LaserScanToXY.resize(2); 
    for(int i=0;i<LaserScanToXY.size();i++)LaserScanToXY[i].resize(15);
   // Pomocni_Vektor.resize(2);
   // for(int i=0;i<Pomocni_Vektor.size();i++)Pomocni_Vektor[i].resize(6);
    sub_=n_.subscribe("robot_0/odom",10,&MPC_Control_Mobile_Robot::callback_odometry,this);
    sub1_=n_.subscribe("robot_0/base_scan",10,&MPC_Control_Mobile_Robot::callback_laser_scan,this);
    sub2_=n_.subscribe("robot_0/cmd_vel",10,&MPC_Control_Mobile_Robot::callback_velocity,this);
    pub_=n_.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",10);
   
    //tf::StampedTransform transform;
    //ROS_INFO("I heard na kraju kontstruktora");
    std::cout<<"I heard na kraju konstruktora"<<"\n"; 
    pub_.publish(pub_msg);
}
MPC_Control_Mobile_Robot::~MPC_Control_Mobile_Robot()
{}
void MPC_Control_Mobile_Robot::callback_odometry(const nav_msgs::Odometry::ConstPtr& msg)//nav_msgs::Odometry::ConstPtr& msg)
{    ROS_INFO("I heard na pocetku callback_odometry");
    // odometry.resize(3);
     odometry[0]=msg->pose.pose.position.x;
     odometry[1]=msg->pose.pose.position.y;
     double quatx=msg->pose.pose.orientation.x;
     double quaty=msg->pose.pose.orientation.y;
     double quatz=msg->pose.pose.orientation.z;
     double quatw=msg->pose.pose.orientation.w;
    // std::cout<<"Quat koordinate(x,y,z,w)t:"<<quatx<<" "<<quaty<<" "<<quatz<<" "<<quatw<<"\n";
   //  tf::Quaternion q(quatx,quaty,quatz,quatw);
   //  tf::Matrix3x3 m(q);
   //  double roll,pitch,yaw;
   //  m.getRPY(roll,pitch,yaw);
   //  std::cout<<"roll,pitch,yaw:"<<roll<<" "<<pitch<<" "<<yaw<<"\n";
     odometry[2]=quatz;//quatz;;
     //double siny=2*(quatw*quatx+quaty*quatz);
     //double cosy=1-2*(quatx*quatx+quaty*quaty);
     //double yaw;
     //yaw=atan2(siny,cosy);
     //odometry[2]=yaw;
  //   ROS_INFO("I heard na kraju callback_odometry");
  //   for(int i=0;i<odometry.size();i++){
  //     std::cout<<"Odometry data:"<<"[i+1]"<<odometry[i]<<"\n"; 
  //   }
 }
void MPC_Control_Mobile_Robot::callback_velocity(const geometry_msgs::Twist::ConstPtr& vel)
 {  ROS_INFO("I heard na pocetku callback_velocity");
    std::cout<<"Only linear:"<<vel->linear.x;
    double help=sqrt(vel->linear.x*vel->linear.x+vel->linear.y*vel->linear.y);
    std::cout<<"Help variable:"<<help<<endl;	
    linear_velocity=vel->linear.x;
    angular_velocity=vel->angular.z;
    std::cout<<linear_velocity<<" "<<angular_velocity<<"\n";
    ROS_INFO("I heard na kraju callback_velocity");
 }
void MPC_Control_Mobile_Robot::callback_laser_scan(const sensor_msgs::LaserScan &scan)
 {   ROS_INFO("I heard na pocetku laser scan");
    // LaserScanToXY.resize(2);
     ROS_INFO("I heard ovdje isto");
     float angular_resolution=(scan.angle_max-scan.angle_min)/scan.ranges.size();
     float half_angle=(scan.angle_max-scan.angle_min)/2;
     ROS_INFO("I heard prije resize petlje");
    // for (int i=0;i<LaserScanToXY.size();i++)LaserScanToXY[i].resize(360);
     ROS_INFO("I heard prije petlje");
     for(int i=0;i<scan.ranges.size();i++)
      {
     //   ROS_INFO("I heard u petlji laser_scan");
        float phi=(i*angular_resolution)-half_angle;
        float x=scan.ranges[i]*cos(phi);
        float y=scan.ranges[i]*sin(phi);
        LaserScanToXY[0][i]=x;
        LaserScanToXY[1][i]=y;
        laserDistanceVector[i]=scan.ranges[i];
       // std::cout<<"x="<<LaserScanToXY[0][i]<<" y="<<LaserScanToXY[1][i]<<"\n";
      } ROS_INFO("I heard nakon petlje laser_scan");
        for(int i=0;i<LaserScanToXY[0].size();i++){
		  geometry_msgs::PointStamped point_in_laser_frame;
                  point_in_laser_frame.header.frame_id="robot_0/base_laser_link";
                  point_in_laser_frame.header.stamp=ros::Time();
		  point_in_laser_frame.point.x=LaserScanToXY[0][i];
		  point_in_laser_frame.point.y=LaserScanToXY[1][i];
		  point_in_laser_frame.point.z=0.0;
		  geometry_msgs::PointStamped point_in_base_frame;
		  listener.transformPoint("robot_0/odom",point_in_laser_frame,point_in_base_frame);
		  LaserScanToXY[0][i]=point_in_base_frame.point.x;
		  LaserScanToXY[1][i]=point_in_base_frame.point.y;
	  }
	 // for(int i=0;i<LaserScanToXY[0].size();i++) {
	//	  std::cout<<"X koordinata prepreke:"<<LaserScanToXY[0][i]<<"Y koordinata prepreke:"<<LaserScanToXY[1][i]<<endl;
	//  }
   //tf::TransformListener;
   //tf::StampedTransform transform;
   // try{
   // ros::Time now=ros::Time::now();
   // listener.waitForTransform("/odom","/base_laser_link",now,ros::Duration(3.0));
   // listener.lookupTransform("/odom","/base_laser_link",ros::Time::now(),transform); 

   // }
   // catch (tf::TransformException ex){
   // ROS_WARN("Base to camera transform unavailable %s", ex.what());
   // }
   // double quatx=transform.getRotation().x();
   // double quaty=transform.getRotation().y();
   // double quatz=transform.getRotation().z();
   // double quatw=transform.getRotation().w();
   // tf::Quaternion q(quatx,quaty,quatz,quatw);	     
   // tf::Matrix3x3 m(q);
   // std::vector<std::vector<float> >Pomocni_Vektor;
	//  Pomocni_Vektor.resize(2);
	  //for(int i=0;i<LaserScanToXY[0].size();i++)Pomocni_Vektor[i].resize(6);
	//  for(int i=0;i<LaserScanToXY[0].size();i++){
	//	  tf::Vector3 d(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
	//	  tf::Vector3 p=m*d;
	//	  Pomocni_Vektor[0][i]=p.getX();
	//	  Pomocni_Vektor[1][i]=p.getY();
   // }
    // DifferentialState x,y,theta;
    // Control v,omega;
    // const double Ts=0.01;
    // const double t_start=0.0;
    // const double N=30.0;
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
    // r(0)=5.0;
    // r(1)=5.0;
    // r(2)=0;
    // ROS_INFO("I heard nakon inicijalizacija Optimalnog problema");
    // OCP ocp(t_start,N*Ts,N);
    // ROS_INFO("Nakon definiranja OCP-a");
    // ocp.minimizeLSQ(Q,h,r);
    // ocp.minimizeLSQEndTerm(P,h,r);
    // ROS_INFO("Nakon definisanja optimizacijskog_horizonta");
    // ocp.subjectTo(f);
    // ocp.subjectTo(AT_START,x==odometry[0]);
    // ocp.subjectTo(AT_START,y==odometry[1]);
    // ocp.subjectTo(AT_START,theta==odometry[2]);
    // ROS_INFO("I heard_ovdje");
    // ocp.subjectTo(AT_START,v==linear_velocity);
    // ocp.subjectTo(AT_START,omega==angular_velocity);
    // ocp.subjectTo(-0.7<=v<=0.7);
    // ocp.subjectTo(-4.2<=omega<=4.2);
    // ROS_INFO("I heard prije algorithm(ocp)");
    // OptimizationAlgorithm algorithm(ocp);
    // algorithm.solve();
   //  VariablesGrid pub_controls;
   //  algorithm.getControls(pub_controls);
   //  geometry_msgs::Twist control_publisher;
   //  control_publisher.linear.x=pub_controls(0,0);
     //control_publisher.linear.y=pub_controls(0,0);
   //  control_publisher.angular.z=pub_controls(0,1);
   //  std::cout<<"Printam sto salje"<<"\n"; 
   //  std::cout<<pub_controls(0,0)<<"  "<<pub_controls(0,1);
   //  pub_.publish(control_publisher);
   //  clearAllStaticCounters();
   //  ROS_INFO("I heard nakon zavrsetka cjelokupnog procesa f-je callback_laser_scan"); 
  // for(int i=0;i<LaserScanToXY[0].size();i++){
  //     LaserScanToXY[0][i]=LaserScanToXY[0][i]*transform.getOrigin().x();
  //     LaserScanToXY[1][i]=LaserScanToXY[1][i]*transform.getOrigin().y();
  //  }
    
}
 void MPC_Control_Mobile_Robot::optimizationfunction()
 {
   //   sub2_=n_.subscribe("base_scan",10,&MPC_Control_Mobile_Robot::callback_laser_scan,this);
       DifferentialState x,y,theta;
       Control v,omega;
       const double Ts=0.1;
       const double t_start=0.0;
       const double N=15.0;
       DifferentialEquation f;
      // f<<next(x)==x+Ts*v*cos(theta);
      // f<<next(y)==y+Ts*v*sin(theta);
      // f<<next(theta)==theta+Ts*omega;
       f<<dot(x)==v*cos(theta);
       f<<dot(y)==v*sin(theta);
       f<<dot(theta)==omega;
       Function h;
       h<<x;
       h<<y;
       h<<theta;
       DMatrix P(3,3);
       P(0,0)=0.1;
       P(1,1)=0.1;
       P(2,2)=0.1;
       DMatrix Q(3,3);
       Q(0,0)=0.8;
       Q(1,1)=0.8;
       Q(2,2)=0.8;
       DVector r(3);
       r(0)=5.0;
       r(1)=5.0;
       r(2)=0.0;
      // ROS_INFO("I heard nakoninicijalizacija Optimalnog problema");
       OCP ocp(t_start,N*Ts,N);
      // ROS_INFO("Nakon definiranja OCP-a");
       ocp.minimizeLSQ(Q,h,r);
       ocp.minimizeLSQEndTerm(P,h,r);
      // ROS_INFO("Nakon definisanja optimizacijskog_horizonta");
       ocp.subjectTo(f);
      // ROS_INFO("Prije odometry_data_pokusaja");
       //std::cout<<"Odometrijski podaci:"<<"\n";
       //std::cout<<"odometry[0]:"<<odometry[0]<<"\n";
       //std::cout<<"odometry[1]:"<<odometry[1]<<"\n";
       //std::cout<<"odometry[2]:"<<odometry[2]<<"\n";
       //std::cout<<"linear_velocity:"<<linear_velocity<<"\n";
       //std::cout<<"angular_velocity:"<<angular_velocity<<"\n";
       ocp.subjectTo(AT_START,x==odometry[0]);
       ocp.subjectTo(AT_START,y==odometry[1]);
       ocp.subjectTo(AT_START,theta==odometry[2]);
      // ROS_INFO("I heard_ovdje");
       ocp.subjectTo(AT_START,v==linear_velocity);
       ocp.subjectTo(AT_START,omega==angular_velocity);
       ocp.subjectTo(-0.7<=v<=0.7);
       ocp.subjectTo(-5<=omega<=5);
      // std::vector<std::vector<float> > CoordinatesVector;
      // CoordinatesVector.resize(2);
      // std::vector<std::vector<float> > obstacleVector;
      // obstacleVector.resize(2);
      // float stepen_u_radijan=3.14/180;
      // float angle1=30*stepen_u_radijan;
      // float angle2=60*stepen_u_radijan;
      // float angle3=90*stepen_u_radijan;
      // float angle4=120*stepen_u_radijan;
      // float angle5=150*stepen_u_radijan;
      // float angle6=180*stepen_u_radijan;
      // float x_obstacle1,y_obstacle1,x_obstacle2,y_obstacle2,x_obstacle3,y_obstacle3,x_obstacle4,y_obstacle4,x_obstacle5,y_obstacle5,x_obstacle6,y_obstacle6;
      // for(int i=0;i<obstacleVector.size();i++)obstacleVector[i].resize(laserDistanceVector.size());
      // for(int i=0;i<laserDistanceVector.size();i++){  
       // std::cout<<endl;
       // std::cout<<"Broj uzoraka tocaka:"<<LaserScanToXY[0].size()<<endl;
             
            // std::cout<<"x_obstacle:"<<x_obstacle;
            // std::cout<<"y_obstacle:"<<y_obstacle;
            // obstacleVector[0][i]=x_obstacle;
            // obstacleVector[1][i]=y_obstacle;
             std::cout<<"Radi 2"<<endl;
         //double distance=(odometry[0]-LaserScanToXY[0][i])*(odometry[0]-LaserScanToXY[0][i])+(odometry[1]-LaserScanToXY[1][i])*(odometry[1]-LaserScanToXY[1][i]);
         //double real_distance=sqrt(distance);
         //std::cout<<"Distance:"<<distance<<"\n";
        // cout << "dif  " << (x-LaserScanToXY[0][i])*(x-LaserScanToXY[0][i])+(y-LaserScanToXY[1][i])*(y-LaserScanToXY[1][i]) << endl;
        // std::cout << "dif stanje " << sizeof(x)  << endl;
      //  double real_distance=sqrt((LaserScanToXY[0][i]*LaserScanToXY[0][i]) + (LaserScanToXY[1][i])*(LaserScanToXY[1][i])); 
      //  std::cout<<"Real distance:"<<real_distance<<endl;
         // if(laserDistanceVector[i]<1.1)
         // {  
            // CoordinatesVector[0].push_back(LaserScanToXY[0][i]);
           //  CoordinatesVector[1].push_back(LaserScanToXY[1][i]);             
	  //   cout << "unisao" << endl;
          //  ocp.subjectTo(1.0<=(x-x_obstacle)*(x-x_obstacle)+(y-y_obstacle)*(y-y_obstacle));
          //  std::cout<<"Azurira ogranicenje"<<"\n";
         // }
       // }
      //tf::TransformListener listener;
      //tf::StampedTransform transform;
     // try{
		//   listener.lookupTransform("odom","base_laser_link",ros::Time(0.001),transform);
	  // }catch (tf::TransformException ex){
         // ROS_WARN("Base to camera transform unavailable %s", ex.what());
     // }
	 // tf::Quaternion q=transform.getRotation();	    
	 // tf::Matrix3x3 m(q);
	 // std::vector<std::vector<float> >Pomocni_Vektor;
	 // Pomocni_Vektor.resize(2);
	 // for(int i=0;i<LaserScanToXY[0].size();i++)Pomocni_Vektor[i].resize(6);
	 // for(int i=0;i<LaserScanToXY[0].size();i++){
		// tf::Vector3 d(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
		 // tf::Vector3 p=m*d;
		 // Pomocni_Vektor[0][i]=p.getX();
		 // Pomocni_Vektor[1][i]=p.getY();
	 // }
	  for(int i=0;i<laserDistanceVector.size();i++){
	        if(laserDistanceVector[i]<3){
		  ocp.subjectTo((x-LaserScanToXY[0][i])*(x+LaserScanToXY[0][i])+(y-LaserScanToXY[1][i])*(y-LaserScanToXY[1][i])>=1.75);
		  }
		  
	  }
   // ROS_INFO("I heard prije algorithm(ocp)");
       OptimizationAlgorithm algorithm(ocp);
       algorithm.solve();
       VariablesGrid pub_controls;
       algorithm.getControls(pub_controls);
       geometry_msgs::Twist control_publisher;
       double distance_from_final_point=sqrt((odometry[0]-r(0))*(odometry[0]-r(0))+(odometry[1]-r(1))*(odometry[1]-r(1)));
       if(distance_from_final_point<=0.01){
        control_publisher.linear.x=0.0;
        control_publisher.linear.y=0.0;
        control_publisher.angular.z=0.0;
       }else{
       control_publisher.linear.x=pub_controls(1,0)*cos(odometry[2]);
       control_publisher.linear.y=pub_controls(1,0)*sin(odometry[2]);
       //control_publisher.angular.z=0.5;
      //  control_publisher.linear.x=5.0;
      //  control_publisher.linear.y=0.0;
       control_publisher.angular.z=pub_controls(1,1);

  }
       std::cout<<"Printam sto salje"<<"\n"; 
       std::cout<<pub_controls<<"\n";
       pub_.publish(control_publisher);
       clearAllStaticCounters();   
       ROS_INFO("I ovo je proslo, kraj optimization_function");
        
 }
