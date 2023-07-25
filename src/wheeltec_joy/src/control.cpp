#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include<iostream>
#include <std_msgs/Float64.h>
using namespace std;
class wheeltec_joy
{
public:
    wheeltec_joy();
    std_msgs::Float64 vlinear_x; //默认值
    std_msgs::Float64 vlinear_z;
private:
    void callback(const sensor_msgs::Joy::ConstPtr& Joy); 
    //实例化节点
    ros::NodeHandle n; 
    ros::Subscriber sub ;
    ros::Publisher pub,pub2 ;
 
    //机器人的初始速度
    double vlinear,vangular;
    //手柄键值
    int axis_ang,axis_lin; 
    int dir=0,flag_mec;

};
 
wheeltec_joy::wheeltec_joy() 
{
   //读取参数服务器中的变量值
     flag_mec=0;

   ros::NodeHandle private_nh("~"); //创建节点句柄
   private_nh.param<int>("axis_linear",axis_lin,1); //默认axes[1]接收速度
   private_nh.param<int>("axis_angular",axis_ang,0);//默认axes[0]接收角度
   private_nh.param<double>("vlinear",vlinear,0.0);//默认线速度0.3 m/s
   private_nh.param<double>("vangular",vangular,2);//默认角速度1 单位rad/s

   pub = n.advertise<geometry_msgs::Twist>("cmd_vel",10);//将速度发给机器人底盘节点
  //  pub2 = n.advertise<geometry_msgs::Twist>("/carla/ego_vehicle/twist",1);//将速度发给机器人底盘节点
   sub = n.subscribe<sensor_msgs::Joy>("joy",10,&wheeltec_joy::callback,this); //订阅手柄发来的数据
} 

void wheeltec_joy::callback(const sensor_msgs::Joy::ConstPtr& Joy) //键值回调函数
 {
   double acce_x,acce_z;
   static int button4_ago;
   int button4;
   
   geometry_msgs::Twist v;

   button4 =Joy->buttons[4];
   acce_x=Joy->axes[2]+1.0;   //读取右摇杆的值对机器人的线速度进行加减速处理
   
   acce_z=Joy->axes[0];   //读取右摇杆的值对机器人的角速度进行加减速处理
   if(button4 ==0 && button4_ago ==1){

    if(dir==1){dir=-1;}
    else{dir=1;}
   }

   button4_ago= button4;
 
   //处理数据

   v.linear.x = dir * vlinear * acce_x;
   v.angular.z = dir* vangular *acce_z;
   
   vlinear_z.data = v.angular.z;
   vlinear_x.data = v.linear.x;
   
   //打印输出
   //ROS_INFO("linear:%.3lf angular:%.3lf",vlinear_x.data,v.angular.z);
   pub.publish(v);
   v.linear.x = dir * 10 * acce_x;
  //  pub2.publish(v);
 
}
int main(int argc,char** argv)
{
  ros::init(argc, argv, "joy_control");
  wheeltec_joy teleop_turtle;
  ros::spin();
  return 0;

} 
