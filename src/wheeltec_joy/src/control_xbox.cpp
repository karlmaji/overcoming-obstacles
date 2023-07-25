#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<iostream>
#include<ptz_serial/ptz.h>
using namespace std;
class xbox_joy
{
public:
    xbox_joy();
    double vlinear_x; //默认值
    double vlinear_fast_x; //默认值
    double vlinear_y;
    double vlinear_fast_y;
private:
    void callback(const sensor_msgs::Joy::ConstPtr& Joy); 
    //实例化节点
    ros::NodeHandle n; 
    ros::Subscriber sub ;
    ros::Publisher pub ;
 
    //机器人的初始速度
    double vlinear,vangular;
    //手柄键值
    int axis_x,axis_y,axis_fast_x,axis_fast_y; 

};
 
xbox_joy::xbox_joy() 
{
   //读取参数服务器中的变量值

   ros::NodeHandle private_nh("~"); //创建节点句柄
   private_nh.param<int>("axis_x",axis_x,0); //默认axes[0]接收x速度
   private_nh.param<int>("axis_y",axis_y,1);//默认axes[1]接收y速度
   private_nh.param<int>("axis_fast_x",axis_fast_x,3); //默认axes[3]接收x速度
   private_nh.param<int>("axis_fast_y",axis_fast_y,4);//默认axes[4]接收y速度
   private_nh.param<double>("vlinear_x",vlinear_x,32);
   private_nh.param<double>("vlinear_y",vlinear_y,32);
   private_nh.param<double>("vlinear_fast_x",vlinear_fast_x,63);
   private_nh.param<double>("vlinear_fast_y",vlinear_fast_y,63);
   pub = n.advertise<ptz_serial::ptz>("ptz_control",1);//将速度发给云台控制节点
  //  pub2 = n.advertise<geometry_msgs::Twist>("/carla/ego_vehicle/twist",1);//将速度发给机器人底盘节点
   sub = n.subscribe<sensor_msgs::Joy>("joy",10,&xbox_joy::callback,this); //订阅手柄发来的数据
} 

void xbox_joy::callback(const sensor_msgs::Joy::ConstPtr& Joy) //键值回调函数
 {
   ptz_serial::ptz ptz_control;

   double x_speed,y_speed;

   x_speed = -int(Joy->axes[axis_x] * vlinear_x + Joy->axes[axis_fast_x] * vlinear_fast_x);
   y_speed = int(Joy->axes[axis_y] * vlinear_y + Joy->axes[axis_fast_y] * vlinear_fast_y);
   
   if(x_speed>63) x_speed = 63;
   if(x_speed<-63) x_speed = -63;
   if(y_speed>63) y_speed = 63;
   if(y_speed<-63) y_speed = -63;
   ptz_control.x_speed = x_speed;
   ptz_control.y_speed = y_speed;
   pub.publish(ptz_control);
}
int main(int argc,char** argv)
{
  ros::init(argc, argv, "joy_xbox_control");
  xbox_joy teleop_turtle;
  ros::spin();
  return 0;

} 
