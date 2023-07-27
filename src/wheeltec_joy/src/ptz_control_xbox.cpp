#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<iostream>
#include<ptz_serial/ptz.h>
using namespace std;
class ptz_control_xbox
{
public:
    ptz_control_xbox();
    double vlinear_x; //默认值
    double vlinear_fast_x; //默认值
    double vlinear_y;
    double vlinear_fast_y;
    double x_speed,y_speed;
    void loop_function();
    int loop_rate;
private:
    void callback(const sensor_msgs::Joy::ConstPtr& Joy); 
    //实例化节点
    ros::NodeHandle n; 
    ros::Subscriber sub ;
    ros::Publisher pub ;
 
    //初始速度
    double vlinear,vangular;
    //手柄键值
    int axis_x,axis_y,axis_fast_x,axis_fast_y; 

    sensor_msgs::Joy joy_ago;
    
    bool control_begin;

};
 
ptz_control_xbox::ptz_control_xbox() 
{
   //读取参数服务器中的变量值

   ros::NodeHandle private_nh("~"); //创建节点句柄
   private_nh.param<int>("loop_rate",loop_rate,10); //默认axes[0]接收x速度
   private_nh.param<int>("axis_x",axis_x,0); //默认axes[0]接收x速度
   private_nh.param<int>("axis_y",axis_y,1);//默认axes[1]接收y速度
   private_nh.param<int>("axis_fast_x",axis_fast_x,3); //默认axes[3]接收x速度
   private_nh.param<int>("axis_fast_y",axis_fast_y,4);//默认axes[4]接收y速度
   private_nh.param<double>("vlinear_x",vlinear_x,32);
   private_nh.param<double>("vlinear_y",vlinear_y,32);
   private_nh.param<double>("vlinear_fast_x",vlinear_fast_x,63);
   private_nh.param<double>("vlinear_fast_y",vlinear_fast_y,63);
   pub = n.advertise<ptz_serial::ptz>("ptz_control",1);//将速度发给云台控制节点
   sub = n.subscribe<sensor_msgs::Joy>("joy",10,&ptz_control_xbox::callback,this); //订阅手柄发来的数据
   x_speed =0 ;
   y_speed =0 ;
   control_begin = false;
} 

void ptz_control_xbox::callback(const sensor_msgs::Joy::ConstPtr& joy) //键值回调函数
 {

    if(joy_ago.axes.size()==0 ||joy_ago.buttons.size()==0) joy_ago=*joy;
    else
    {
        if(abs(joy_ago.buttons[7]<1e-3) && abs(joy->buttons[7]-1)<1e-3)
        {
            control_begin = false;
        }
        else if (abs(joy_ago.buttons[6]<1e-3) && abs(joy->buttons[6]-1)<1e-3)
        {
            control_begin = !control_begin;
            if(control_begin) ROS_INFO("ptz_control_open!"); else
            ROS_INFO("ptz_control_close!");
        }
        else if (abs(joy_ago.buttons[8]<1e-3) && abs(joy->buttons[8]-1)<1e-3)
        {
            control_begin = false;
        }
        if(control_begin)
        {   
            x_speed = -int(joy->axes[axis_x] * vlinear_x + joy->axes[axis_fast_x] * vlinear_fast_x);
            y_speed = int(joy->axes[axis_y] * vlinear_y + joy->axes[axis_fast_y] * vlinear_fast_y);
            if(x_speed>63) x_speed = 63;
            if(x_speed<-63) x_speed = -63;
            if(y_speed>63) y_speed = 63;
            if(y_speed<-63) y_speed = -63;
        }
        joy_ago = *joy;

    }
      
}

void ptz_control_xbox::loop_function()
{
  if(control_begin)
  {
    std::cout<<"x_speed:"<<x_speed<<",y_speed"<<y_speed<<std::endl;
    ptz_serial::ptz ptz_msg;
    ptz_msg.x_speed = x_speed;
    ptz_msg.y_speed = y_speed;
    pub.publish(ptz_msg);
  }
}
int main(int argc,char** argv)
{
  ros::init(argc, argv, "ptz_control_xbox");
  ptz_control_xbox ptz_control;
  ros::Rate rate(ptz_control.loop_rate);
  
  ros::AsyncSpinner s(1);
  s.start();
  while(ros::ok())
  {
    ptz_control.loop_function();
    rate.sleep();
  }
  return 0;

} 
