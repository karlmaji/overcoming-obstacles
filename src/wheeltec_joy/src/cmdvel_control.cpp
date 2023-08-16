#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include<iostream>
#include <std_msgs/Float64.h>
using namespace std;
class cmd_vel_joy
{
public:
    cmd_vel_joy();
    double vlinear_x_max; 
    double vlinear_z_max;
    double v_linear_x;
    double v_linear_z;
    void loop_function();
    int loop_rate;
private:
    void callback(const sensor_msgs::Joy::ConstPtr& Joy); 
    //实例化节点
    ros::NodeHandle n; 
    ros::Subscriber sub ;
    ros::Publisher pub ;

    sensor_msgs::Joy joy_ago;

    bool control_begin;

 
};
 
cmd_vel_joy::cmd_vel_joy() 
{


   ros::NodeHandle private_nh("~"); //创建节点句柄
   private_nh.param<double>("vlinear_x_max",vlinear_x_max,5); 
   private_nh.param<double>("vlinear_z_max",vlinear_z_max,3);
   private_nh.param<int>("loop_rate",loop_rate,10); 
   pub = n.advertise<geometry_msgs::Twist>("cmd_vel",10);//将速度发给机器人底盘节点
   sub = n.subscribe<sensor_msgs::Joy>("joy",10,&cmd_vel_joy::callback,this); //订阅手柄发来的数据
   control_begin = false;

   v_linear_x = 0;
   v_linear_z = 0;



} 

void cmd_vel_joy::callback(const sensor_msgs::Joy::ConstPtr& joy) //键值回调函数
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
            control_begin = false;
        }
        else if (abs(joy_ago.buttons[8]<1e-3) && abs(joy->buttons[8]-1)<1e-3)
        {
            control_begin = !control_begin;
            if(control_begin) ROS_INFO("cmdvel_control_open!"); else
            ROS_INFO("cmdvel_control_close!");
        }
        if(control_begin)
        {   
            v_linear_x = joy->axes[1] * vlinear_x_max ;
            v_linear_z = joy->axes[0] * vlinear_z_max ;

        }
        joy_ago = *joy;

    }
}

void cmd_vel_joy::loop_function()
{
  if(control_begin)
  {
    geometry_msgs::Twist v;
    v.linear.x = v_linear_x;
    v.angular.z = v_linear_z;

    pub.publish(v);

  }
}
int main(int argc,char** argv)
{
  ros::init(argc, argv, "cmdvel_control");
  cmd_vel_joy cmd_vel_joy_class;
  ros::Rate rate(cmd_vel_joy_class.loop_rate);
  ros::AsyncSpinner s(1);
  s.start();
  while(ros::ok())
  {
    cmd_vel_joy_class.loop_function();
    rate.sleep();
  }
  return 0;

} 
