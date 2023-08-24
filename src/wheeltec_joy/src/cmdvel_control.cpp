#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include<iostream>
#include <std_msgs/Float64.h>
#include<std_msgs/Int16.h>
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
    ros::Publisher putdown_pub ;
    sensor_msgs::Joy joy_ago;


    bool control_begin;
    bool put_down_menu;
    int putdown_id;
    bool confirm;
    bool X_press;
    bool Y_press;
    bool A_press;
    bool B_press;

 
};
 
cmd_vel_joy::cmd_vel_joy() 
{


   ros::NodeHandle private_nh("~"); //创建节点句柄
   private_nh.param<double>("vlinear_x_max",vlinear_x_max,5); 
   private_nh.param<double>("vlinear_z_max",vlinear_z_max,3);
   private_nh.param<int>("loop_rate",loop_rate,10); 
   pub = n.advertise<geometry_msgs::Twist>("cmd_vel",10);//将速度发给机器人底盘节点
   sub = n.subscribe<sensor_msgs::Joy>("joy",10,&cmd_vel_joy::callback,this); //订阅手柄发来的数据
   putdown_pub = n.advertise<std_msgs::Int16>("putdown_cmd",1);

   control_begin = false;

   v_linear_x = 0;
   v_linear_z = 0;

   put_down_menu = false;
   putdown_id = 1;
   confirm = false;
   X_press = false;
   Y_press = false;
   A_press = false;
   B_press = false;



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

            if(abs(joy_ago.buttons[2]<1e-3) && abs(joy->buttons[2]-1)<1e-3)
            {
                X_press = true;
            }
            if(abs(joy_ago.buttons[3]<1e-3) && abs(joy->buttons[3]-1)<1e-3)
            {
                Y_press = true;
            }
           if(abs(joy_ago.buttons[0]<1e-3) && abs(joy->buttons[0]-1)<1e-3)
            {
                A_press = true;
            }
            if(abs(joy_ago.buttons[1]<1e-3) && abs(joy->buttons[1]-1)<1e-3)
            {
                B_press = true;
            }
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

    if(Y_press)
    {
      Y_press = false;
      put_down_menu =!put_down_menu;
      if(put_down_menu)
      {
        ROS_INFO("-------------put down menu start -----------------");
        ROS_INFO("choose id(1,2,3,4) to put down:now id is %d",putdown_id);
      }
      else
      {
              
        ROS_INFO("-------------put down menu close -----------------");
      }
    }
    if(X_press)
    {
      X_press = false;
      if(put_down_menu)
      {
        putdown_id = putdown_id -1 ;
        if(putdown_id<=1) putdown_id = 1 ;
        ROS_INFO("choose id(1,2,3,4) to put down:now id is %d",putdown_id);
      }
    }
    if(B_press)
    {
      B_press = false;
      if(put_down_menu)
      {
        putdown_id = putdown_id + 1 ;
        if(putdown_id>=4) putdown_id = 4 ;
        ROS_INFO("choose id(1,2,3,4) to put down:now id is %d",putdown_id);
      }
    }
    if(A_press)
    {
      A_press = false;
      if(put_down_menu)
      {
        std_msgs::Int16 pd_cmd;
        pd_cmd.data = putdown_id;
        putdown_pub.publish(pd_cmd);
        ROS_INFO("id:%d, is put down now",putdown_id);
      }
    }
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
