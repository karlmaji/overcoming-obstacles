#include<ros/ros.h>
#include<serial/serial.h>
#include<geometry_msgs/Twist.h>
#include <string>

serial::Serial ser;

void openserial()
{
    try
        {
            ser.setPort("/dev/ttyUSB0");//设备端口号
            ser.setBaudrate(115200);//波特率
            serial::Timeout t = serial::Timeout::simpleTimeout(1000);//这个应该是超时，但是是必须的！！ 
            ser.setTimeout(t); 
            ser.open();//打开串口
        }
    catch (serial::IOException& e) 
            { 
                ROS_ERROR_STREAM("Unable to open port "); 
            } 
    if(ser.isOpen())
        {
            ROS_INFO("OPEN");
        }

}
void angle_callback(const geometry_msgs::Twist& cmd_vel){
    uint8_t data[14];
    char str[40];
    uint8_t recv_data[40];
    int linear_speed=cmd_vel.linear.x;
    if(linear_speed>1000) linear_speed=1000;
    else if(linear_speed<-1000) linear_speed=-1000;

    int angel_speed=cmd_vel.angular.z;
    if(angel_speed>1000) angel_speed=1000;
    else if(angel_speed<-1000) angel_speed=-1000;

    sprintf(str,"!M %d %d \r\n",angel_speed,linear_speed);
    

    ser.write(str);

}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"cmdvel_serial");
    ros::NodeHandle n;
    ros::Subscriber angle_sub = n.subscribe("cmd_vel",1,angle_callback);

    openserial();    

    ros::spin();


    return 0;


}