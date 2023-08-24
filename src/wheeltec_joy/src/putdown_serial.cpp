#include<ros/ros.h>
#include<serial/serial.h>
#include<std_msgs/Int16.h>

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
void angle_callback(const std_msgs::Int16& pd_cmd){
    uint8_t data[1];

    data[0] = uint8_t(pd_cmd.data);

    ROS_INFO("%d",data[0]);








    ser.write(data,1);

}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"putdown_serial");
    ros::NodeHandle n;
    ros::Subscriber angle_sub = n.subscribe("putdown_cmd",1,angle_callback);

    openserial();    

    ros::spin();


    return 0;


}