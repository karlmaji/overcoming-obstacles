#include<ros/ros.h>
#include<serial/serial.h>
#include<ptz_serial/ptz.h>



serial::Serial ser;
//功能码
const uint8_t up = 0x08;
const uint8_t down = 0x10;
const uint8_t left = 0x04;
const uint8_t right = 0x02;

void openserial()
{
    try
        {
            ser.setPort("/dev/ttyUSB0");//设备端口号
            ser.setBaudrate(9600);//波特率
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
void angle_callback(const ptz_serial::ptzConstPtr& xy_speed){
    uint8_t data[7];
    uint16_t sum=0;
    
    data[0] = 0xff;
    data[1] = 0x01;
    data[2] = 0x00;
    data[3] = 0x00;
    if(xy_speed->x_speed >0) data[3]+=right;
    if(xy_speed->x_speed <0) data[3]+=left;
    if(xy_speed->y_speed >0) data[3]+=up;
    if(xy_speed->y_speed <0) data[3]+=down;
    data[4] = (abs(xy_speed->x_speed));
    data[5] = (abs(xy_speed->y_speed));
    
    for(int i = 1; i<=5;i++)
    {
        sum += data[i];
    }

    data[6] = sum & 0x00FF;

    ROS_INFO("%d", data[6]);
    ROS_INFO("%d", sum);
    ser.write(data,7);
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"serial_stm32");
    ros::NodeHandle n;
    ros::Subscriber angle_sub = n.subscribe("ptz_control",1,angle_callback);
    //ros::NodeHandle n_private("~");
    openserial();
    ros::Rate loop_rate(1);

    ros::spin();

    return 0;


}