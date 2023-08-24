#include<ros/ros.h>
#include<std_msgs/Bool.h>
#include<serial/serial.h>




serial::Serial ser;

void openserial()
{
    try
        {
            ser.setPort("/dev/ttyACM0");//设备端口号
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
void bool_callback(const std_msgs::BoolConstPtr& bl){
    uint8_t data[14]={0xFF,0xFE,0xFD,0xFC,0x01,0x06,0x02,0x01,0x00,0x64,0x00,0x00,0x00,0xFB};
    
    if(bl->data == false)
    {
        data[9] = 0x00;
        ROS_INFO("GRAP_CLOSE!");
    }
    else{
        ROS_INFO("GRAP_OPEN!");
    }
    
    ser.write(data,14);
    

}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"grap_serial");
    ros::NodeHandle n;
    ros::Subscriber angle_sub = n.subscribe("grap_control",1,bool_callback);
    //ros::NodeHandle n_private("~");
    openserial();
    ros::Rate loop_rate(2);
    if(ser.isOpen())
    {
        uint8_t data_[14]={0xFF,0xFE,0xFD,0xFC,0x01,0x08,0x02,0x01,0x00,0x00,0x00,0x00,0x00,0xFB};
        ser.write(data_,14);
        while(ros::ok())
        {
            loop_rate.sleep();
            data_[7]=0x00;
            ser.write(data_,14);

            uint8_t buffer[14];

            

            auto rev = ser.read(buffer,14);
            //ROS_INFO("%d",rev);
            for(int i=0;i<14;i++)
            {
            std::cout<< std::hex<< (buffer[i] & 0xff) <<"  ";
            }
            std::cout<<std::endl;
        
            if(buffer[7] == 0x01)
            {
                std::cout<<"init finish"<<std::endl;
                break; 
            }
        }
    }
    ros::spin();

    return 0;


}