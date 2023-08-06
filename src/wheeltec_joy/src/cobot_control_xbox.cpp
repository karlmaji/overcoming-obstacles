#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/Joy.h>
#include<std_msgs/String.h>
#include<std_msgs/Bool.h>
class cobot_control_xbox
{
    public:
        cobot_control_xbox();
        float speed;
        int loop_rate;
        void loop_function();
    private:
        ros::NodeHandle n;
        ros::Publisher cmd_pub;
        ros::Publisher virtual_cobot_pub;
        ros::Publisher set_cobot_pub;
        ros::Publisher grap_control_pub;
        ros::Subscriber joy_sub;

        sensor_msgs::Joy joy_ago;
        std::vector<double> J_speed;
        sensor_msgs::JointState js;
        bool control_begin;
        bool A_press;
        bool B_press;
        bool X_press;
        bool Y_press;
        bool LT_press;
        bool RT_press;
        bool speed_up;
        bool speed_down;
        bool grap_control;

        std::vector<double> virtual_angles={0,0,0,0,0,0};
        std::vector<double> quansuo_angles={0.782557,-92.0187,150,-245.061,4.25,3.25};
        std::vector<double> zhuaqu_angles={-3.3829,-77.356,56.8438,-162.793,-86.6055,0.515625};
        std::vector<double> J_angels_max={180.00,90.00,150.00,80.00,168.00,174.00};
        std::vector<double> J_angels_min={-180.00,-270.00,-150.00,-260.00,-168.00,-174.00};

        void joy_callbk(const sensor_msgs::JoyConstPtr& joy);
        



};

cobot_control_xbox::cobot_control_xbox()
{
    ros::NodeHandle private_nh("~"); //创建节点句柄
    private_nh.param<float>("speed",speed,1); //默认axes[0]接收x速度
    private_nh.param<int>("loop_rate",loop_rate,10); //默认axes[0]接收x速度

    cmd_pub = n.advertise<std_msgs::String>("cobot_cmd",1);
    virtual_cobot_pub = n.advertise<sensor_msgs::JointState>("virtual_robot/joint_states",1);
    set_cobot_pub = n.advertise<sensor_msgs::JointState>("set_cobot_joints",1);
    grap_control_pub = n.advertise<std_msgs::Bool>("grap_control",1);
    joy_sub = n.subscribe<sensor_msgs::Joy>("joy",1,&cobot_control_xbox::joy_callbk,this);


    ros::V_string vs = {"virtual_robot/joint2_to_joint1","virtual_robot/joint3_to_joint2",\
                        "virtual_robot/joint4_to_joint3","virtual_robot/joint5_to_joint4",\
                        "virtual_robot/joint6_to_joint5","virtual_robot/joint6output_to_joint6"};
    js.name = vs;
    
    
    control_begin = false;
    A_press = false;
    B_press = false;
    X_press = false;
    Y_press = false;
    speed_up = false;
    speed_down = false;
    LT_press = false;

    grap_control = false;
    RT_press = false;

}

void cobot_control_xbox::joy_callbk(const sensor_msgs::JoyConstPtr& joy)
{
    if(joy_ago.axes.size()==0 ||joy_ago.buttons.size()==0) joy_ago=*joy;
    else
    {
        if(abs(joy_ago.buttons[7]<1e-3) && abs(joy->buttons[7]-1)<1e-3)
        {
            control_begin = !control_begin;
            if(control_begin) ROS_INFO("cobot_control_open!"); else
            ROS_INFO("cobot_control_close!");
        }
        else if (abs(joy_ago.buttons[6]<1e-3) && abs(joy->buttons[6]-1)<1e-3)
        {
        control_begin = false;
        
        }
        else if (abs(joy_ago.buttons[8]<1e-3) && abs(joy->buttons[8]-1)<1e-3)
        {
        control_begin = false;
        
        }
        

        if(control_begin)
        {   
            J_speed.clear();
            J_speed.push_back(joy->axes[0]);
            J_speed.push_back(joy->axes[1]);
            J_speed.push_back(joy->axes[3]);
            J_speed.push_back(joy->axes[4]);
            J_speed.push_back(-joy->axes[6]);
            J_speed.push_back(joy->axes[7]);
            if(abs(joy_ago.buttons[0]<1e-3) && abs(joy->buttons[0]-1)<1e-3)
            {
                A_press = true;

            }
            if(abs(joy_ago.buttons[1]<1e-3) && abs(joy->buttons[1]-1)<1e-3)
            {
                B_press = true;
            }
            if(abs(joy_ago.buttons[4]<1e-3) && abs(joy->buttons[4]-1)<1e-3)
            {
                speed_down = true;
            }
            if(abs(joy_ago.buttons[5]<1e-3) && abs(joy->buttons[5]-1)<1e-3)
            {
                speed_up = true;
            }
            if(abs(joy_ago.buttons[2]<1e-3) && abs(joy->buttons[2]-1)<1e-3)
            {
                X_press = true;
            }
            if(abs(joy_ago.buttons[3]<1e-3) && abs(joy->buttons[3]-1)<1e-3)
            {
                Y_press = true;
            }
            if(abs(joy_ago.buttons[9]<1e-3) && abs(joy->buttons[9]-1)<1e-3)
            {
                LT_press = true;
            }
            if(abs(joy_ago.buttons[10]<1e-3) && abs(joy->buttons[10]-1)<1e-3)
            {
                RT_press = true;
            }
        }

        

        joy_ago = *joy;
    }
}

void cobot_control_xbox::loop_function()
{
    if(control_begin)
    {
    if(J_speed.size()!=0)
    {
        std::cout<<"virtual_angels:";
        js.position.clear();

        for(int i=0;i<virtual_angles.size();i++)
        {
            virtual_angles[i] = virtual_angles[i] + speed *J_speed[i];
            if(virtual_angles[i]> J_angels_max[i])
                virtual_angles[i] = J_angels_max[i];
            if(virtual_angles[i]<J_angels_min[i])
                virtual_angles[i] = J_angels_min[i];
            std::cout<<virtual_angles[i]<<",";

            js.position.push_back(virtual_angles[i] * M_PI /180);

        }
        std::cout<<std::endl;
        js.header.stamp = ros::Time::now();
        virtual_cobot_pub.publish(js);
    }

    if(A_press)
    {
        A_press=false;
        set_cobot_pub.publish(js);
        
    }
    if(B_press)
    {
        B_press = false;
        boost::shared_ptr<sensor_msgs::JointState const> real_js;
        real_js = ros::topic::waitForMessage<sensor_msgs::JointState>("real_robot/joint_states",ros::Duration(2));
        if(real_js)
        {
            for(int i=0;i<virtual_angles.size();i++)
            {
                virtual_angles[i] = real_js->position[i] *180/M_PI;
                
                js.position = real_js->position;
                virtual_cobot_pub.publish(js);
            }
        }
        
    }
    if(speed_up)
    {
        speed = speed + 0.25;
        ROS_INFO("speed:%2f",speed);
        speed_up = false;
    }
    if(speed_down)
    {
        speed = speed - 0.25;
        speed_down = false;
        ROS_INFO("speed:%2f",speed);
    }
    if(X_press)
    {
        X_press = false;
        js.position.clear();
        for(int i=0;i<virtual_angles.size();i++)
        {
            virtual_angles[i] = quansuo_angles[i];
            
            js.position.push_back(virtual_angles[i] * M_PI /180);
        }
        js.header.stamp = ros::Time::now();
        virtual_cobot_pub.publish(js);

    }
    if(Y_press)
    {
        Y_press = false;
        js.position.clear();
        for(int i=0;i<virtual_angles.size();i++)
        {
            virtual_angles[i] = zhuaqu_angles[i];
            
            js.position.push_back(virtual_angles[i] * M_PI /180);
        }
        js.header.stamp = ros::Time::now();
        virtual_cobot_pub.publish(js);

    }
    if(LT_press)
    {
        LT_press = false;
        std_msgs::Bool bl;
        bl.data = grap_control;
        grap_control_pub.publish(bl);

        
        grap_control=!grap_control;

    }
    if(RT_press)
    {   
        ros::Rate d_5(5);
        RT_press = false;
        std_msgs::String st;
        st.data = "power_on()\n";
        cmd_pub.publish(st);

        d_5.sleep();
        

        st.data = "state_on()\n";
        cmd_pub.publish(st);

    }

    }


}

int main(int argc,char** argv)
{
  ros::init(argc, argv, "cobot_control_xbox");
  ros::Time::init();
  cobot_control_xbox control;
  ros::Rate rate(control.loop_rate);
  
  ros::AsyncSpinner s(1);
  s.start();
  while(ros::ok())
  {
    control.loop_function();
    rate.sleep();
  }

  
  return 0;

} 
