import rospy
from socket import *
from multiprocessing import Lock
import sys
import math
from sensor_msgs.msg import Joy,JointState
global ER
mutex = Lock()
J_angels_max = [180.00,90.00,150.00,80.00,168.00,174.00]
J_angels_min = [-180.00,-270.00,-150.00,-260.00,-168.00,-174.00]
control_index = 0
control_begin = False
joy_ago = None
J_speed = []
virtual_angles = []
real_angles = []
A_press = False
B_press = False

class ElephantRobot(object):
    def __init__(self, host, port):
        # setup connection
        # 建立连接
        self.BUFFSIZE = 2048
        self.ADDR = (host, port)
        self.tcp_client = socket(AF_INET, SOCK_STREAM)
    def start_client(self):
        try:
            print(f"begin to connect {self.ADDR}")
            self.tcp_client.connect(self.ADDR)
            print("strat ok")
            return ""
        except:
            print("tcp error")
            print(self.ADDR)
            return "error"

    def stop_client(self):
        self.tcp_client.close()
        
    def send_command(self, command):
        with mutex:
            self.tcp_client.send(command.encode())
            recv_data = self.tcp_client.recv(self.BUFFSIZE).decode()
            res_str = str(recv_data)
            print("recv = " + res_str)
            res_arr = res_str.split(":")
            if len(res_arr) == 2:
                return res_arr[1]
            else:
                return ""
    def get_angles(self):
        command = "get_angles()\n"
        res = self.send_command(command)
        return self.string_to_coords(res)
    def string_to_coords(self, data):
        data = data.replace("[", "")
        data = data.replace("]", "")
        data_arr = data.split(",")
        if len(data_arr) == 6:
            try:
                coords_1 = float(data_arr[0])
                coords_2 = float(data_arr[1])
                coords_3 = float(data_arr[2])
                coords_4 = float(data_arr[3])
                coords_5 = float(data_arr[4])
                coords_6 = float(data_arr[5])
                coords = [coords_1, coords_2, coords_3, coords_4, coords_5, coords_6]
                return coords
            except:
                return self.invalid_coords()
        return self.invalid_coords()
    def invalid_coords(self):
        coords = [-1, -2, -3, -4, -1, -1]
        return coords
    def jog_angle(self, joint_str, direction, speed):
        command = (
            "jog_angle(" + joint_str + "," + str(direction) + "," + str(speed) + ")\n"
        )
        self.send_command(command)
    def write_angles(self, angles, speed):
        """set angles,设置角度"""
        command = "set_angles("
        for item in angles:
            command += str(item) + ","
        command += str(speed) + ")\n"
        self.send_command(command)
    def command_wait_done(self):
        command = "wait_command_done()\n"
        self.send_command(command)

 
def joy_callback(joy):
    global joy_ago
    global control_begin
    global J_speed,A_press,B_press
    if joy_ago != None and control_begin:
        J1_speed = joy.axes[0]
        J2_speed = joy.axes[1]
        J3_speed = joy.axes[3]
        J4_speed = joy.axes[4]
        J5_speed = -joy.axes[6]
        J6_speed = joy.axes[7]
        J_speed = [J1_speed,J2_speed,J3_speed,\
                   J4_speed,J5_speed,J6_speed]
        
        if abs(joy_ago.buttons[0]-0)<1e-3 and abs(joy.buttons[0]-1)<1e-3:
            A_press = True
        if abs(joy_ago.buttons[1]-0)<1e-3 and abs(joy.buttons[1]-1)<1e-3:
            B_press = True

    elif(abs(joy.axes[2] -1.0)<1e-2 and abs(joy.axes[5] -1.0)<1e-2):
        control_begin = True
        print("control_open")
    
    joy_ago = joy
    
    
    
    
def check_angles(angles_ls):
    # 更新角度
    error_angles = [-1, -2, -3, -4, -1, -1]
    if len(angles_ls)==6:
        count = 0
        for i in range(len(error_angles)):
            if math.isclose(angles_ls[i],error_angles[i],rel_tol=1e-2):
                count=count +1
        if count != len(error_angles):
            return True
    
    return False
    

        
if __name__== '__main__':

    rospy.init_node("mycobot_control_node",anonymous=True)
    
    
    ip = rospy.get_param("~ip", "10.42.0.35")
    ER = ElephantRobot(ip, 5001)
    
    # START CLIENT,启动客户端
    res = ER.start_client()
    if res != "":
        print ('start_client->',res)
        sys.exit(1)
    else:
        print(res)
        
    js  = ["joint2_to_joint1","joint3_to_joint2","joint4_to_joint3",\
                        "joint5_to_joint4","joint6_to_joint5","joint6output_to_joint6"]
    joint_state_real = JointState()
    joint_state_virtual = JointState()
    
    joint_state_virtual.name = ["virtual_robot/" + iter for iter in js]
    joint_state_real.name = ["real_robot/" + iter for iter in js]
    
    pub_real = rospy.Publisher("real_robot/joint_states",JointState,queue_size=1)
    pub_virtual = rospy.Publisher("virtual_robot/joint_states",JointState,queue_size=1)
    
    sub = rospy.Subscriber("joy",Joy,joy_callback)
    loop_rate =rospy.Rate(10)
    


        
    angles_check = ER.get_angles()
    if check_angles(angles_check):
        virtual_angles = angles_check
        real_angles = angles_check
        
        joint_state_real.position = [iter*math.pi/180 for iter in angles]
        joint_state_real.header.stamp = rospy.Time().now()
        pub_real.publish(joint_state_real)

        joint_state_virtual.position = [iter*math.pi/180 for iter in angles]
        joint_state_virtual.header.stamp = rospy.Time().now()
        pub_virtual.publish(joint_state_virtual)
    else:
        print("获取机械臂初始角度失败！")
        virtual_angles = [0,0,0,0,0,0]
        real_angles = [0,0,0,0,0,0]
    # virtual_angles = [0,0,0,0,0,0]
    loop_1_s = 0
    while not rospy.is_shutdown():
        
        if loop_1_s<10:
            loop_1_s = loop_1_s + 1
        else:
            loop_1_s=0
            angles_check = ER.get_angles()
            if check_angles(angles_check):
                real_angles = angles_check
                joint_state_real.position = [iter*math.pi/180 for iter in real_angles]
                joint_state_real.header.stamp = rospy.Time().now()
                pub_real.publish(joint_state_real)
            

            
        if len(virtual_angles)==6 and len(J_speed)!=0:
            for i in range(len(virtual_angles)):
                virtual_angles[i] = virtual_angles[i] + 1*J_speed[i]
                if virtual_angles[i]> J_angels_max[i]:
                    virtual_angles[i] = J_angels_max[i]
                if virtual_angles[i]<J_angels_min[i]:
                    virtual_angles[i] = J_angels_min[i]
            print(virtual_angles)
            joint_state_virtual.position = [iter*math.pi/180 for iter in virtual_angles]
            joint_state_virtual.header.stamp = rospy.Time().now()
            pub_virtual.publish(joint_state_virtual)
        
        
        if A_press:
            ER.write_angles(virtual_angles,1999)
            ER.command_wait_done()
            A_press = False
            
        if B_press:
            virtual_angles = real_angles
            joint_state_virtual.position = [iter*math.pi/180 for iter in virtual_angles]
            joint_state_virtual.header.stamp = rospy.Time().now()
            pub_virtual.publish(joint_state_virtual)
            B_press = False
            
        loop_rate.sleep()

        
        
        
    rospy.spin()
    

    
    
