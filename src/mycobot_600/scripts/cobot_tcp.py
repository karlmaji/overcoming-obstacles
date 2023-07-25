import rospy
from sensor_msgs.msg import JointState
from multiprocessing import Lock
from socket import *
from std_msgs.msg import String
import math
import sys

global ER

# 是否每秒 发送真实关节状态
send_joints = True

mutex = Lock()

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
    def power_on(self):
        command = "power_on()\n"
        res = self.send_command(command)
        return True
    def power_off(self):
        command = "power_off()\n"
        res = self.send_command(command)
        return True
    def state_on(self):
        command = "state_on()\n"
        self.send_command(command)

    def state_off(self):
        command = "state_off()\n"
        self.send_command(command)
        
    def check_running(self):
        command = "check_running()\n"
        res = self.send_command(command)
        return res == "1"
    def state_check(self):
        command = "state_check()\n"
        res = self.send_command(command)
        return res == "1"

def check_angles(angles_ls):
    # 检查角度
    error_angles = [-1, -2, -3, -4, -1, -1]
    if len(angles_ls)==6:
        count = 0
        for i in range(len(error_angles)):
            if math.isclose(angles_ls[i],error_angles[i],rel_tol=1e-2):
                count=count +1
        if count != len(error_angles):
            return True
    
    return False

# 控制真实机械臂关节
old_list=[]
J_angels_max = [180.00,90.00,150.00,80.00,168.00,174.00]
J_angels_min = [-180.00,-270.00,-150.00,-260.00,-168.00,-174.00]


def js_callback(data):
    """callback function,回调函数"""
    global old_list
    print ("position", data.position)
    data_list = []
    for index, value in enumerate(data.position):
        value = value * 180 / math.pi
        
        if value> J_angels_max[index]:
            value = J_angels_max[index]
        if value<J_angels_min[index]:
            value = J_angels_min[index]
            
        data_list.append(value)
    print ("data", data_list)

    if not old_list:
        old_list = data_list
        ER.write_angles(data_list, 1999)
        #ER.command_wait_done()
    elif old_list != data_list:
        old_list = data_list
        ER.write_angles(data_list, 1999)
        #ER.command_wait_done()

def cmd_callback(string_data):
    global send_joints
    command = string_data.data
    if command == "begin_send_joints()\n":
        send_joints = True
    elif command == "stop_send_joints()\n":
        send_joints = False
    else:
        res = ER.send_command(command)
        print(f"{command}\r res:{res}")
    
if __name__ =="__main__":
    rospy.init_node("cobot_tcp",anonymous=True)
    
    ip = rospy.get_param("~ip", "10.42.0.35")
    ER = ElephantRobot(ip, 5001)
    
    # START CLIENT,启动客户端
    res = ER.start_client()
    if res != "":
        print ('start_client->',res)
        sys.exit(1)
    else:
        print(res)
    
    sub = rospy.Subscriber("set_cobot_joints",JointState,js_callback)
    cmd_sub = rospy.Subscriber("cobot_cmd",String,cmd_callback)
    
    loop_rate =rospy.Rate(1)
    
    
    joint_state_real = JointState()
    js  = ["joint2_to_joint1","joint3_to_joint2","joint4_to_joint3",\
                        "joint5_to_joint4","joint6_to_joint5","joint6output_to_joint6"]
    joint_state_real.name = ["real_robot/" + iter for iter in js]
    pub_real = rospy.Publisher("real_robot/joint_states",JointState,queue_size=1)

    # 每秒发送机械臂关节状态
    while not rospy.is_shutdown():
        if send_joints:
            angles_check = ER.get_angles()
            if check_angles(angles_check):
                real_angles = angles_check
                joint_state_real.position = [iter*math.pi/180 for iter in real_angles]
                joint_state_real.header.stamp = rospy.Time().now()
                pub_real.publish(joint_state_real)
        
        
        loop_rate.sleep()
    
    