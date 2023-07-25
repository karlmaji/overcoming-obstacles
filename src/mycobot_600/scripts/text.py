from socket import *
import math
import sys
import time
from multiprocessing import Lock
import time
import rospy
from sensor_msgs.msg import JointState
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
            self.tcp_client.connect(self.ADDR)
            print("strat ok")
            return ""
        except:
            print("tcp error")
            print(self.ADDR)

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
                return invalid_coords()
        return invalid_coords()

    def string_to_double(self, data):
        try:
            val = float(data)
            return val
        except:
            return -9999.99

    def string_to_int(self, data):
        try:
            val = int(data)
            return val
        except:
            return -9999

    def invalid_coords(self):
        coords = [-1, -2, -3, -4, -1, -1]
        return coords

    def get_angles(self):
        command = "get_angles()\n"
        res = self.send_command(command)
        return self.string_to_coords(res)

    def get_coords(self):
        command = "get_coords()\n"
        res = self.send_command(command)
        return self.string_to_coords(res)

    def get_speed(self):
        command = "get_speed()\n"
        res = self.send_command(command)
        return self.string_to_double(res)

    def power_on(self):
        command = "power_on()\n"
        res = self.send_command(command)
        return True

    def power_off(self):
        command = "power_off()\n"
        res = self.send_command(command)
        return True

    def check_running(self):
        command = "check_running()\n"
        res = self.send_command(command)
        return res == "1"

    def state_check(self):
        command = "state_check()\n"
        res = self.send_command(command)
        return res == "1"

    def program_open(self, file_path):
        command = "program_open(" + file_path + ")\n"
        res = self.send_command(command)
        return self.string_to_int(res)

    def program_run(self, start_line):
        """run program，运行程序"""
        command = "program_run(" + str(start_line) + ")\n"
        res = self.send_command(command)
        return self.string_to_int(res)

    def read_next_error(self):
        command = "read_next_error()\n"
        res = self.send_command(command)
        return res

    def write_coords(self, coords, speed):
        """set coords,设置坐标"""
        command = "set_coords("
        for item in coords:
            command += str(item) + ","
        command += str(speed) + ")\n"
        self.send_command(command)

    def write_coord(self, axis, value, speed):
        coords = self.get_coords()
        if coords != self.invalid_coords():
            coords[axis] = value
            self.write_coords(coords, speed)

    def write_angles(self, angles, speed):
        """set angles,设置角度"""
        command = "set_angles("
        for item in angles:
            command += str(item) + ","
        command += str(speed) + ")\n"
        self.send_command(command)

    def write_angle(self, joint, value, speed):
        angles = self.get_angles()
        if angles != self.invalid_coords():
            angles[joint] = value
            self.write_angles(angles, speed)

    def set_speed(self, percentage):
        command = "set_speed(" + str(percentage) + ")\n"
        self.send_command(command)

    def set_carte_torque_limit(self, axis_str, value):
        command = "set_torque_limit(" + axis_str + "," + str(value) + ")\n"
        self.send_command(command)

    def set_upside_down(self, up_down):
        up = "1"
        if up_down:
            up = "0"
        command = "set_upside_down(" + up + ")\n"
        self.send_command(command)

    def set_payload(self, payload):
        command = "set_speed(" + str(payload) + ")\n"
        self.send_command(command)

    def state_on(self):
        command = "state_on()\n"
        self.send_command(command)

    def state_off(self):
        command = "state_off()\n"
        self.send_command(command)

    def task_stop(self):
        command = "task_stop()\n"
        self.send_command(command)

    def jog_angle(self, joint_str, direction, speed):
        command = (
            "jog_angle(" + joint_str + "," + str(direction) + "," + str(speed) + ")\n"
        )
        self.send_command(command)

    def jog_coord(self, axis_str, direction, speed):
        command = (
            "jog_coord(" + axis_str + "," + str(direction) + "," + str(speed) + ")\n"
        )
        self.send_command(command)
        print(command)

    def get_digital_in(self, pin_number):
        command = "get_digital_in(" + str(pin_number) + ")\n"
        self.send_command(command)

    def get_digital_out(self, pin_number):
        command = "get_digital_out(" + str(pin_number) + ")\n"
        self.send_command(command)

    def set_digital_out(self, pin_number, pin_signal):
        command = "set_digital_out(" + str(pin_number) + "," + str(pin_signal) + ")\n"
        self.send_command(command)

    def set_analog_out(self, pin_number, pin_signal):
        command = "set_analog_out(" + str(pin_number) + "," + str(pin_signal) + ")\n"
        self.send_command(command)

    def get_acceleration(self):
        command = "get_acceleration()\n"
        res = self.send_command(command)
        return self.string_to_int(res)

    def set_acceleration(self, acceleration):
        command = "set_acceleration(" + str(acceleration) + ")\n"
        self.send_command(command)

    def command_wait_done(self):
        command = "wait_command_done()\n"
        self.send_command(command)

    def wait(self, seconds):
        command = "wait(" + str(seconds) + ")\n"
        self.send_command(command)

    def assign_variable(self, var_name, var_value):
        command = 'assign_variable("' + str(var_name) + '",' + str(var_value) + ")\n"
        self.send_command(command)

    def get_variable(self, var_name):
        command = 'get_variable("' + str(var_name) + '")\n'
        return self.send_command(command)

global mc
ip = "10.42.0.35"
print (ip)
mc = ElephantRobot(ip, 5001)
# START CLIENT,启动客户端
res = mc.start_client()
if res != "":
    sys.exit(1)
# mc.power_on()
# time.sleep(3)
# mc.state_on()
# time.sleep(3)
#mc.set_digital_out(16,1)
#mc.get_digital_out(16)
#mc.write_angles([-1.604186,-7.369690,-70.204758,-156,-89.648437,23.642578],1500)
#mc.write_coords([0,100,-83.252605,-139.570312,-87.099609,26.015625],1500)
#mc.jog_coord('z',0,500)
#mc.write_coord(0,50.5,1999)
# print mc.get_coords()
#mc.set_speed(150)
coords=mc.get_coords()
angels=mc.get_angles()
#mc.jog_angle('J1',0,100)
#mc.write_angles([0,-130,20,30,-20,0],1500)

# while 1:
#     inword=input("请输入：")
#     print(inword)
#     if inword=='q':
#         angels[0]=angels[0]+5
#         mc.write_angles(angels,1500)
#     elif inword=='w':
#         angels[0]=angels[0]-5
#         mc.write_angles(angels,1500)
    
#     elif inword=='e':
#         angels[1]=angels[1]+5
#         mc.write_angles(angels,1500)
#     elif inword=='r':
#         angels[1]=angels[1]-5
#         mc.write_angles(angels,1500)
        
#     elif inword=='a':
#         angels[2]=angels[2]+5
#         mc.write_angles(angels,1500)
#     elif inword=='s':
#         angels[2]=angels[2]-5
#         mc.write_angles(angels,1500)
        
#     elif inword=='d':
#         angels[3]=angels[3]+5
#         mc.write_angles(angels,1500)
#     elif inword=='f':
#         angels[3]=angels[3]-5
#         mc.write_angles(angels,1500)
    
#     elif inword=='z':
#         angels[4]=angels[4]+5
#         mc.write_angles(angels,1500)
#     elif inword=='x':
#         angels[4]=angels[4]-5
#         mc.write_angles(angels,1500)
        
#     elif inword=='c':
#         angels[5]=angels[5]+5
#         mc.write_angles(angels,1500)
#     elif inword=='v':
#         angels[5]=angels[5]-5
#         mc.write_angles(angels,1500)
#     mc.get_angles()
#     mc.get_coords()


