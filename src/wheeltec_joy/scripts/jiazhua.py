import serial
import time

def seral_init():
    ser = serial.Serial("/dev/ttyACM0",115200)
    # ser.port='/dev/ttyACM0'
    # ser.baudrate=115200
    # ser.bytesize = serial.EIGHTBITS  # 数据位
    # ser.stopbits = serial.STOPBITS_ONE  # 停止位
    # ser.parity = serial.PARITY_NONE  # 校验位
    #ser.open()
    if ser.is_open:
        data = b'FFFEFDFC010802010000000000FB'
        ser.write(data)
        while True:
            time.sleep(0.5)
            data=b'FFFEFDFC010802000000000000FB'
            ser.write(data)
            rev=ser.read_all()
            print(rev)
            if str(rev)[30:31]=='1':
                print("init finish")
                break             
    return ser
def ten_to_hex(val):
    value = hex(val)
    interval = value[2:].upper()
    if val<16:
        interval='0'+interval
    return interval

#夹持力 20-100
def force(ser,value):
    data='FFFEFDFC0105020100'+ten_to_hex(value)+'000000FB'
    data=bytes(data,encoding='utf-8')
    ser.write(data)
    rev=ser.read_all()
    return rev
#位置 0-100   
def position(ser,value):
    data='FFFEFDFC0106020100'+ten_to_hex(value)+'000000FB'
    #print(data)
    data=bytes(data,encoding='utf-8')
    ser.write(data)
    rev=ser.read_all()
    return rev
    #print(rev)

def feedback(ser):
    data='FFFEFDFC010F01000000000000FB'
    data=bytes(data,encoding='utf-8')
    ser.write(data)
    rev=ser.read_all()
    return rev
if __name__ == '__main__':
    ser=seral_init()
    time.sleep(1)
   # force(ser,30)
   # time.sleep(0.5)
    position(ser,0)
    #time.sleep(2)
    #print(feedback(ser))
    time.sleep(0.5)
    print(feedback(ser))
    # print(feedback(ser,100))
    
   
    ser.close()

    
    
    
