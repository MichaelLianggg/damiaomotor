# 方式1：调用函数接口打开串口时传入配置参数
import serial
 
ser = serial.Serial("/dev/ttyACM0", 115200)    # 打开COM17，将波特率配置为115200，其余参数使用默认值
if ser.isOpen():                        # 判断串口是否成功打开
    print("打开串口成功。")
    print(ser.name)    # 输出串口号
else:
    print("打开串口失败。")
 