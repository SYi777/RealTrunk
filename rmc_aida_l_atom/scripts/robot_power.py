import serial
import time
import ini_parameter
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
# 读取配置文件
config = ini_parameter.RobotConfig(f'{ script_dir }/config.ini')


def chassis_power_control(port, baudrate, timeout: int=2, power: bool=True):
    print(hex(int(power)), "\t", type(power))
    try:
        with serial.Serial(port=port, baudrate=baudrate, timeout=timeout) as ser:
            command = bytes([0xee, 0x00, int(power), 0xec])
            # 写入命令
            ser.write(command)
            print(f"命令已发送：{command}")

            # 读取响应
            response = ser.readline()  # 或根据实际需求选择合适的读取方法
            print(f"收到响应：{response}")

            if response == b'\xf5\x07\x08\xf3':
                return True
            else:
                return False

    except serial.SerialException as e:
        print(f"串口通信错误：{e}")
    except Exception as e:
        print(f"发生错误：{e}")


if __name__ == "__main__":
    while True:

        if chassis_power_control(config.Robot_Power.port, config.Robot_Power.baudrate, power=True):
            break
        else:
            time.sleep(1)