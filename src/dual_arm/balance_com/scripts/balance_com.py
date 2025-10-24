import time
import serial
from modbus_tk import modbus_rtu
import rospy
from gripper_modbus.srv import Balance, BalanceRequest, BalanceResponse
from std_msgs.msg import Float64


class Gripper_ctrl:
    def __init__(self, serial_port):
        self.master_Balance = modbus_rtu.RtuMaster(
            serial.Serial(port=serial_port, baudrate=9600, bytesize=8, parity='N', stopbits=1, rtscts=False, dsrdtr=True)
        )
        self.master_Balance.set_timeout(0.2)
        self.master_Balance.set_verbose(True)

    def Balance_tare(self):
        """去皮功能实现"""
        try:
            print(f'去皮重量: {self.Balance_weight():.1f} mg')
            # 按照去皮功能码及地址等设置
            request_data = self.master_Balance.execute(
                0x15,  # 站号1
                0x06,  # 功能码 0x06
                0x0006,  # 寄存器起始地址
                output_value=1
            )
            print(f"去皮操作请求数据: {request_data}")
            if abs(self.Balance_weight())>3:
                return self.Balance_tare()
            return True
        except Exception as e:
            rospy.logerr(f"去皮操作失败: {e}")
            return False

    def Balance_weight(self):
        """读取重量功能实现"""
        try:
            # 按照读取重量功能码及地址等设置
            last_value=-1000000
            time.sleep(2)
            same_count=0
            while not rospy.is_shutdown():
                result = self.master_Balance.execute(
                    0x15,  # 站号1
                    0x03,  # 功能码 0x03
                    0x0000,  # 寄存器起始地址
                    0x0002  # 寄存器数量
                )
                # 解析结果，这里假设结果处理逻辑
                weight_value = (result[0] << 16) + result[1]
                if weight_value & 0x80000000:  # 检查最高位是否为1
                    # 转换为有符号整数
                    weight_value = -(0x100000000 - weight_value)
                weight_value = round(weight_value / 10.0,1)  # 假设单位转换，具体按实际
                if abs(weight_value-last_value)<0.2:
                    same_count+=1
                    if same_count>3:
                        break
                else:
                    same_count=0
                last_value=weight_value
                # print(f"重量(变动): {weight_value:.1f}")
                time.sleep(0.2)
            print(f">>>重量(确认): {weight_value:.1f}")
            return weight_value
        except Exception as e:
            rospy.logerr(f"读取重量操作失败: {e}")
            return -1

    def doReq(self, req):
        """服务处理函数"""
        if req.operation == "tare":
            success = self.Balance_tare()
            res = BalanceResponse()
            res.success = success
            return res
        elif req.operation == "weight":
            weight = self.Balance_weight()
            res = BalanceResponse()
            res.weight = weight
            return res


if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("Balance_com", anonymous=True)
    # 假设实际串口设备路径，这里按示例修改
    # balance_ctrl = Gripper_ctrl('/dev/ttyUSB5')
    balance_ctrl = Gripper_ctrl('/dev/BALANCE')
    # 3.创建服务对象
    balance_server = rospy.Service("/Balance_com", Balance, balance_ctrl.doReq)
    print("------- Balance Server has started--------")
    # 4.回调函数处理请求并产生响应
    # 5.spin 函数
    while True:
        print(balance_ctrl.Balance_weight())
        time.sleep(1)
    rospy.spin()
