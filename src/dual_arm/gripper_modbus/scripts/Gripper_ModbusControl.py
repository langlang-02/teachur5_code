#!/usr/bin/python
# 需要安装 modbus_tk 模块
# pip install modbus_tk
import time
import serial
from modbus_tk import modbus_rtu
import rospy
from gripper_modbus.srv import Gripper, GripperRequest, GripperResponse
from modbus_tk.exceptions import ModbusInvalidResponseError, ModbusError


# -------------------------- Modbus地址定义（统一管理） --------------------------
# 通用地址（PGI和RGI共用）
MODBUS_ADDR = {
    # 写入地址（输出寄存器）
    "INIT_CMD": 256,          # 初始化命令：1=回零，165=标定
    "INIT_DIR": 769,          # 初始化方向：1=张开最大，0=闭合最小
    "FORCE": 257,    # 平行力值（20-100%）
    "POSITION": 259,          # 位置（0-1000千分比）
    "VELOCITY": 260, # 平行速度（1-100%）
    
    # 读取地址（输入寄存器）
    "INIT_STATE": 512,        # 初始化状态：0=未初始化，1=已初始化
    "CLAMP_STATE": 513,       # 夹持状态：0=运动中，1=到达位置，2=夹住物体，3=物体掉落
    "POSITION_ACTUAL": 514,    # 实际夹持位置：0-1000
    
    # RGI专用地址（旋转功能）
    "ABS_ANGLE_LOW": 261,     # 绝对角度低位（-32768~32767）
    "ABS_ANGLE_HIGH": 262,    # 绝对角度高位（-160~160）
    "SPEED": 263,   # 旋转速度（1-100%）
    "TORQUE": 264,      # 旋转力值（20-100%）
    "REL_ANGLE": 265,         # 相对角度（-32768~32767）
    "ROTATE_STOP": 1282,      # 旋转停止命令：1=强制停止
    
    # RGI专用读取地址
    "ROTATE_ANGLE_LOW": 261,  # 旋转角度反馈低位
    "ROTATE_ANGLE_HIGH": 262, # 旋转角度反馈高位
    "ROTATE_STATE": 523       # 旋转状态：0=运动中，1=到达位置，2=堵转
}

# 状态码文本映射
STATE_TEXT = {
    "CLAMP": {
        0: "running",    # 运动中
        1: "idle",       # 到达位置
        2: "clamping",   # 夹住物体
        3: "idle"       # 物体掉落（故障）!!!!!!!
    },
    "ROTATE": {
        0: "running",    # 运动中
        1: "idle",       # 到达位置
        2: "blocking",   # 堵转
        3: "blocking"    # 堵转
    },
    "SPECIAL": {
        "offline": "offline",                  # 未连接
        "clamping&blocking": "clamping&blocking" # 夹持且堵转
    }
}

# 默认响应值
DEFAULT_RESPONSE = {
    "position": -1,
    "set_position": -1,
    "force": -1,
    "velocity": -1,
    "abs_angle": 99999999,
    "torque": -1,
    "speed": -1,
    "status": STATE_TEXT["SPECIAL"]["offline"]
}
# --------------------------------------------------------------------------------


class GripperCtrl:
    def __init__(self, port, slave, type_):
        self.port = port
        self.slave = slave
        self.type = type_  # RGI 或 PGI
        self.master = None
        self.connected = False
        self.set_pos=-1
        self.force=-1
        self.velocity=-1
        self.torque=-1
        self.speed=-1
        # 初始连接
        self._connect()

    def _connect(self):
        """建立Modbus RTU连接"""
        try:
            if self.master:
                self.master.close()
            self.master = modbus_rtu.RtuMaster(
                serial.Serial(
                    port=self.port,
                    baudrate=115200,
                    bytesize=8,
                    parity='N',
                    stopbits=1,
                    xonxoff=0,
                    timeout=0.5
                )
            )
            self.master.set_timeout(0.5)
            self.master.set_verbose(False)
            self.connected = True
            rospy.loginfo(f"{self.type} 夹爪已连接至 {self.port} (slave={self.slave})")
            return True
        except Exception as e:
            rospy.logerr(f"{self.type} 夹爪连接失败: {str(e)}")
            self.connected = False
            self.master = None
            return False

    def _read_register(self, addr):
        """读取寄存器值，最多尝试两次"""
        # if addr==512:
        #     return 0
        max_attempts = 4
        for attempt in range(max_attempts):
            if not self.connected and not self._connect():
                # 连接失败，尝试下一次
                if attempt < max_attempts - 1:
                    rospy.loginfo(f"读取重试 {attempt + 1}/{max_attempts} (addr={addr})")
                    continue
                else:
                    rospy.loginfo(f"放弃读取 {attempt + 1}/{max_attempts} (addr={addr})")
                    return -1
                    
            try:
                result = self.master.execute(
                    slave=self.slave,
                    function_code=3,
                    starting_address=addr,
                    quantity_of_x=1
                )
                # 读取成功，重置连接状态为正常
                self.connected = True
                # rospy.loginfo(f"读取寄存器:{addr} {result}")
                return result[0]
            except (ModbusInvalidResponseError, ModbusError) as e:
                rospy.logwarn(f"寄存器读取失败 (addr={addr}, 尝试 {attempt + 1}/{max_attempts}): {str(e)}")
                self.connected = False
                # 如果不是最后一次尝试，等待片刻后重试
                if attempt < max_attempts - 1:
                    time.sleep(0.01)
                    continue
                return -1

    def _write_register(self, addr, value):
        """写入寄存器值，最多尝试两次"""
        # rospy.loginfo(f"写入寄存器:{value}->{addr}")
        max_attempts = 4
        for attempt in range(max_attempts):
            if not self.connected and not self._connect():
                # 连接失败，尝试下一次
                if attempt < max_attempts - 1:
                    rospy.loginfo(f"写入重试 {attempt + 1}/{max_attempts} (addr={addr})")
                    continue
                else:
                    rospy.loginfo(f"放弃写入 {attempt + 1}/{max_attempts} (addr={addr})")
                    return False
                    
            try:
                self.master.execute(
                    slave=self.slave,
                    function_code=6,
                    starting_address=addr,
                    output_value=value
                )
                # 写入成功，重置连接状态为正常
                self.connected = True
                return True
            except (ModbusInvalidResponseError, ModbusError) as e:
                rospy.logwarn(f"寄存器写入失败 (addr={addr}, val={value}, 尝试 {attempt + 1}/{max_attempts}): {str(e)}")
                self.connected = False
                # 如果不是最后一次尝试，等待片刻后重试
                if attempt < max_attempts - 1:
                    time.sleep(0.01)
                    continue
                return False


    def _get_status_text(self, clamp_state, rotate_state=-1):
        """根据状态码生成状态文本"""
        if not self.connected:
            return STATE_TEXT["SPECIAL"]["offline"]
        
        # 基础夹持状态
        base_status = STATE_TEXT["CLAMP"].get(clamp_state, "error")
        # print(f'{clamp_state}->{base_status}')
        # RGI旋转状态补充
        if self.type == "RGI" and rotate_state!=-1:
            rotate_status = STATE_TEXT["ROTATE"].get(rotate_state, "error")
            # print(f'合并状态 {base_status}({clamp_state}) {rotate_status}({rotate_state})')
            # 组合状态：夹持+堵转
            if base_status == "clamping" and rotate_status == "blocking":
                return STATE_TEXT["SPECIAL"]["clamping&blocking"]
            # 旋转状态优先（如堵转时覆盖基础状态）
            if rotate_status in ["blocking", "running"]:
                return rotate_status
        
        return base_status

    def reset(self):
        """初始化夹爪（重置）"""
        if not self.connected and not self._connect():
            return False
        
        # 读取当前初始化状态（地址：INIT_STATE）
        init_state = self._read_register(MODBUS_ADDR["INIT_STATE"])
        if init_state == -1:
            return False
        
        # 已初始化则返回成功
        # if init_state == 1:
        #     rospy.loginfo(f"{self.type} 夹爪已完成初始化")
        #     return True
        
        # 未初始化则执行标定（写入165到地址：INIT_CMD）
        rospy.loginfo(f"{self.type} 夹爪开始初始化...")
        # if self.type=="PGI":
        #     if not self._write_register(MODBUS_ADDR["INIT_DIR"], 0):
        #         return False
        #     if not self._write_register(MODBUS_ADDR["INIT_CMD"], 1):
        #         return False
        # else:
            # if not self._write_register(MODBUS_ADDR["INIT_CMD"], 165):
            #     return False
        if not self._write_register(MODBUS_ADDR["INIT_CMD"], 165):
            return False
        
        # 等待初始化完成（最多10秒）
        for _ in range(20):
            time.sleep(0.5)
            init_state = self._read_register(MODBUS_ADDR["INIT_STATE"])
            if init_state == 1:
                rospy.loginfo(f"{self.type} 夹爪初始化成功")
                return True
        
        rospy.logerr(f"{self.type} 夹爪初始化超时")
        return False

    def stop(self):
        """强制停止运动（仅RGI有效）"""
        if self.type == "RGI":
            # 写入旋转停止命令（地址：ROTATE_STOP）
            return self._write_register(MODBUS_ADDR["ROTATE_STOP"], 1)
        return True  # PGI无需旋转停止

    def get_feedback(self,idx=(1<<6)-1):
        """获取夹爪当前状态反馈（读取寄存器值）"""
        feedback = DEFAULT_RESPONSE.copy()
        if not self.connected:
            return feedback

        # 读取平行夹持参数
        # set_pos = self._read_register(MODBUS_ADDR["POSITION"]) if idx>>0&1 else -1  # 设定位置
        act_pos = self._read_register(MODBUS_ADDR["POSITION_ACTUAL"]) if idx>>0&1 else -1  # 实际位置
        self.force = self._read_register(MODBUS_ADDR["FORCE"]) if (idx>>1&1 and self.force==-1) else self.force  # 夹持力
        self.velocity = self._read_register(MODBUS_ADDR["VELOCITY"]) if (idx>>2&1 and self.velocity==-1) else self.velocity  # 平行速度

        # 读取RGI旋转参数
        abs_angle = 99999999
        torque = -1
        speed = -1
        rotate_state = -1
        if self.type == "RGI":
            # 角度（低位+高位）
            angle_low = self._read_register(MODBUS_ADDR["ROTATE_ANGLE_LOW"]) if idx>>3&1 else -1
            angle_high = self._read_register(MODBUS_ADDR["ROTATE_ANGLE_HIGH"]) if idx>>3&1 else -1
            print(f'angle high={angle_high} low={angle_low}')
            # if angle_high==65535 and angle_low==32268
            if angle_low!=-1 and angle_high!=-1:
                abs_angle = angle_high * 32768 + angle_low
                if abs_angle > 32768:  # 处理负数（补码转换）
                    abs_angle -= 65536
            
            self.torque = self._read_register(MODBUS_ADDR["TORQUE"]) if (idx>>4&1 and self.torque==-1) else self.torque  # 旋转力
            self.speed = self._read_register(MODBUS_ADDR["SPEED"]) if (idx>>5&1 and self.speed==-1) else self.speed  # 旋转速度
            rotate_state = self._read_register(MODBUS_ADDR["ROTATE_STATE"])# 旋转状态

        # 读取夹持状态
        clamp_state = self._read_register(MODBUS_ADDR["CLAMP_STATE"])

        # 填充反馈数据
        if self.type=='RGI':
            feedback["position"] = act_pos if act_pos else -1
            # feedback["set_position"] = set_pos if set_pos else -1
        else:
            feedback["position"] = 1000-act_pos if act_pos else -1
            # feedback["set_position"] = 1000-set_pos if set_pos else -1
        feedback["force"] = self.force
        feedback["velocity"] = self.velocity
        feedback["abs_angle"] = abs_angle
        feedback["torque"] = torque if (torque and self.type == "RGI") else -1
        feedback["speed"] = speed if (speed and self.type == "RGI") else -1
        feedback["status"] = self._get_status_text(clamp_state, rotate_state)
        print(f"获取夹爪状态：position={feedback['position']} abs_angle={abs_angle} status={feedback['status']} ")
        if self.set_pos!=-1 and self.type=="PGI" and feedback["status"]=="idle" and abs(self.set_pos-feedback["position"])>20 and self.set_pos<200:
            rospy.logerr(f"PGI夹爪夹持状态修正(idle->clamping)，{self.set_pos}!={feedback['position']}")
            self._write_register(MODBUS_ADDR["POSITION"], 1000-self.set_pos)
            rospy.logwarn(f'{self.type} 写入POSITION={1000-self.set_pos} 重新夹紧')
            # time.sleep(0.1)
            # return self.get_feedback(idx)
            feedback["status"]="clamping"
        return feedback

    def execute(self, req):
        """执行夹爪操作（核心逻辑）"""
        # 优先处理重置请求
        if req.reset_flag:
            if not self.reset():
                feedback = DEFAULT_RESPONSE.copy()
                feedback["status"] = "error"
                return self._pack_response(feedback)
        else:
            # 读取当前初始化状态（地址：INIT_STATE）
            init_state = self._read_register(MODBUS_ADDR["INIT_STATE"])
            if init_state == -1:
                feedback = DEFAULT_RESPONSE.copy()
                feedback["status"] = "error"
                return self._pack_response(feedback)
            
            if init_state == 0:
                rospy.logwarn(f"{self.type} 夹爪未初始化")

                # op=input('init(1) or ignore(2) or exit(3): ')
                op='1'
                if op=='1':
                    if not self.reset():
                        feedback = DEFAULT_RESPONSE.copy()
                        feedback["status"] = "error"
                        return self._pack_response(feedback)
                elif op=='2':
                    pass
                else:
                    exit(1)

                # for i in range(3):
                #     init_state = self._read_register(MODBUS_ADDR["INIT_STATE"])
                #     if init_state == 0:
                #         rospy.logwarn(f"{self.type} 夹爪确认初始化状态为未初始化")
                #     else:
                #         rospy.loginfo(f"{self.type} 夹爪确认初始化状态为已初始化，系误判")
                #         break
                    time.sleep(0.3)

            else:
                # rospy.loginfo(f"{self.type} 夹爪已初始化")
                pass


        # 处理停止请求
        if req.stop_flag:
            self.stop()
            time.sleep(0.1)  # 等待停止生效
            feedback = self.get_feedback(req.feedback)
            return self._pack_response(feedback)

        # 写入平行运动参数
        success = True

        # 夹持力（20-100%）
        if not (20 <= req.force <= 100):
            # rospy.logerr(f"力参数无效: {req.force} (范围20-100)")
            pass
        else:
            success &= self._write_register(MODBUS_ADDR["FORCE"], min(80,req.force))
            rospy.loginfo(f'{self.type} 写入FORCE={req.force}')

        # 运动速度（1-100%）
        if not (1 <= req.velocity <= 100):
            # rospy.logerr(f"速度参数无效: {req.velocity} (范围1-100)")
            pass
        else:
            success &= self._write_register(MODBUS_ADDR["VELOCITY"], req.velocity)
            rospy.loginfo(f'{self.type} 写入VELOCITY={req.velocity}')

        # 写入RGI旋转参数
        if self.type == "RGI":
            # 旋转力（20-100%）
            if not (20 <= req.torque <= 100):
                # rospy.logerr(f"旋转力参数无效: {req.torque} (范围20-100)")
                pass
            else:
                success &= self._write_register(MODBUS_ADDR["TORQUE"], req.torque)
                rospy.loginfo(f'{self.type} 写入TORQUE={req.torque}')

            # 旋转速度（1-100%）
            if not (1 <= req.speed <= 100):
                # rospy.logerr(f"旋转速度参数无效: {req.speed} (范围1-100)")
                pass
            else:
                success &= self._write_register(MODBUS_ADDR["SPEED"], req.speed)
                rospy.loginfo(f'{self.type} 写入SPEED={req.speed}')

        # 位置（PGI需要反向转换）
        position = req.position
        if 0<=position<=1000:
            self.set_pos=position
        position = 1000 - req.position if self.type == "PGI" else req.position
        if not (0 <= position <= 1000):
            # rospy.logerr(f"位置参数无效: {position} (范围0-1000)")
            pass
        else:
            success &= self._write_register(MODBUS_ADDR["POSITION"], position)
            rospy.loginfo(f'{self.type} 写入POSITION={req.position}')


        # 写入RGI旋转参数
        if self.type == "RGI":
            # 绝对角度（高位+低位）
            abs_angle_low = req.abs_angle % 65536
            if abs_angle_low>32767:
                abs_angle_low-=65536
            abs_angle_high = (req.abs_angle - abs_angle_low) // 32768
            if not (-160 <= abs_angle_high <= 160):
                # rospy.logerr(f"绝对角度参数无效: {req.abs_angle}")
                pass
            else:
                print(f'写入abs {req.abs_angle}->high={abs_angle_high} low={abs_angle_low}')
                success &= self._write_register(MODBUS_ADDR["ABS_ANGLE_LOW"], abs_angle_low)
                success &= self._write_register(MODBUS_ADDR["ABS_ANGLE_HIGH"], abs_angle_high)
                rospy.loginfo(f'{self.type} 写入ABS ANGLE={req.abs_angle}')

            # 相对角度（-32768~32767）
            if req.rel_angle==0:
                pass
            elif not (-32768 <= req.rel_angle <= 32767):
                # rospy.logerr(f"相对角度参数无效: {req.rel_angle}")
                pass
            else:
                success &= self._write_register(MODBUS_ADDR["REL_ANGLE"], req.rel_angle)
                rospy.loginfo(f'{self.type} 写入REL ANGLE={req.rel_angle}')

        # 阻塞等待完成（如果需要）
        if success and req.block_flag:
            time.sleep(0.2)
            rospy.loginfo(f"{self.type} 夹爪开始运动")
            start_time = time.time()
            while not rospy.is_shutdown():
                feedback = self.get_feedback(req.feedback)
                rospy.loginfo(f"{self.type} 夹爪运动中:{feedback['status']} set={self.set_pos}")
                if feedback["status"] in ["idle", "clamping", "blocking", "clamping&blocking", "error"]:
                    if self.type=='PGI' and self.set_pos!=-1 and abs(feedback['position']-self.set_pos)>10 and self.set_pos<200:
                        rospy.logwarn(f"{self.type} 判定{feedback['status']} {feedback['position']}!={self.set_pos}")
                        if feedback['status']=='idle':
                            rospy.logerr(f"{self.type} 判定夹持异常")
                        #     op=input('exit(1) or ignore(2): ')
                        #     if op=='2':
                        #         pass
                        #     else:
                        #         exit(1)
                    break
                time.sleep(0.1)
            else:
                rospy.logwarn(f"{self.type} 夹爪运动超时")
        else:
            # 获取最终状态反馈
            feedback = self.get_feedback(req.feedback)
        if not success:
            feedback["status"] = "error"
        if feedback["status"]=="error" or feedback["status"]=="offline":
            rospy.logerr(f"{self.type} 夹爪状态为{feedback['status']} position={feedback['position']} set={self.set_pos}!!!!!!!!!!!!")
        else:
            rospy.loginfo(f"{self.type} 夹爪状态为{feedback['status']} position={feedback['position']} set={self.set_pos}")
        rospy.loginfo("----------------------------------------")
        return self._pack_response(feedback)

    def _pack_response(self, feedback):
        """将反馈字典打包为srv响应格式"""
        resp = GripperResponse()
        resp.position = feedback["position"]
        resp.force = feedback["force"]
        resp.velocity = feedback["velocity"]
        resp.abs_angle = feedback["abs_angle"]
        resp.torque = feedback["torque"]
        resp.speed = feedback["speed"]
        resp.status = feedback["status"]
        return resp

    def do_req(self, req):
        """服务回调函数（带重试机制）"""
        max_retries = 2
        for attempt in range(max_retries):
            try:
                return self.execute(req)
            except Exception as e:
                rospy.logerr(f"操作失败（尝试 {attempt+1}/{max_retries}）: {str(e)}")
                if attempt == max_retries - 1:
                    # 最后一次尝试失败，返回错误状态
                    feedback = DEFAULT_RESPONSE.copy()
                    feedback["status"] = "error"
                    return self._pack_response(feedback)
                time.sleep(0.5)

if __name__ == "__main__":
    try:
        rospy.init_node("gripper_modbus_controller")
        
        # 从参数服务器获取配置（默认值可根据实际调整）
        # gripper_port = rospy.get_param("~Goal_gripper", "/tmp/LTOOL")
        gripper_port = rospy.get_param("~Used_port", "/dev/LPGI")
        gripper_type = rospy.get_param("~Goal_gripper", "PGI")
        gripper_prefix = rospy.get_param("~Prefix", "PGI")
        print(f'gripper params:port={gripper_port} type={gripper_type} service=/{gripper_prefix}_gripper_control')

        # 初始化夹爪控制器
        gripper_gripper = GripperCtrl(gripper_port, 1, gripper_type)
        # 创建服务
        rospy.Service(f"/{gripper_prefix}_gripper_control", Gripper, gripper_gripper.do_req)

        rospy.loginfo("夹爪控制服务已启动")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass