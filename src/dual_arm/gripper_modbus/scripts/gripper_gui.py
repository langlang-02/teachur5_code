#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import rospy
import threading
import time
from gripper_modbus.srv import Gripper, GripperRequest

class ROSGripperControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS夹爪控制界面")
        self.root.geometry("950x700")
        self.root.minsize(900, 650)
        self.root.configure(bg="#f5f5f5")
        
        # 设置中文字体支持
        self.style = ttk.Style()
        self.style.configure("TLabel", font=("SimHei", 10), background="#f5f5f5")
        self.style.configure("TButton", font=("SimHei", 10))
        self.style.configure("TCheckbutton", font=("SimHei", 10), background="#f5f5f5")
        self.style.configure("TRadiobutton", font=("SimHei", 10), background="#f5f5f5")
        self.style.configure("TCombobox", font=("SimHei", 10))
        self.style.configure("Header.TLabel", font=("SimHei", 12, "bold"), background="#e0e0e0")
        self.style.configure("Emergency.TButton", font=("SimHei", 10), background="#ff4444", foreground="white")
        
        # 夹爪类型和服务状态
        self.gripper_type = tk.StringVar(value="PGI")
        self.service_available = {
            "PGI": False,
            "RGI": False
        }
        self.service_proxies = {
            "PGI": None,
            "RGI": None
        }
        self.service_topics = {
            "PGI": "/PGI_gripper_control",
            "RGI": "/RGI_gripper_control"
        }
        
        # 状态变量
        self.monitoring_active = False
        self.last_response = None
        self.ros_initialized = False
        
        # 创建UI组件
        self._create_widgets()
        
        # 先显示ROS初始化中状态
        self._log("正在初始化ROS节点...")
        
    def _create_widgets(self):
        """创建界面组件"""
        # 顶部标题
        header_frame = ttk.Frame(self.root, padding="10")
        header_frame.pack(fill=tk.X, padx=10, pady=5)
        ttk.Label(header_frame, text="ROS夹爪控制中心", style="Header.TLabel").pack(side=tk.LEFT)
        
        # 夹爪类型选择
        type_frame = ttk.Frame(self.root, padding="10")
        type_frame.pack(fill=tk.X, padx=10)
        ttk.Label(type_frame, text="夹爪类型:").pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(
            type_frame, text="PGI (平行夹爪)", 
            variable=self.gripper_type, value="PGI",
            command=self._on_gripper_type_change
        ).pack(side=tk.LEFT, padx=10)
        ttk.Radiobutton(
            type_frame, text="RGI (旋转夹爪)", 
            variable=self.gripper_type, value="RGI",
            command=self._on_gripper_type_change
        ).pack(side=tk.LEFT, padx=10)
        
        # 服务状态显示
        self.service_status_var = tk.StringVar(value="服务状态: 未连接")
        ttk.Label(type_frame, textvariable=self.service_status_var).pack(side=tk.RIGHT, padx=10)
        
        # 参数设置区域
        params_frame = ttk.LabelFrame(self.root, text="控制参数", padding="10")
        params_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 网格布局放置参数
        param_grid = ttk.Frame(params_frame)
        param_grid.pack(fill=tk.X, padx=5, pady=5)
        
        # 基础参数 (PGI和RGI共用)
        # 位置
        ttk.Label(param_grid, text="位置 (0-1000):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.position_var = tk.IntVar(value=500)
        ttk.Scale(param_grid, from_=0, to=1000, variable=self.position_var, command=self._update_position_label).grid(row=0, column=1, sticky=tk.EW, padx=5, pady=5)
        self.position_label = ttk.Label(param_grid, text="500")
        self.position_label.grid(row=0, column=2, padx=5, pady=5)
        
        # 夹持力
        ttk.Label(param_grid, text="夹持力 (20-100%):").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.force_var = tk.IntVar(value=50)
        ttk.Scale(param_grid, from_=20, to=100, variable=self.force_var, command=self._update_force_label).grid(row=1, column=1, sticky=tk.EW, padx=5, pady=5)
        self.force_label = ttk.Label(param_grid, text="50")
        self.force_label.grid(row=1, column=2, padx=5, pady=5)
        
        # 速度
        ttk.Label(param_grid, text="速度 (1-100%):").grid(row=2, column=0, sticky=tk.W, padx=5, pady=5)
        self.velocity_var = tk.IntVar(value=50)
        ttk.Scale(param_grid, from_=1, to=100, variable=self.velocity_var, command=self._update_velocity_label).grid(row=2, column=1, sticky=tk.EW, padx=5, pady=5)
        self.velocity_label = ttk.Label(param_grid, text="50")
        self.velocity_label.grid(row=2, column=2, padx=5, pady=5)
        
        # RGI专用参数
        self.RGI_params_frame = ttk.LabelFrame(params_frame, text="旋转参数 (仅RGI)", padding="10")
        self.RGI_params_frame.pack(fill=tk.X, padx=5, pady=5)
        
        RGI_grid = ttk.Frame(self.RGI_params_frame)
        RGI_grid.pack(fill=tk.X, padx=5, pady=5)
        
        # 绝对角度
        ttk.Label(RGI_grid, text="绝对角度:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.abs_angle_var = tk.IntVar(value=0)
        ttk.Entry(RGI_grid, textvariable=self.abs_angle_var, width=15).grid(row=0, column=1, padx=5, pady=5)
        
        # 相对角度
        ttk.Label(RGI_grid, text="相对角度:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.rel_angle_var = tk.IntVar(value=0)
        ttk.Entry(RGI_grid, textvariable=self.rel_angle_var, width=15).grid(row=1, column=1, padx=5, pady=5)
        
        # 旋转力
        ttk.Label(RGI_grid, text="旋转力 (20-100%):").grid(row=0, column=2, sticky=tk.W, padx=5, pady=5)
        self.torque_var = tk.IntVar(value=50)
        ttk.Scale(RGI_grid, from_=20, to=100, variable=self.torque_var, command=self._update_torque_label).grid(row=0, column=3, sticky=tk.EW, padx=5, pady=5)
        self.torque_label = ttk.Label(RGI_grid, text="50")
        self.torque_label.grid(row=0, column=4, padx=5, pady=5)
        
        # 旋转速度
        ttk.Label(RGI_grid, text="旋转速度 (1-100%):").grid(row=1, column=2, sticky=tk.W, padx=5, pady=5)
        self.speed_var = tk.IntVar(value=50)
        ttk.Scale(RGI_grid, from_=1, to=100, variable=self.speed_var, command=self._update_speed_label).grid(row=1, column=3, sticky=tk.EW, padx=5, pady=5)
        self.speed_label = ttk.Label(RGI_grid, text="50")
        self.speed_label.grid(row=1, column=4, padx=5, pady=5)
        
        # 配置网格权重，使控件可以拉伸
        param_grid.columnconfigure(1, weight=1)
        RGI_grid.columnconfigure(3, weight=1)
        
        # 控制选项
        options_frame = ttk.LabelFrame(self.root, text="控制选项", padding="10")
        options_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.stop_flag_var = tk.BooleanVar(value=False)
        self.block_flag_var = tk.BooleanVar(value=True)
        self.reset_flag_var = tk.BooleanVar(value=False)
        
        ttk.Checkbutton(
            options_frame, text="强制停止", variable=self.stop_flag_var
        ).pack(side=tk.LEFT, padx=15)
        ttk.Checkbutton(
            options_frame, text="阻塞模式", variable=self.block_flag_var
        ).pack(side=tk.LEFT, padx=15)
        ttk.Checkbutton(
            options_frame, text="重置夹爪", variable=self.reset_flag_var
        ).pack(side=tk.LEFT, padx=15)
        
        # 按钮区域
        buttons_frame = ttk.Frame(self.root, padding="10")
        buttons_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(
            buttons_frame, text="发送指令", command=self._send_command,
            width=15
        ).pack(side=tk.LEFT, padx=10)
        
        self.monitor_btn = ttk.Button(
            buttons_frame, text="开始监控", command=self._toggle_monitoring,
            width=15
        )
        self.monitor_btn.pack(side=tk.LEFT, padx=10)
        
        ttk.Button(
            buttons_frame, text="紧急停止", command=self._emergency_stop,
            width=15, style="Emergency.TButton"
        ).pack(side=tk.RIGHT, padx=10)
        
        # 状态显示区域
        status_frame = ttk.LabelFrame(self.root, text="夹爪状态", padding="10")
        status_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # 状态信息网格
        status_grid = ttk.Frame(status_frame)
        status_grid.pack(fill=tk.X, padx=5, pady=5)
        
        status_labels = [
            ("当前位置:", "position_var"),
            ("当前夹持力:", "force_var"),
            ("当前速度:", "velocity_var"),
            ("当前角度:", "angle_var"),
            ("当前旋转力:", "torque_status_var"),
            ("当前旋转速度:", "speed_status_var"),
            ("状态:", "status_var")
        ]
        self.status_widgets = {}  # 保存标签控件引用
        self.status_vars = {}
        for i, (label_text, var_name) in enumerate(status_labels):
            ttk.Label(status_grid, text=label_text).grid(row=i//2, column=(i%2)*2, sticky=tk.W, padx=10, pady=5)
            var = tk.StringVar(value="-")
            self.status_vars[var_name] = var
            # 保存标签控件
            label_widget = ttk.Label(status_grid, textvariable=var, foreground="blue")
            label_widget.grid(row=i//2, column=(i%2)*2 + 1, sticky=tk.W, padx=10, pady=5)
            self.status_widgets[var_name] = label_widget
            ttk.Label(status_grid, textvariable=var, foreground="blue").grid(row=i//2, column=(i%2)*2 + 1, sticky=tk.W, padx=10, pady=5)
        
        # 日志区域
        log_frame = ttk.LabelFrame(self.root, text="操作日志", padding="10")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, font=("SimHei", 10))
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.config(state=tk.DISABLED)
        
        # 初始隐藏RGI参数区域（根据默认类型）
        self._on_gripper_type_change()

    # 更新滑块标签显示
    def _update_position_label(self, value):
        self.position_label.config(text=str(int(float(value))))
    
    def _update_force_label(self, value):
        self.force_label.config(text=str(int(float(value))))
    
    def _update_velocity_label(self, value):
        self.velocity_label.config(text=str(int(float(value))))
    
    def _update_torque_label(self, value):
        self.torque_label.config(text=str(int(float(value))))
    
    def _update_speed_label(self, value):
        self.speed_label.config(text=str(int(float(value))))
    
    def _on_gripper_type_change(self):
        """切换夹爪类型时的处理"""
        current_type = self.gripper_type.get()
        if current_type == "RGI":
            self.RGI_params_frame.pack(fill=tk.X, padx=5, pady=5)
        else:
            self.RGI_params_frame.pack_forget()
        self._log(f"切换到{current_type}夹爪控制模式")
    
    def init_ros(self):
        """在主线程中初始化ROS节点"""
        print('初始化ROS')
        try:
            if not rospy.core.is_initialized():
                rospy.init_node('gripper_control_gui', anonymous=True)
                self.ros_initialized = True
                self._log("ROS节点初始化成功")
                print('启动服务检查')
                self._check_service_availability()
            else:
                self.ros_initialized = True
                self._log("ROS节点已初始化")
        except rospy.ROSException as e:
            self._log(f"ROS节点初始化失败: {str(e)}", is_error=True)
            # 重试初始化
            self.root.after(2000, self.init_ros)
    def _check_service_availability(self):
        """检查ROS服务是否可用（兼容版本）"""
        if not self.ros_initialized:
            # ROS未初始化，稍后再试
            self.root.after(1000, self._check_service_availability)
            return
            
        current_type = self.gripper_type.get()
        topic = self.service_topics[current_type]
        print(f'ROS已初始化 准备检查service:{topic} type:{current_type}')
        
        try:
            # 检查服务是否存在（兼容版本）
            # 使用wait_for_service并设置短超时来判断服务是否存在
            service_exists = False
            try:
                # 等待服务0.5秒，超时则认为服务不存在
                rospy.wait_for_service(topic, timeout=0.5)
                service_exists = True
            except rospy.ROSException:
                service_exists = False
                
            if service_exists:
                if not self.service_proxies[current_type]:
                    self.service_proxies[current_type] = rospy.ServiceProxy(topic, Gripper)
                    self.service_available[current_type] = True
                    self._log(f"已连接到{current_type}夹爪服务: {topic}")
            else:
                self.service_available[current_type] = False
                self.service_proxies[current_type] = None
            
            # 更新服务状态显示
            if self.service_available[current_type]:
                self.service_status_var.set(f"服务状态: 已连接到 {topic}")
            else:
                self.service_status_var.set(f"服务状态: 等待 {topic} ...")
                
        except Exception as e:
            self._log(f"服务检查出错: {str(e)}", is_error=True)
            self.service_available[current_type] = False
        
        # 定期检查
        self.root.after(2000, self._check_service_availability)
    
    def _send_command(self):
        """发送控制指令到ROS服务"""
        if not self.ros_initialized:
            messagebox.showerror("错误", "ROS节点尚未初始化，请等待...")
            return
            
        current_type = self.gripper_type.get()
        
        # 检查服务是否可用
        if not self.service_available[current_type] or not self.service_proxies[current_type]:
            messagebox.showerror("错误", f"{current_type}夹爪服务不可用，请检查连接")
            return
        
        # 创建请求
        req = GripperRequest()
        req.position = self.position_var.get()
        req.force = self.force_var.get()
        req.velocity = self.velocity_var.get()
        
        # RGI专用参数
        if current_type == "RGI":
            req.abs_angle = self.abs_angle_var.get()
            req.rel_angle = self.rel_angle_var.get()
            req.torque = self.torque_var.get()
            req.speed = self.speed_var.get()
        else:
            req.abs_angle = 0
            req.rel_angle = 0
            req.torque = 0
            req.speed = 0
        
        # 控制标志
        req.stop_flag = self.stop_flag_var.get()
        req.block_flag = self.block_flag_var.get()
        req.reset_flag = self.reset_flag_var.get()
        
        # 发送请求（在单独线程中执行，避免界面卡顿）
        threading.Thread(target=self._call_service, args=(current_type, req), daemon=True).start()
    
    def _call_service(self, gripper_type, req):
        """调用ROS服务的实际函数"""
        self._log(f"发送{gripper_type}控制指令...")
        try:
            # 调用服务
            response = self.service_proxies[gripper_type](req)
            self.last_response = response
            
            # 记录响应
            self._log(f"指令执行完成，状态: {response.status}")
            self._update_status_display(response)
            
            # 如果是停止或重置指令，自动取消对应复选框
            if req.stop_flag:
                self.root.after(100, lambda: self.stop_flag_var.set(False))
            if req.reset_flag:
                self.root.after(100, lambda: self.reset_flag_var.set(False))
                
        except rospy.ServiceException as e:
            self._log(f"服务调用失败: {str(e)}", is_error=True)
        except Exception as e:
            self._log(f"指令发送出错: {str(e)}", is_error=True)
    
    def _emergency_stop(self):
        """紧急停止功能"""
        if not self.ros_initialized:
            messagebox.showerror("错误", "ROS节点尚未初始化，请等待...")
            return
            
        current_type = self.gripper_type.get()
        if messagebox.askyesno("确认", f"确定要紧急停止{current_type}夹爪吗?"):
            # 创建仅包含停止标志的请求
            req = GripperRequest()
            req.stop_flag = True
            req.block_flag = False
            req.reset_flag = False
            
            # 其他参数设为默认值
            req.position = 0
            req.force = 0
            req.velocity = 0
            req.abs_angle = 0
            req.rel_angle = 0
            req.torque = 0
            req.speed = 0
            
            # 发送请求
            threading.Thread(target=self._call_service, args=(current_type, req), daemon=True).start()
    
    def _toggle_monitoring(self):
        """切换状态监控功能"""
        if not self.ros_initialized:
            messagebox.showerror("错误", "ROS节点尚未初始化，请等待...")
            return
            
        self.monitoring_active = not self.monitoring_active
        if self.monitoring_active:
            self.monitor_btn.config(text="停止监控")
            self._log("开始状态监控...")
            self._start_monitoring()
        else:
            self.monitor_btn.config(text="开始监控")
            self._log("停止状态监控")
    
    def _start_monitoring(self):
        """定期获取夹爪状态"""
        if not self.monitoring_active:
            return
            
        current_type = self.gripper_type.get()
        if self.service_available[current_type] and self.service_proxies[current_type]:
            # 创建一个空请求，仅用于获取当前状态
            req = GripperRequest()
            req.position = self.position_var.get()  # 使用当前设置的位置
            req.force = self.force_var.get()
            req.velocity = self.velocity_var.get()
            req.stop_flag = False
            req.block_flag = False
            req.reset_flag = False
            
            if current_type == "RGI":
                req.abs_angle = self.abs_angle_var.get()
                req.rel_angle = 0  # 不发送相对角度
                req.torque = self.torque_var.get()
                req.speed = self.speed_var.get()
            else:
                req.abs_angle = 0
                req.rel_angle = 0
                req.torque = 0
                req.speed = 0
            
            # 调用服务获取状态
            try:
                response = self.service_proxies[current_type](req)
                self.last_response = response
                self._update_status_display(response)
            except Exception as e:
                self._log(f"监控状态获取失败: {str(e)}", is_error=True)
        
        # 继续监控（500ms后）
        if self.monitoring_active:
            self.root.after(50, self._start_monitoring)
    
    def _update_status_display(self, response):
        """更新状态显示区域"""
        self.status_vars["position_var"].set(str(response.position))
        self.status_vars["force_var"].set(str(response.force))
        self.status_vars["velocity_var"].set(str(response.velocity))
        self.status_vars["angle_var"].set(str(response.abs_angle))
        self.status_vars["torque_status_var"].set(str(response.torque))
        self.status_vars["speed_status_var"].set(str(response.speed))
        
        # 根据状态设置不同颜色
        status = response.status
        color = "green" if status in ["idle", "clamping"] else \
                "orange" if status in ["running", "blocking", "clamping&blocking"] else \
                "red"
        
        # 直接访问保存的标签控件修改颜色
        self.status_widgets["status_var"].config(foreground=color)
        # self.status_vars["status_var"].set(status)
        # for widget in self.status_vars["status_var"].widget.master.winfo_children():
        #     if widget == self.status_vars["status_var"].widget:
        #         widget.config(foreground=color)
        #         break
    
    def _log(self, message, is_error=False):
        if is_error:
            rospy.logerr(message)
        else:
            rospy.loginfo(message)
        """添加日志信息"""
        self.log_text.config(state=tk.NORMAL)
        timestamp = time.strftime("[%H:%M:%S] ")
        if is_error:
            self.log_text.insert(tk.END, timestamp + "错误: " + message + "\n", "error")
        else:
            self.log_text.insert(tk.END, timestamp + message + "\n")
        self.log_text.tag_config("error", foreground="red")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

if __name__ == "__main__":
    root = tk.Tk()
    app = ROSGripperControlGUI(root)
    
    # 在主线程中初始化ROS，解决信号处理问题
    root.after(100, app.init_ros)
    
    root.mainloop()
    