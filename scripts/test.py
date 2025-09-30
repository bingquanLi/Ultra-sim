#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vehicle_msgs.msg import VehicleCmd, VehicleStatus
import numpy as np
import matplotlib.pyplot as plt
import os
from threading import Thread
from datetime import datetime
import time

class VehiclePathController(Node):
    def __init__(self):
        super().__init__('vehicle_path_controller')
        # 发布控制指令和订阅状态信息
        self.cmd_publisher = self.create_publisher(VehicleCmd, 'vehicle_cmd', 10)
        self.status_subscriber = self.create_subscription(
            VehicleStatus,
            'vehicle_status',
            self.status_callback,
            10)
        
        # 车辆控制参数
        self.target_speed = 2.0  # 目标速度 (m/s)
        self.target_steering = 0.3  # 目标转向角 (rad)
        self.duration = 50.0  # 运行时间 (秒)
        
        # 状态标志
        self.completed = False    # 路径完成标志
        self.start_time = time.time()  # 记录开始时间
        
        # 数据记录列表
        self.time_data = []
        self.x_data = []
        self.y_data = []
        self.yaw_data = []
        self.speed_data = []
        self.steering_data = []
        
        # 控制定时器（10Hz控制频率）
        self.control_timer = self.create_timer(0.1, self.control_callback)
        
        self.get_logger().info("车辆路径控制器已启动, 将运行50秒后停止并绘制图表")

    def control_callback(self):
        # 计算已运行时间
        elapsed_time = time.time() - self.start_time
        
        # 检查是否完成运行
        if elapsed_time >= self.duration and not self.completed:
            self.stop_vehicle()
            self.completed = True
            self.get_logger().info("车辆已运行完成，正在绘制图表...")
            self.generate_static_plots()

            return
        elif self.completed:
            return
        
        # 生成控制指令
        cmd = VehicleCmd()
        cmd.speed = float(self.target_speed)
        cmd.steering_angle = float(self.target_steering)
        self.cmd_publisher.publish(cmd)
        
        # 记录控制指令
        self.steering_data.append(self.target_steering)

    def status_callback(self, msg):
        # 记录车辆状态数据
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.x_data.append(msg.position.x)
        self.y_data.append(msg.position.y)
        self.yaw_data.append(msg.yaw)
        self.speed_data.append(msg.speed)


    def stop_vehicle(self):
        # 发送停止指令
        stop_cmd = VehicleCmd()
        stop_cmd.speed = 0.0
        stop_cmd.steering_angle = 0.0
        self.cmd_publisher.publish(stop_cmd)
        self.get_logger().info("车辆已停止")
        
    def generate_static_plots(self):
        # 确保有足够的数据
        if len(self.time_data) < 2:
            self.get_logger().error("数据不足，无法绘制图表")
            return
        
        # 创建4个子图
        fig, axs = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Vehicle Status During 10-Second Run', fontsize=16)
       
        
        # 绘制轨迹图
        axs[0,0].plot(self.x_data, self.y_data, 'b-', label='Trajectory')
        axs[0,0].scatter(self.x_data[0], self.y_data[0], c='green', s=100, label='Start')
        axs[0,0].scatter(self.x_data[-1], self.y_data[-1], c='red', s=100, label='End')
        axs[0,0].set_xlabel('X Position (m)')
        axs[0,0].set_ylabel('Y Position (m)')
        axs[0,0].set_title('Vehicle Trajectory')
        axs[0,0].legend()
        axs[0,0].grid(True)
        axs[0,0].set_aspect('equal')
        
        # 绘制偏航角曲线
        axs[0,1].plot(self.time_data, self.yaw_data, 'r-', label='Yaw Angle')
        axs[0,1].set_xlabel('Time (s)')
        axs[0,1].set_ylabel('Yaw Angle (rad)')
        axs[0,1].set_title('Vehicle Yaw')
        axs[0,1].legend()
        axs[0,1].grid(True)
        
        # 绘制速度曲线
        axs[1,0].plot(self.time_data, self.speed_data, 'g-', label='Speed')
        axs[1,0].set_xlabel('Time (s)')
        axs[1,0].set_ylabel('Speed (m/s)')
        axs[1,0].set_title('Vehicle Speed')
        axs[1,0].legend()
        axs[1,0].grid(True)
        
        # 绘制转向角曲线
        axs[1,1].plot(self.time_data[:len(self.steering_data)], self.steering_data, 'm-', label='Steering Angle')
        axs[1,1].set_xlabel('Time (s)')
        axs[1,1].set_ylabel('Steering Angle (rad)')
        axs[1,1].set_title('Steering Command')
        axs[1,1].legend()
        axs[1,1].grid(True)
        
        plt.tight_layout(rect=[0, 0, 1, 0.96])  # 预留标题空间
        
        plot_filename = f"vehicle_run.png"
        plt.savefig(plot_filename, dpi=300)
        self.get_logger().info(f"图表已保存为: {plot_filename}")
        
        # 显示图表
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    controller = VehiclePathController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.stop_vehicle()
        
      

if __name__ == '__main__':
    main()