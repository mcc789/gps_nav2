#!/usr/bin/env python3
"""
GPS导航目标发布节点: 
    将指定的gps位置转换成 Nav2 导航框架能识别的map坐标系下的坐标,
    发送给 Nav2 的navigate_to_pose动作服务器, 进行运动
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


# ================= 全局配置区域 =================
# 目标GPS经纬度
TARGET_LAT = 37.412066  # 纬度（北为正）
TARGET_LON = -122.063752 # 经度（东为正，西为负）

# 地图原点GPS（Gazebo world文件中定义的GPS原点，需匹配）
ORIGIN_LAT = 37.41203   # 原点纬度
ORIGIN_LON = -122.06382  # 原点经度
# ================================================================


class GPSGoalPublisher(Node):
    def __init__(self):
        super().__init__('gps_goal_publisher')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.target_lat = TARGET_LAT
        self.target_lon = TARGET_LON
        self.origin_lat = ORIGIN_LAT
        self.origin_lon = ORIGIN_LON

    
    def gps_to_cartesian(self, lat, lon):
        """
        经纬度转笛卡尔坐标(超大范围需要UTM转换)
        """
        R = 6371000.0  # 地球半径

        # 角度转弧度
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)
        
        # 计算差值
        dlat = lat_rad - origin_lat_rad
        dlon = lon_rad - origin_lon_rad
        
        # 转换成局部坐标
        x = R * dlon * math.cos(origin_lat_rad)
        y = R * dlat
        
        return x, y


    def send_goal(self):
        """
        发送导航目标
        """
        # 等待Nav2服务器就绪
        self.get_logger().info('等待 Nav2 服务器连接...')
        self._action_client.wait_for_server()
        
        # 经纬转局部坐标
        x, y = self.gps_to_cartesian(self.target_lat, self.target_lon)
        self.get_logger().info(f'目标局部坐标: x={x:.2f}, y={y:.2f}')
        
        # 构建导航目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 设置目标位置
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # 设置目标朝向，默认x轴正向
        goal_msg.pose.pose.orientation.w = 1.0 

        # 发送目标
        self.get_logger().info('正在发送导航目标...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        """
        目标响应回调
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝！')
            return

        self.get_logger().info('正在前往...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        """
        目标响应回调
        """
        result = future.result().result
        self.get_logger().info('导航结束！')
        rclpy.shutdown()



def main():
    rclpy.init()
    node = GPSGoalPublisher()
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()