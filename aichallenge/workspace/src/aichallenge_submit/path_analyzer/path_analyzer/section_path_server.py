import sys

from path_service.srv import GetPath, GetObstaclePath
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from .submodules.rrt_star_reeds_shepp import RRTStarReedsShepp

import matplotlib.pyplot as plt


class SectionPathServer(Node):

    def __init__(self):
        super().__init__('section_path_server')
        self.declare_parameter('section_id')
        self.section_id_ = str(self.get_parameter('section_id').value)
        self.get_path_client = self.create_client(GetPath, '/get_path')
        self.get_path_request = GetPath.Request()

        self.obstacle_path_srv = self.create_service(GetObstaclePath, '/obstacle_path_' + self.section_id_ , self.get_obstacle_path_callback)
        self.get_logger().info('Section Path Server has been started.')

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/costmap_2d',
            self.costmap_callback,
            10)

        self.get_path_future = None
        self.costmap_data = None
        self.obj_margin_index = 5
        self.cut_margin = 30

    def send_path_req(self):
        self.get_path_request.csv_path = self.section_id_
        self.get_path_future = self.get_path_client.call_async(self.get_path_request)
    

    def get_obstacle_path_callback(self, request, response):
        obstacle_pose = [request.x, request.y]
        self.send_path_req()
        section_path=None
        if self.get_path_future.done():
            try:
                section_path = self.get_path_future.result().path
                self.get_logger().info('Received path')
            except Exception as e:
                self.get_logger().info('Service call failed %r' % (e,))
        if section_path is not None:
            optimal_path = self.generate_obstacle_path(section_path, obstacle_pose)
            response.optimal_path = optimal_path
            return response
    
    def costmap_callback(self, msg):

        self.costmap_data = msg



    def generate_obstacle_path(self, section_path, obstacle_pose):
        optimal_path = Path()
        optimal_path.header = section_path.header
        optimal_path.path.poses = []

        path_index_nearest_obstacle = self.search_nearest_index(section_path, obstacle_pose)

        change_path_start_index = path_index_nearest_obstacle - self.obj_margin_index
        change_path_end_index = path_index_nearest_obstacle + self.obj_margin_index
        if change_path_start_index < 0:
            change_path_start_index = 0
        if change_path_end_index >= len(section_path.poses):
            change_path_end_index = len(section_path.poses) - 1
        # 変更しない部分(前半を追加)
        optimal_path.poses = section_path.poses[:change_path_start_index]
        lane_pose = self.cut_costmap(self.costmap_data, section_path.poses[path_index_nearest_obstacle].pose.position.x, section_path.poses[path_index_nearest_obstacle].pose.position.y, self.cut_margin)
        change_start_pose = section_path.poses[change_path_start_index].pose
        change_end_pose = section_path.poses[change_path_end_index].pose
        obstacle_list = []
        # lane_poseのx,y座標の最小値を取得
        min_x = min(lane_pose[0])
        min_y = min(lane_pose[1])
        # lane_poseのx,y座標の最大値を取得
        max_x = max(lane_pose[0])
        max_y = max(lane_pose[1])
        # 差分を取得
        diff_x = max_x - min_x
        diff_y = max_y - min_y
        # 大きい方の差分を取得
        diff = max(diff_x, diff_y)

        for i in range(len(lane_pose[0])):
            obstacle_list.append((lane_pose[0][i]-min_x, lane_pose[1][i]-min_y, 0.5))
        # add obstacle
        obstacle_list.append((obstacle_pose[0]-min_x, obstacle_pose[1]-min_y, 1.5))
        rrt_star_reeds_shepp_path = RRTStarReedsShepp([change_start_pose.position.x-min_x, change_start_pose.position.y-min_y, self.euler_from_quaternion(change_start_pose.orientation)[2]], [change_end_pose.position.x-min_x, change_end_pose.position.y-min_y, self.euler_from_quaternion(change_end_pose.orientation)[2]], obstacle_list, [0.0, diff], max_iter=100)
        path = rrt_star_reeds_shepp_path.planning(animation=False)
        if path:
            for i in range(len(path)):
                pose = PoseStamped()
                pose.pose.position.x = path[i][0]+min_x
                pose.pose.position.y = path[i][1]+min_y
                pose.pose.orientation = self.quaternion_from_euler(0, 0, path[i][2])
                optimal_path.poses.append(pose)
        # 変更しない部分(後半を追加)
        optimal_path.poses += section_path.poses[change_path_end_index+1:]
        return optimal_path


    def cut_costmap(self, costmap_data, center_pose_x, center_pose_y, margin):
        """
        center_poseを中心にmargin分だけ切り取り、要素が127の部分の座標を返す
        """
        cost_data = costmap_data.data
        width = costmap_data.info.width
        height = costmap_data.info.height
        resolution = costmap_data.info.resolution
        origin = costmap_data.info.origin
        # reshape costmap data
        cost_data = np.array(cost_data).reshape((height, width))
        # center_poseをindexに変換
        center_pose_index_x = int((center_pose_x - origin.position.x) / resolution)
        center_pose_index_y = int((center_pose_y - origin.position.y) / resolution)
        # margin分だけ切り取り
        cut_cost_data = cost_data[center_pose_index_y - margin:center_pose_index_y + margin, center_pose_index_x - margin:center_pose_index_x + margin]
        # 127の部分の座標を取得
        obs_index = np.where(cut_cost_data == 127)
        # 座標変換
        obs_index[0] = obs_index[0] + center_pose_index_y - margin
        obs_index[1] = obs_index[1] + center_pose_index_x - margin
        # xy座標に変換
        obs_index[0] = obs_index[0] * resolution + origin.position.y
        obs_index[1] = obs_index[1] * resolution + origin.position.x
        obs_pose = [obs_index[1], obs_index[0]]
        obs_pose = np.unique(obs_pose, axis=1)
        return obs_pose

    
    def search_nearest_index(self, path, obstacle_pose):
        min_distance = np.inf
        nearest_index = 0
        for i in range(len(path.poses)):
            distance = math.sqrt((path.poses[i].pose.position.x - obstacle_pose[0])**2 + (path.poses[i].pose.position.y - obstacle_pose[1])**2)
            if distance < min_distance:
                min_distance = distance
                nearest_index = i
        return nearest_index

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
    

def main(args=None):
    rclpy.init(args=args)

    section_path_server = SectionPathServer()

    rclpy.spin(section_path_server)

    section_path_server.destroy_node()
    rclpy.shutdown()





if __name__ == '__main__':
    main()