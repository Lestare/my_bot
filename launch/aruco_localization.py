#!/usr/bin/env python3
import rclpy
import numpy as np
import yaml
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from aruco_msgs.msg import MarkerArray as ArucoMarkerArray

class ArucoLocalization(Node):
    def __init__(self):
        super().__init__('aruco_localization')
        
        # Параметры
        self.declare_parameter('map_path', '')
        map_path = self.get_parameter('map_path').value
        
        # Загрузка карты меток
        self.marker_map = self.load_marker_map(map_path)
        
        # Публикаторы
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/map_markers', 10)
        
        # Подписчики
        self.create_subscription(
            ArucoMarkerArray,
            '/aruco_markers',
            self.markers_callback,
            10
        )
        
        # Трансформации
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Публикация визуализации карты
        self.publish_map_markers()
        
        self.get_logger().info("Aruco Localization Node Started")

    def load_marker_map(self, file_path):
        """Загрузка карты меток из YAML файла"""
        with open(file_path, 'r') as file:
            map_data = yaml.safe_load(file)
        return map_data['markers']

    def publish_map_markers(self):
        """Публикация меток карты для визуализации в RViz"""
        marker_array = MarkerArray()
        
        for marker_id, marker_data in self.marker_map.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "map_markers"
            marker.id = int(marker_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Позиция
            marker.pose.position.x = marker_data['position'][0]
            marker.pose.position.y = marker_data['position'][1]
            marker.pose.position.z = marker_data['position'][2]
            
            # Ориентация
            marker.pose.orientation.x = marker_data['orientation'][0]
            marker.pose.orientation.y = marker_data['orientation'][1]
            marker.pose.orientation.z = marker_data['orientation'][2]
            marker.pose.orientation.w = marker_data['orientation'][3]
            
            # Масштаб
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.01
            
            # Цвет
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.7
            
            marker.lifetime.sec = 0
            
            marker_array.markers.append(marker)
        
        self.markers_pub.publish(marker_array)

    def markers_callback(self, msg):
        """Обработка обнаруженных меток"""
        if not msg.markers:
            return
            
        # Для простоты используем первую найденную метку
        detected_marker = msg.markers[0]
        marker_id = str(detected_marker.id)
        
        if marker_id not in self.marker_map:
            self.get_logger().warn(f"Marker {marker_id} not in map")
            return
            
        # Получаем данные метки из карты
        map_marker = self.marker_map[marker_id]
        
        # Публикуем позицию (в реальной системе здесь будет расчет позиции)
        self.publish_robot_pose(map_marker)
        
        # Публикуем трансформацию
        self.publish_tf_transform(map_marker)

    def publish_robot_pose(self, map_marker):
        """Публикация позиции робота"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        
        # Позиция (упрощенный расчет)
        pose_msg.pose.pose.position.x = map_marker['position'][0]
        pose_msg.pose.pose.position.y = map_marker['position'][1]
        pose_msg.pose.pose.position.z = 0.0
        
        # Ориентация
        pose_msg.pose.pose.orientation.x = map_marker['orientation'][0]
        pose_msg.pose.pose.orientation.y = map_marker['orientation'][1]
        pose_msg.pose.pose.orientation.z = map_marker['orientation'][2]
        pose_msg.pose.pose.orientation.w = map_marker['orientation'][3]
        
        # Ковариация (примерные значения)
        pose_msg.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        
        self.pose_pub.publish(pose_msg)

    def publish_tf_transform(self, map_marker):
        """Публикация трансформации map->base_link"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        
        # Позиция
        transform.transform.translation.x = map_marker['position'][0]
        transform.transform.translation.y = map_marker['position'][1]
        transform.transform.translation.z = 0.0
        
        # Ориентация
        transform.transform.rotation.x = map_marker['orientation'][0]
        transform.transform.rotation.y = map_marker['orientation'][1]
        transform.transform.rotation.z = map_marker['orientation'][2]
        transform.transform.rotation.w = map_marker['orientation'][3]
        
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalization()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()