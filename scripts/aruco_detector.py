#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        
        # Параметры
        self.declare_parameter('marker_size', 0.15)  # Размер метки в метрах
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        
        # Подписчики
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Издатели
        self.markers_pub = self.create_publisher(MarkerArray, '/aruco_markers', 10)
        
        # Переменные
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = self.get_parameter('marker_size').value
        
        # Словарь ArUco
        dictionary_name = self.get_parameter('dictionary').value
        self.aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, dictionary_name))
        self.parameters = aruco.DetectorParameters_create()
        
        self.get_logger().info("Aruco Detector Node Started")

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Received camera parameters")

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn("Camera parameters not received yet")
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Обнаружение маркеров
            corners, ids, rejected = aruco.detectMarkers(
                gray, 
                self.aruco_dict,
                parameters=self.parameters
            )
            
            if ids is not None:
                # Оценка позы маркеров
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, 
                    self.marker_size, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                # Создание сообщения MarkerArray
                marker_array = MarkerArray()
                
                for i in range(len(ids)):
                    marker = Marker()
                    marker.header = msg.header
                    marker.ns = "aruco_markers"
                    marker.id = int(ids[i][0])
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    
                    # Позиция
                    marker.pose.position = Point(
                        x=float(tvecs[i][0][0]),
                        y=float(tvecs[i][0][1]),
                        z=float(tvecs[i][0][2])
                    )
                    
                    # Ориентация
                    rotation_matrix = np.eye(4)
                    rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                    quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
                    marker.pose.orientation = Quaternion(
                        x=quaternion[0],
                        y=quaternion[1],
                        z=quaternion[2],
                        w=quaternion[3]
                    )
                    
                    # Масштаб
                    marker.scale.x = self.marker_size
                    marker.scale.y = self.marker_size
                    marker.scale.z = 0.01
                    
                    # Цвет
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.5  # Полупрозрачный
                    
                    marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
                    
                    marker_array.markers.append(marker)
                
                self.markers_pub.publish(marker_array)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def rotation_matrix_to_quaternion(self, R):
        # Конвертация матрицы вращения в кватернион
        q = np.empty((4,))
        trace = np.trace(R)
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            q[3] = 0.25 * s
            q[0] = (R[2, 1] - R[1, 2]) / s
            q[1] = (R[0, 2] - R[2, 0]) / s
            q[2] = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s
            
        return q

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()