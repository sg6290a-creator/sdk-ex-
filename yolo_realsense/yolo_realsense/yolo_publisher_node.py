#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ament_index_python.packages import get_package_share_directory

class YoloMultiPublisher(Node):
    def __init__(self):
        super().__init__('yolo_multi_publisher')

        # Parameters – default model lives inside the installed share directory
        _default_model = os.path.join(
            get_package_share_directory('yolo_realsense'), 'models', 'best2.pt')
        self.declare_parameter('model_path', _default_model)
        self.declare_parameter('camera_frame', 'd405_optical_frame')
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('show_visualization', False)

        model_path = self.get_parameter('model_path').value
        self.show_visualization = self.get_parameter('show_visualization').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        # CV Bridge
        self.bridge = CvBridge()
        self.intrinsics_received = False

        # --- Publishers (각 객체별 전용 토픽) ---
        self.can_pub = self.create_publisher(PointStamped, 'can_target_point', 10)
        self.box_pub = self.create_publisher(PointStamped, 'box_target_point', 10)
        self.phone_pub = self.create_publisher(PointStamped, 'phone_target_point', 10)
        
        # 클래스 ID와 퍼블리셔 매핑 (0: can, 1: box, 2: phone)
        self.pub_map = {0: self.can_pub, 1: self.box_pub, 2: self.phone_pub}

        self.detection_image_pub = self.create_publisher(Image, '/yolo/detection_image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/yolo/target_markers', 10)

        # Subscribers
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self.camera_info_callback, qos)
        
        self.color_sub = Subscriber(self, Image, self.get_parameter('color_topic').value, qos_profile=qos)
        self.depth_sub = Subscriber(self, Image, self.get_parameter('depth_topic').value, qos_profile=qos)
        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=5, slop=0.1)
        self.sync.registerCallback(self.image_callback)

        # YOLO 모델 로드 (GPU 직접 지정)
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        import torch
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"PyTorch CUDA available: {torch.cuda.is_available()}")
        if torch.cuda.is_available():
            self.get_logger().info(f"GPU: {torch.cuda.get_device_name(0)}")
        self.get_logger().info(f"Using device: {self.device}")
        
        # 모델 로드 시 바로 device 지정
        self.model = YOLO(model_path)
        self.model.to(self.device)
        
        # Half precision (FP16) for faster inference on GPU
        self.use_half = self.device == 'cuda'
        if self.use_half:
            self.model.model.half()
            self.get_logger().info("Using FP16 (half precision) for faster GPU inference")
        
        self.get_logger().info("YOLO Multi-Topic Publisher (Direct Depth Mode) Ready")

    def camera_info_callback(self, msg: CameraInfo):
        if not self.intrinsics_received:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.intrinsics_received = True

    def deproject_pixel_to_point(self, u, v, depth):
        if not self.intrinsics_received: return None
        x = float((u - self.cx) * depth / self.fx)
        y = float((v - self.cy) * depth / self.fy)
        z = float(depth)
        return [x, y, z]

    def image_callback(self, color_msg: Image, depth_msg: Image):
        if not self.intrinsics_received: return

        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            depth_array = depth_image.astype(np.float32) / (1000.0 if depth_image.dtype == np.uint16 else 1.0)
        except Exception as e:
            self.get_logger().error(f"Image error: {e}")
            return

        # YOLO inference (GPU 사용, device 고정)
        results = self.model(color_image, verbose=False, device=self.device, half=self.use_half)
        annotated_frame = color_image.copy()
        marker_array = MarkerArray()
        obj_id_counter = 0

        for result in results:
            for box in result.boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                if conf < self.confidence_threshold: continue

                # 1. 객체별 중심점 계산
                x_c, y_c, _, _ = box.xywh[0].cpu().numpy()
                ix, iy = int(x_c), int(y_c)

                # 이미지 경계 체크
                if ix < 0 or ix >= depth_array.shape[1] or iy < 0 or iy >= depth_array.shape[0]:
                    continue

                # 2. 중심점 1개 픽셀의 Depth 값 직접 사용 (안정화 로직 제거)
                depth = depth_array[iy, ix]

                # 유효 범위 체크 (D405 등 카메라 특성에 맞춰 조정 가능)
                if 0.01 < depth < 2.0:
                    point_3d = self.deproject_pixel_to_point(ix, iy, depth)
                    if point_3d:
                        # 3. 객체별 전용 토픽 발행
                        msg = PointStamped()
                        msg.header = color_msg.header
                        msg.header.frame_id = self.camera_frame
                        msg.point.x, msg.point.y, msg.point.z = point_3d
                        
                        if cls_id in self.pub_map:
                            self.pub_map[cls_id].publish(msg)

                        # 4. RViz 마커 추가
                        marker = self.create_marker(point_3d, color_msg.header, obj_id_counter, cls_id)
                        marker_array.markers.append(marker)
                        obj_id_counter += 1

                # 시각화용 드로잉
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                color = [(0,0,255), (0,255,0), (255,0,0)][cls_id % 3] # Can:빨강, Box:초록, Phone:파랑
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                cv2.circle(annotated_frame, (ix, iy), 5, (0, 0, 255), -1) # 중심점 표시
                cv2.putText(annotated_frame, f"{self.model.names[cls_id]}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        self.marker_pub.publish(marker_array)
        
        # 화면 출력 (파라미터로 제어)
        if self.show_visualization:
            cv2.imshow('YOLO Multi-Detection (Direct Depth)', annotated_frame)
            cv2.waitKey(1)

        # 결과 이미지 토픽 발행
        self.detection_image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8"))

    def create_marker(self, point, header, obj_id, cls_id):
        marker = Marker()
        marker.header = header
        marker.header.frame_id = self.camera_frame
        marker.ns = "yolo_multi"
        marker.id = obj_id
        marker.type = Marker.SPHERE
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = point
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        colors = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        marker.color.r, marker.color.g, marker.color.b = colors[cls_id % 3]
        marker.color.a = 0.8
        marker.lifetime.sec = 1
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = YoloMultiPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()