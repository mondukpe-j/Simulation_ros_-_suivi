import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        self.bridge = CvBridge()

        # Dictionnaire ArUco standard, 6x6, 250 marqueurs possibles
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.marker_length_m = 0.25   # taille réelle du marqueur, à ajuster selon celui que tu utiliseras en Gazebo

        self.camera_matrix = None
        self.dist_coeffs = None

        self.image_sub = self.create_subscription(
            Image,
            '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image',
            self.image_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/drone_perception/aruco_pose', 10)

    def camera_info_callback(self, msg: CameraInfo):
        # Reçu en continu, mais la matrice ne change pas ; on ne la stocke qu'une fois
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Calibration caméra reçue.")

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            self.get_logger().warn("Calibration caméra pas encore reçue, frame ignorée.")
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            self.get_logger().info(f"Marqueurs détectés: {ids.flatten().tolist()}")

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length_m, self.camera_matrix, self.dist_coeffs)

            # On publie la pose du premier marqueur détecté (à étendre si plusieurs cibles utiles)
            tvec = tvecs[0][0]
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'camera_link'   # repère caméra, pas encore transformé en repère monde
            pose_msg.pose.position.x = float(tvec[0])
            pose_msg.pose.position.y = float(tvec[1])
            pose_msg.pose.position.z = float(tvec[2])
            self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()