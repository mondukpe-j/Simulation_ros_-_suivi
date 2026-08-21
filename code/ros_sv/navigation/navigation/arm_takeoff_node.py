import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped


class ArmTakeoffNode(Node):
    def __init__(self):
        super().__init__('arm_takeoff_node')

        state_qos = QoSProfile(depth=10)
        state_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        state_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        pose_qos.durability = QoSDurabilityPolicy.VOLATILE

        self.current_state = State()
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, state_qos)

        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, pose_qos)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_altitude = 0.0
        self._mode_request_in_flight = False
        self._arm_request_in_flight = False

        self.timer = self.create_timer(1.0, self.sequence_callback)
        self.step = 'WAIT_CONNECTION'

    def state_callback(self, msg: State):
        self.current_state = msg

    def pose_callback(self, msg: PoseStamped):
        self.current_altitude = msg.pose.position.z

    def sequence_callback(self):
        self.get_logger().info(
            f"État: {self.step} | Altitude: {self.current_altitude:.2f}m | "
            f"Armé: {self.current_state.armed} | Mode: {self.current_state.mode}"
        )

        if self.step == 'WAIT_CONNECTION':
            if self.current_state.connected:
                self.get_logger().info("Connexion établie avec le drone.")
                self.step = 'SET_MODE'

        elif self.step == 'SET_MODE':
            if self.current_state.mode == 'AUTO.TAKEOFF':
                self.get_logger().info("Mode AUTO.TAKEOFF déjà actif.")
                self.step = 'ARMING'
            elif not self._mode_request_in_flight:
                self._mode_request_in_flight = True
                req = SetMode.Request()
                req.custom_mode = 'AUTO.TAKEOFF'
                future = self.set_mode_client.call_async(req)
                future.add_done_callback(self.on_set_mode_response)
            # sinon : requête déjà envoyée, on attend que current_state.mode confirme

        elif self.step == 'ARMING':
            if self.current_state.armed:
                self.get_logger().info("Drone armé (confirmé).")
                self.step = 'MONITORING'
            elif not self._arm_request_in_flight:
                self._arm_request_in_flight = True
                req = CommandBool.Request()
                req.value = True
                future = self.arming_client.call_async(req)
                future.add_done_callback(self.on_arm_response)
            # sinon : requête déjà envoyée, on attend que current_state.armed confirme

        elif self.step == 'MONITORING':
            if self.current_altitude >= 2.0:
                self.get_logger().info("Altitude de 2 mètres atteinte.")
                self.step = 'DONE'

        elif self.step == 'DONE':
            self.get_logger().info("Séquence terminée avec succès.")
            self.timer.cancel()
            rclpy.shutdown()

    def on_set_mode_response(self, future):
        self._mode_request_in_flight = False
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Exception lors du changement de mode: {e}")
            return

        if result.mode_sent:
            self.get_logger().info("Requête de mode envoyée, en attente de confirmation réelle...")
        else:
            self.get_logger().error("Échec de l'envoi du changement de mode, nouvelle tentative...")

    def on_arm_response(self, future):
        self._arm_request_in_flight = False
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Exception lors de l'armement: {e}")
            return

        if not result.success:
            self.get_logger().error("Échec de l'armement, nouvelle tentative...")


def main(args=None):
    rclpy.init(args=args)
    node = ArmTakeoffNode()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()