import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from mavros_msgs.msg import State, Waypoint, WaypointList, WaypointReached
from mavros_msgs.srv import WaypointPush, SetMode, WaypointSetCurrent


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        state_qos = QoSProfile(depth=10)
        state_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        state_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.current_state = State()
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, state_qos)

        self.mission_reached_sub = self.create_subscription(
            WaypointReached, '/mavros/mission/reached', self.reached_callback, 10)

        self.push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.set_current_client = self.create_client(WaypointSetCurrent, '/mavros/mission/set_current')

        self.waypoints = [
            (47.3977000, 8.5459000, 10.0),
            (47.3977900, 8.5459000, 10.0),
            (47.3978800, 8.5459000, 10.0),
            (47.3979700, 8.5459000, 10.0),

            # Ligne 2 (Nord -> Sud)
            (47.4000000, 8.5450000, 12.0),
            (47.3990000, 8.5450000, 12.0),
            (47.3980000, 8.5450000, 12.0),
            (47.3970000, 8.5450000, 12.0),

            # Ligne 3 (Sud -> Nord)
            (47.3970000, 8.5460000, 15.0),
            (47.3980000, 8.5460000, 15.0),
            (47.3990000, 8.5460000, 15.0),
            (47.4000000, 8.5460000, 15.0),

            # Ligne 4 (Nord -> Sud)
            (47.4000000, 8.5470000, 10.0),
            (47.3990000, 8.5470000, 10.0),
            (47.3980000, 8.5470000, 10.0),
            (47.3970000, 8.5470000, 10.0)
        ]
        self.last_reached_seq = -1

        self._push_in_flight = False
        self._reset_in_flight = False
        self._mode_request_in_flight = False
        self.mission_pushed = False

        self.timer = self.create_timer(1.0, self.sequence_callback)
        self.step = 'WAIT_CONNECTION'

    def state_callback(self, msg: State):
        self.current_state = msg

    def reached_callback(self, msg: WaypointReached):
        self.last_reached_seq = msg.wp_seq
        self.get_logger().info(f"Waypoint atteint: seq={msg.wp_seq}")

    def build_waypoint_list(self):
        wps = []

        # Item 0 : NAV_TAKEOFF (position 0, seul item marqué "current" au départ)
        takeoff_wp = Waypoint()
        takeoff_wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        takeoff_wp.command = 22          # MAV_CMD_NAV_TAKEOFF
        takeoff_wp.is_current = True
        takeoff_wp.autocontinue = True
        takeoff_wp.param1 = 0.0
        takeoff_wp.x_lat = self.waypoints[0][0]
        takeoff_wp.y_long = self.waypoints[0][1]
        takeoff_wp.z_alt = self.waypoints[0][2]
        wps.append(takeoff_wp)

        # Items 1 à N : les waypoints réels (jamais "current", le takeoff l'est déjà)
        for lat, lon, alt in self.waypoints:
            wp = Waypoint()
            wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp.command = 16               # MAV_CMD_NAV_WAYPOINT
            wp.is_current = False
            wp.autocontinue = True
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = alt
            wps.append(wp)

        return wps

    def sequence_callback(self):
        self.get_logger().info(
            f"État: {self.step} | Mode: {self.current_state.mode} | Dernier waypoint atteint: {self.last_reached_seq}"
        )

        if self.step == 'WAIT_CONNECTION':
            if self.current_state.connected:
                self.step = 'PUSH_MISSION'

        elif self.step == 'PUSH_MISSION':
            if not self.mission_pushed and not self._push_in_flight:
                self._push_in_flight = True
                req = WaypointPush.Request()
                req.start_index = 0
                req.waypoints = self.build_waypoint_list()
                future = self.push_client.call_async(req)
                future.add_done_callback(self.on_push_response)

        elif self.step == 'RESET_CURRENT':
            if not self._reset_in_flight:
                self._reset_in_flight = True
                req = WaypointSetCurrent.Request()
                req.wp_seq = 0
                future = self.set_current_client.call_async(req)
                future.add_done_callback(self.on_reset_response)

        elif self.step == 'SET_MODE':
            if self.current_state.mode == 'AUTO.MISSION':
                self.get_logger().info("Mode AUTO.MISSION confirmé actif.")
                self.step = 'MONITORING'
            elif not self._mode_request_in_flight:
                self._mode_request_in_flight = True
                req = SetMode.Request()
                req.custom_mode = 'AUTO.MISSION'
                future = self.set_mode_client.call_async(req)
                future.add_done_callback(self.on_set_mode_response)

        elif self.step == 'MONITORING':
            # +1 car l'item 0 est le NAV_TAKEOFF : le dernier waypoint réel
            # est à l'index len(self.waypoints), pas len(self.waypoints) - 1
            last_index = len(self.waypoints)
            if self.last_reached_seq == last_index:
                self.get_logger().info("Dernier waypoint atteint, déclenchement du RTL...")
                self.step = 'RTL'

        elif self.step == 'RTL':
            if self.current_state.mode == 'AUTO.RTL':
                self.get_logger().info("Mode AUTO.RTL confirmé actif.")
                self.step = 'DONE'
            elif not self._mode_request_in_flight:
                self._mode_request_in_flight = True
                req = SetMode.Request()
                req.custom_mode = 'AUTO.RTL'
                future = self.set_mode_client.call_async(req)
                future.add_done_callback(self.on_set_mode_response)

        elif self.step == 'DONE':
            self.get_logger().info("Mission + RTL terminés.")
            self.timer.cancel()
            rclpy.shutdown()

    def on_push_response(self, future):
        self._push_in_flight = False
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Exception lors de l'upload de la mission: {e}")
            return

        if result.success:
            self.get_logger().info(f"Mission uploadée ({result.wp_transfered} waypoints).")
            self.mission_pushed = True
            self.step = 'RESET_CURRENT'
        else:
            self.get_logger().error("Échec de l'upload de la mission, nouvelle tentative...")

    def on_reset_response(self, future):
        self._reset_in_flight = False
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Exception lors du reset de l'index mission: {e}")
            return

        if result.success:
            self.get_logger().info("Index de mission réinitialisé à 0.")
            self.step = 'SET_MODE'
        else:
            self.get_logger().error("Échec du reset, nouvelle tentative...")

    def on_set_mode_response(self, future):
        self._mode_request_in_flight = False
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Exception lors du changement de mode: {e}")
            return

        if not result.mode_sent:
            self.get_logger().error("Échec de l'envoi du changement de mode, nouvelle tentative...")


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()