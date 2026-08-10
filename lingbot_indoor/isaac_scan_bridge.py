"""
Ponte UDP -> ROS2: recebe o scan+pose do Isaac (run_isaac_teleop_lidar.py) por UDP e publica
/scan (LaserScan) + TF (map->odom->base_link->laser) para o RViz2. Corre no PYTHON DO SISTEMA
(ROS2 Humble, py3.10 — onde rclpy/tf2 funcionam).

Uso:
  source /opt/ros/humble/setup.bash
  python3 isaac_scan_bridge.py            # (python3 do sistema)
Opções: --port 9092
Depois: rviz2 -d lab_v4.rviz   (fixed frame = odom)
"""
import socket, struct, sys, math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

PORT = int(sys.argv[sys.argv.index("--port")+1]) if "--port" in sys.argv else 9092
RANGE_MAX = 12.0


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))


class ScanBridge(Node):
    def __init__(self):
        super().__init__("isaac_scan_bridge")
        self.pub = self.create_publisher(LaserScan, "/scan", 10)
        self.pub_pc = self.create_publisher(PointCloud2, "/points", 5)
        self.tfb = TransformBroadcaster(self)
        stf = StaticTransformBroadcaster(self)
        # estáticos: map->odom (identidade) e base_link->laser (lidar 0.35m acima)
        now = self.get_clock().now().to_msg()
        s1 = TransformStamped(); s1.header.stamp = now; s1.header.frame_id = "map"
        s1.child_frame_id = "odom"; s1.transform.rotation.w = 1.0
        s2 = TransformStamped(); s2.header.stamp = now; s2.header.frame_id = "base_link"
        s2.child_frame_id = "laser"; s2.transform.translation.z = 0.35; s2.transform.rotation.w = 1.0
        stf.sendTransform([s1, s2])
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", PORT)); self.sock.setblocking(False)
        self.n = 0
        self.create_timer(0.02, self.tick)     # 50Hz a drenar o socket
        self.get_logger().info(f"a ouvir scan UDP {PORT} -> /scan + TF (fixed frame: odom)")

    def tick(self):
        pkt = None
        while True:
            try:
                pkt = self.sock.recv(65535)          # cabe o pacote 3D (rings*az floats)
            except BlockingIOError:
                break
        if pkt is None:
            return
        # cabeçalho: pose(4f) + rays(i) rings(i) + vfov(f) range(f) + ranges(rings*rays f)
        x, y, z, yaw = struct.unpack("<4f", pkt[:16])
        rays, rings, vfov, rmax = struct.unpack("<iiff", pkt[16:32])
        ntot = rays * rings
        rr = struct.unpack(f"<{ntot}f", pkt[32:32 + 4 * ntot])
        now = self.get_clock().now().to_msg()
        # TF odom->base_link (pose do robô)
        t = TransformStamped(); t.header.stamp = now; t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(x); t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        t.transform.rotation.x = qx; t.transform.rotation.y = qy
        t.transform.rotation.z = qz; t.transform.rotation.w = qw
        self.tfb.sendTransform(t)

        # /scan (LaserScan) — anel do meio (horizontal), mantém o SLAM 2D a funcionar
        mid = rings // 2
        ring_mid = rr[mid * rays:(mid + 1) * rays]
        sc = LaserScan(); sc.header.stamp = now; sc.header.frame_id = "laser"
        sc.angle_min = -math.pi; sc.angle_max = math.pi
        sc.angle_increment = (2 * math.pi) / rays
        sc.range_min = 0.2; sc.range_max = rmax
        sc.ranges = [float(r) for r in ring_mid]
        self.pub.publish(sc)

        # /points (PointCloud2) — só em 3D (rings>1); reconstrói XYZ no frame do laser
        if rings > 1:
            ainc = (2 * math.pi) / rays
            vr = math.radians(vfov)
            pts = []
            for k in range(rings):
                el = (-vr / 2) + k * (vr / (rings - 1))
                ce, se = math.cos(el), math.sin(el)
                base = k * rays
                for i in range(rays):
                    r = rr[base + i]
                    if r >= rmax - 0.01:
                        continue                       # sem retorno -> não envia ponto
                    a = -math.pi + i * ainc
                    pts.append((r * math.cos(a) * ce, r * math.sin(a) * ce, r * se))
            hdr = Header(); hdr.stamp = now; hdr.frame_id = "laser"
            self.pub_pc.publish(point_cloud2.create_cloud_xyz32(hdr, pts))

        self.n += 1
        if self.n % 30 == 0:
            self.get_logger().info(f"scan #{self.n} {rings}anel×{rays}az pose=({x:.2f},{y:.2f})")


def main():
    rclpy.init()
    node = ScanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
