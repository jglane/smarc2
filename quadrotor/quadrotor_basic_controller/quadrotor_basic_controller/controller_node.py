import sys
import rclpy
from rclpy.node import Node
import numpy as np
from geodesy import utm

import rclpy.time
from smarc_msgs.msg import ThrusterRPM, ThrusterFeedback
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('controller_node')

        # _w : Unity world frame
        # _s : Taeyoung Lee world frame
        # _r : ROS world frame
        # _a : Unity body frame
        # _b : Taeyoung Lee body frame
        # _c : ROS body frame

        # Measurements
        self.t_odom_prev = None
        self.t_odom_curr = None
        self.x_w = None
        self.v_c = None
        self.R_wc = None
        self.W_c = None

        # Saved values
        self.t_prev = None
        self.R_sb_d_prev = np.eye(3)
        self.W_d_b_prev = np.zeros(3)
        self.R_sb_prev = np.eye(3)

        self.hold_control = True
        self.times = 0

        # Publishers
        self.prop1_cmd_pub = self.create_publisher(ThrusterRPM, '/prop1_cmd', 10)
        self.prop2_cmd_pub = self.create_publisher(ThrusterRPM, '/prop2_cmd', 10)
        self.prop3_cmd_pub = self.create_publisher(ThrusterRPM, '/prop3_cmd', 10)
        self.prop4_cmd_pub = self.create_publisher(ThrusterRPM, '/prop4_cmd', 10)

        self.prop1_cmd = ThrusterRPM()
        self.prop2_cmd = ThrusterRPM()
        self.prop3_cmd = ThrusterRPM()
        self.prop4_cmd = ThrusterRPM()

        # Subscribers
        #self.create_subscription(ThrusterFeedback, '/prop1_fb', self.fb_cb, 10)
        #self.create_subscription(ThrusterFeedback, '/prop2_fb', self.fb_cb, 10)
        #self.create_subscription(ThrusterFeedback, '/prop3_fb', self.fb_cb, 10)
        #self.create_subscription(ThrusterFeedback, '/prop4_fb', self.fb_cb, 10)

        #self.create_subscription(NavSatFix, '/drone/gps', self.gps_cb, 10)
        #self.create_subscription(Imu, '/drone/imu', self.imu_cb, 10)
        self.create_subscription(Odometry, '/Quadrotor/odom_gt', self.odom_cb, 10)

        self.pub = self.create_publisher(Wrench, '/drone/wrench', 10)

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        #self.tf_buffer = Buffer()
        #self.tf_listener = TransformListener(self.tf_buffer, self)

    def publish_rpms(self, rpms) -> None:
        self.prop1_cmd.rpm = rpms[0]
        self.prop2_cmd.rpm = rpms[1]
        self.prop3_cmd.rpm = rpms[2]
        self.prop4_cmd.rpm = rpms[3]
        
        self.prop1_cmd_pub.publish(self.prop1_cmd)
        self.prop2_cmd_pub.publish(self.prop2_cmd)
        self.prop3_cmd_pub.publish(self.prop3_cmd)
        self.prop4_cmd_pub.publish(self.prop4_cmd)
    
    def fb_cb(self, msg: ThrusterFeedback) -> None:
        pass
        #self.get_logger().error(f"Time: {self.get_clock().now().nanoseconds*1e-9}")
        #self.get_logger().error(f"RPM: {msg.rpm}")

    def gps_cb(self, msg: NavSatFix) -> None:
        drone_utm = utm.fromLatLong(msg.latitude, msg.longitude)
        
        #if self.t_prev == 0:
        #    self.t_prev = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        #else:
        #    t_now = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        #    dt = t_now - self.t_prev
        #    self.t_prev = t_now
        #    self.v_w[0] = (drone_utm.easting - self.x_w[0])/dt
        #    self.v_w[1] = (drone_utm.northing - self.x_w[1])/dt
        #    self.v_w[2] = (msg.altitude - self.x_w[2])/dt
        #    self.hold_control = False
        #self.get_logger().info(f"v: {self.v_w}")
        
        self.x_w[0] = drone_utm.easting - 651301.133
        self.x_w[1] = drone_utm.northing - 6524094.583
        self.x_w[2] = msg.altitude - 3.8518
        #self.get_logger().info(f"x: {self.x_w}")
        self.hold_control = False

        #self.get_logger().info(f"Tra t: {msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9}")

    def imu_cb(self, msg: Imu) -> None:
        # trying = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        # self.R_wa = self._quat2mat((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))@trying.T
        # #self.get_logger().info(f"R: {self.R_wa}")
        
        # self.W_w[0] = -msg.angular_velocity.y
        # self.W_w[1] = msg.angular_velocity.x
        # self.W_w[2] = msg.angular_velocity.z
        # #self.get_logger().info(f"W: {self.W_w}")
        pass

    def odom_cb(self, msg: Odometry) -> None:
        self.t_odom_curr = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

        self.x_w = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.v_c = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        self.R_wc = self._quat2mat((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.W_c = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        
        R_wr = np.array([[0, -1, 0],
                         [1, 0, 0],
                         [0, 0, 1]])
        R_rw = R_wr.T
        R_rc = R_rw@self.R_wc
        R_cr = R_rc.T
        #W_c = R_cr@self.W_r

        self.marker_array_pub.publish(MarkerArray(markers=[
            self._marker(R_rw@(self.x_w-np.array([0, 1000, 3.9])), "Quadrotor/odom_gt", (255, 255, 255), 4),
            self._marker(self.v_c, "Quadrotor/base_link_gt", (0, 0, 0), 5),
            self._marker(R_rc[:, 0], "Quadrotor/odom_gt", (255, 0, 0), 1),
            self._marker(R_rc[:, 1], "Quadrotor/odom_gt", (0, 255, 0), 2),
            self._marker(R_rc[:, 2], "Quadrotor/odom_gt", (0, 0, 255), 3),
            self._marker(self.W_c, "Quadrotor/base_link_gt", (200, 0, 255), 6)
        ]))

    def control_step(self, t: float) -> np.ndarray:
        # try:
        #     #tf = self.tf_buffer.lookup_transform('map_gt', 'Quadrotor/base_link_gt', rclpy.time.Time())#self.t_prev)
        #     #t_curr = tf.header.stamp.sec + tf.header.stamp.nanosec*1e-9

        #     if True:#self.t_prev is not None:
        #         #dt = t_curr - self.t_prev

        #         #self.x_w = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        #         #self.v_w = (self.x_w - self.x_w_prev)/dt

        #         #self.R_wa = self._quat2mat((tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w))@np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]).T
        #         #self.W_w = self._vee(self._logm3(self.R_wa_prev.T@self.R_wa)/dt)

        #         self.get_logger().info(f"\nx: {self.x_w}\nv: {self.v_c}\nR: {self.R_wc}\nW: {self.W_r}\n")
        #         #self.hold_control = False
                
        #     #self.t_prev = t_curr
        #     #self.x_w_prev = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])#self.x_w
        #     #self.R_wa_prev = self._quat2mat((tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w))@np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]).T#self.R_wa

        #     #self.get_logger().info(f"TF: {tf.translation}, {self._quat2mat((tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w))}")
        # except Exception as e:
        #     self.get_logger().error(f"{e}")
        #     return np.zeros(4)
        
        if self.t_odom_prev is None:
            self.t_odom_prev = self.t_odom_curr
            return np.zeros(4)
        
        dt = self.t_odom_curr - self.t_odom_prev
        self.t_odom_prev = self.t_odom_curr
        self.get_logger().error(f"dt: {dt}")

        if dt == 0:
            return np.zeros(4)

        # Parameters
        m = 4.34
        d = 0.315 #np.sqrt(2)*0.12 #np.sqrt(0.12**2 + 0.85**2)
        J = np.diag([0.0820, 0.0820, 0.1377])
        c_tau_f = 8.004e-2
        g = 9.81
        e3 = np.array([0, 0, 1])

        # Gains
        kx = 3.2*m
        kv = 2.8*m
        kR = 8.81
        kW = 2.54

        # Rotation matrices
        R_ws = np.array([[0, 1, 0],
                         [1, 0, 0],
                         [0, 0, -1]])
        R_wr = np.array([[0, -1, 0],
                         [1, 0, 0],
                         [0, 0, 1]])
        R_ab = R_ws
        R_ac = R_wr
        R_cb = R_ac.T@R_ab
        R_sw = R_ws.T
        R_sb = R_sw@self.R_wc@R_cb
        R_ba = R_ab.T
        R_sa = R_sb@R_ba
        R_sc = R_sa@R_ac
        R_bs = R_sb.T
        R_br = R_bs@R_sw@R_wr
        R_bc = R_cb.T

        # Transformations
        x_s = R_sw@self.x_w
        v_s = R_sc@self.v_c
        W_b = R_bc@self.W_c

        # Desired (in the {s} frame)
        #tf = 20
        x_d_s = lambda t: R_ws.T@np.array([0, 1000, 5])#20*np.array([1 - np.cos(np.pi*t/self.tf), np.sin(np.pi*t/self.tf), 0]) + self.x0
        v_d_s = lambda t: R_ws.T@np.array([0, 0, 0])#20*np.array([np.pi/self.tf*np.sin(np.pi*t/self.tf), np.pi/self.tf*np.cos(np.pi*t/self.tf), 0])
        a_d_s = lambda t: np.zeros(3)#20*np.array([np.pi**2/self.tf**2*np.cos(np.pi*t/self.tf), -np.pi**2/self.tf**2*np.sin(np.pi*t/self.tf), 0])
        b1d = np.array([1, 0, 0])
        
        # Errors
        ex = x_s-x_d_s(t)
        ev = v_s-v_d_s(t)

        pid = -kx*ex - kv*ev - m*g*e3 + m*a_d_s(t)
        #self.get_logger().info(f"PID: {pid}")
        b3d = -pid/np.linalg.norm(pid)
        b2d = np.cross(b3d, b1d)/np.linalg.norm(np.cross(b3d, b1d))
        R_sb_d = np.array([np.cross(b2d, b3d), b2d, b3d]).T

        W_d_b = self._vee(self._logm3(self.R_sb_d_prev.T@R_sb_d)/dt)
        W_d_dot_b = (W_d_b - self.W_d_b_prev)/dt
        #self.get_logger().warn(f"\n{R_sb}\n{R_sb_d}")

        eR = 0.5*self._vee(R_sb_d.T@R_sb - R_sb.T@R_sb_d)
        eW = W_b - R_sb.T@R_sb_d@W_d_b

        f = np.dot(-pid, R_sb@e3)
        M = -kR*eR - kW*eW + np.cross(W_b, J@W_b) - J@(self._hat(W_b)@R_sb.T@R_sb_d@W_d_b - R_sb.T@R_sb_d@W_d_dot_b)

        self.R_sb_d_prev = R_sb_d
        self.W_d_b_prev = W_d_b
        if self.times < 2:
            self.times += 1
            return np.zeros(4)

        wrench_b = np.array([M[0], M[1], M[2], 0, 0, -f])
        wrench_a = self._adjoint(R_ab.T, np.zeros(3)).T@wrench_b
        #wrench_a[0]=0; wrench_a[1]=0
        T = np.array([[1, 1, 1, 1], [d, 0, -d, 0], [0, -d, 0, d], [c_tau_f, -c_tau_f, c_tau_f, -c_tau_f]])
        F = np.linalg.inv(T)@np.array([wrench_a[5], wrench_a[0], wrench_a[1], wrench_a[2]])

        # pub_msg = Wrench()
        # pub_msg.torque.x = 0.0#-wrench_a[0]
        # pub_msg.torque.y = 0.0#-wrench_a[1]
        # pub_msg.torque.z = 0.0#-wrench_a[2]
        # pub_msg.force.x = -pid[0]#wrench_a[3]
        # pub_msg.force.y = -pid[1]#wrench_a[4]
        # pub_msg.force.z = -pid[2]#wrench_a[5]
        # self.pub.publish(pub_msg)

        des_s = -kx*ex - kv*ev
        R_rb = R_br.T
        R_rs = R_wr.T@R_ws
        R_rb_d = R_rs@R_sb_d
        # self.marker_array_pub.publish(MarkerArray(markers=[
        #     # self._marker(R_cs@des_s, "Quadrotor/base_link_gt", (0, 255, 0), 1),
        #     # self._marker(R_rb[:, 0], "Quadrotor/odom_gt", (255, 0, 0), 2),
        #     # self._marker(R_rb[:, 1], "Quadrotor/odom_gt", (0, 255, 0), 3),
        #     # self._marker(R_rb[:, 2], "Quadrotor/odom_gt", (0, 0, 255), 4),
        #     # self._marker(R_rb_d[:, 0], "Quadrotor/odom_gt", (255, 0, 255), 5),
        #     # self._marker(R_rb_d[:, 1], "Quadrotor/odom_gt", (255, 255, 0), 6),
        #     # self._marker(R_rb_d[:, 2], "Quadrotor/odom_gt", (0, 255, 255), 7),
        #     self._marker(R_cb@W_b, "Quadrotor/base_link_gt", (0, 0, 255), 0),
        #     self._marker(R_cb@W_d_b, "Quadrotor/base_link_gt", (0, 255, 255), 1),
        # ]))
        # self.marker_pub.publish(self._marker(R_cb@W_b, "Quadrotor/base_link_gt", (0, 0, 255), 0))
        # self.marker_pub.publish(self._marker(R_cb@(self._vee(self._logm3(self.R_sb_prev.T@R_sb)/dt)), "Quadrotor/base_link_gt", (0, 255, 255), 1))
        # self.R_sb_prev = R_sb
        
        #self._marker(self.W_a, "Quadrotor/base_link_gt", (0, 0, 255))

        #self._marker(wrench_a[:3], (127, 0, 255))
        #self._marker(wrench_a[3:], (255, 127, 0))
        
        #self.get_logger().info(f"Errors: {np.round(ex, 3)}, {np.round(ev, 3)}, {np.round(eR, 3)}, {np.round(eW, 3)}")
        #self.get_logger().info(f"Wrench: {np.round(wrench_a, 3)}")
        #self.get_logger().info(f"{t}")
        #self.get_logger().info(f"{self.R_wa} {R_sb}")
        #self.get_logger().info("\n--------------------------")

        return F#np.ones(4)*2129/1000*5
    
    def _hat(self, v: np.ndarray) -> np.ndarray:
        return np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]])
    
    def _vee(self, S: np.ndarray) -> np.ndarray:
        error = 1e-6
        assert S[0, 0] < error
        assert S[1, 1] < error
        assert S[2, 2] < error
        assert np.abs(S[1, 2] + S[2, 1]) < error
        assert np.abs(S[0, 2] + S[2, 0]) < error
        assert np.abs(S[0, 1] + S[1, 0]) < error
        return np.array([S[2, 1], S[0, 2], S[1, 0]])
    
    def _logm3(self, R: np.ndarray) -> np.ndarray:
        acosinput = (np.trace(R) - 1) / 2
        if acosinput >= 1:
            m_ret = np.zeros((3, 3))
        elif acosinput <= -1:
            if not self._near_zero(1 + R[2, 2]):
                omg = (1.0 / np.sqrt(2 * (1 + R[2, 2])))*np.array([R[0, 2], R[1, 2], 1 + R[2, 2]])
            elif not self._near_zero(1 + R[1, 1]):
                omg = (1.0 / np.sqrt(2 * (1 + R[1, 1])))*np.array([R[0, 1], 1 + R[1, 1], R[2, 1]])
            else:
                omg = (1.0 / np.sqrt(2 * (1 + R[0, 0])))*np.array([1 + R[0, 0], R[1, 0], R[2, 0]])
            m_ret = self._hat(np.pi * omg)
        else:
            theta = np.arccos(acosinput)
            m_ret = theta / 2 / np.sin(theta)*(R - R.T)
        return m_ret
    
    def _near_zero(self, val):
        return np.abs(val) < 1e-6
    
    def _adjoint(self, R: np.ndarray, p: np.ndarray) -> np.ndarray:
        return np.block([[R, np.zeros((3, 3))],
                         [self._hat(p)@R, R]])
    
    def _quat2mat(self, q: tuple):
        x, y, z, w = q
        Nq = w*w + x*x + y*y + z*z
        s = 2.0/Nq
        X = x*s
        Y = y*s
        Z = z*s
        wX = w*X; wY = w*Y; wZ = w*Z
        xX = x*X; xY = x*Y; xZ = x*Z
        yY = y*Y; yZ = y*Z; zZ = z*Z
        return np.array(
            [[ 1.0-(yY+zZ), xY-wZ, xZ+wY ],
            [ xY+wZ, 1.0-(xX+zZ), yZ-wX ],
            [ xZ-wY, yZ+wX, 1.0-(xX+yY) ]])
    
    def _marker(self, v: np.ndarray, frame_id: str, color: tuple, id: int) -> None:
        x, y, z = v/np.linalg.norm(v)
        ai = 0.0
        aj = np.arctan2(-z, np.sqrt(x**2 + y**2))
        ak = np.arctan2(y, x)
        
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = np.cos(ai)
        si = np.sin(ai)
        cj = np.cos(aj)
        sj = np.sin(aj)
        ck = np.cos(ak)
        sk = np.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*cc + sj*ss
        q[1] = cj*sc - sj*cs
        q[2] = cj*ss + sj*cc
        q[3] = cj*cs - sj*sc
        
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ns"
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = q[1]
        marker.pose.orientation.y = q[2]
        marker.pose.orientation.z = q[3]
        marker.pose.orientation.w = q[0]
        marker.scale.x = np.linalg.norm(v)
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = color[0]/255
        marker.color.g = color[1]/255
        marker.color.b = color[2]/255
        return marker

def controller():
    rclpy.init(args=sys.argv)
    node = ControllerNode()
    t0 = node.get_clock().now().nanoseconds*1e-9
    dt = 1/50

    def control_loop():
        t = node.get_clock().now().nanoseconds*1e-9 - t0
        #node.get_logger().info(f"t: {t}")
        #if t > node.tf:
        #    return
        F = node.control_step(t)
        rpm2force = 5
        rpms = F*1000/rpm2force
        rpms = [int(rpm) for rpm in rpms]
        #node.get_logger().info(f"Setting RPMs to {rpms}")
        node.publish_rpms(rpms)

    node.create_timer(dt, control_loop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

def test():
    rclpy.init(args=sys.argv)
    node = ControllerNode()

    rpm = 0

    def loop():
        nonlocal rpm
        #rpm = 0#10000 - rpm
        node.publish_rpms([rpm, rpm, rpm, rpm])
        #node.get_logger().info(f"Setting RPM to {rpm}")

    node.create_timer(0.5, loop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    test()