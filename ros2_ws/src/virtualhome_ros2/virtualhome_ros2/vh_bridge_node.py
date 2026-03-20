import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, TransformStamped, Twist
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math
import threading
import time
import numpy as np

# Ensure navigator is importable
# Path inside workspace allowed bounds
NAV_DIR = '/data/code/virtualhome/agent_navigation'
if NAV_DIR not in sys.path:
    sys.path.append(NAV_DIR)

from navigator import AgentNavigator

class VHBridgeNode(Node):
    def __init__(self):
        super().__init__('vh_bridge_node')
        
        # 1. Simulator Path configuration
        self.exec_path = '/data/code/virtualhome/virtualhome/simulation/unity_simulator/linux_exec.v2.3.0.x86_64'
        self.get_logger().info(f"Using Simulator Exec: {self.exec_path}")
        
        # 2. Initialize AgentNavigator
        self.get_logger().info("Initializing AgentNavigator...")
        self.nav = AgentNavigator(exec_path=self.exec_path)
        
        # Reset scene to 0 (Standard room setup)
        self.nav.reset(0)
        self.get_logger().info("Environment graph initialized, adding agent...")
        self.nav.add_agent() 
        
        
        # 3. CV Bridge
        self.bridge = CvBridge()
        
        # 4. ROS Subscriptions & Publishers
        self.declare_parameter('enable_cameras', True)
        self.sub_cmd = self.create_subscription(Pose2D, '/virtualhome/cmd_pose', self.cmd_callback, 10)
        self.sub_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        
        # continuous kinematics buffers
        self.v_x = 0.0
        self.omega = 0.0
        self.last_cmd_vel_time = self.get_clock().now()
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        
        self.tick_count = 0 # For frequency decoupling
        
        self.pub_pose = self.create_publisher(Pose2D, '/virtualhome/debug_pose2d', 10)
        self.pub_rgb = self.create_publisher(Image, '/virtualhome/camera/color/image_raw', 10)
        self.pub_depth = self.create_publisher(Image, '/virtualhome/camera/depth/image_raw', 10)
        self.pub_seg = self.create_publisher(Image, '/virtualhome/camera/segmentation/image_raw', 10)
        self.pub_topview = self.create_publisher(Image, '/virtualhome/camera/topview/image_raw', 10)
        
        # --- Standard Robot Abstraction ---
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()

        # 5. Timer for Pose Tracking & Camera publishes at 20Hz (~0.05s)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Concurrency Thread Lock for Movement commands
        self.move_thread_lock = threading.Lock()
        self.target_pose = None
        
        # Simulator Socket Access Lock
        self.sim_lock = threading.Lock()
        
        # --- PHASE 5N: Scene Camera to bypass bone local space caches ---
        with self.sim_lock:
            pos_tuple = self.nav.get_position()
            ix, iy, iz = pos_tuple[0] if pos_tuple else (0.0, 1.0, 0.0)
            self.nav.comm.add_camera(position=[ix, 1.0, iz], rotation=[0,0,0], field_view=60)
            sc_cnt, cc_cnt = self.nav.comm.camera_count()
            self.scene_cam_idx = cc_cnt - 1 # absolute Scene Camera Index
        self.get_logger().info(f"Allocated Scene Camera Index: {self.scene_cam_idx} at absolute position {ix:.2f},{iz:.2f}")
        
        self.get_logger().info("VirtualHome ROS 2 Bridge Node online!")

    def cmd_callback(self, msg: Pose2D):
        self.get_logger().info(f"Target command received: X={msg.x:.2f}, Z={msg.y:.2f}")
        # Trigger thread to handle non-blocking iterations iteratively
        with self.move_thread_lock:
            self.target_pose = (msg.x, msg.y)
            
        # Spawn thread if not already running controlling loop
        thr = threading.Thread(target=self.control_loop_worker)
        thr.start()

    def control_loop_worker(self):
        import numpy as np # Needed for depth filters

        while rclpy.ok():
            # Check for /cmd_vel continuous timeout safety decays
            now = self.get_clock().now()
            dt_since_cmd = (now - self.last_cmd_vel_time).nanoseconds / 1e9
            if dt_since_cmd > 0.5:
                self.v_x = 0.0
                self.omega = 0.0
                
            with self.move_thread_lock:
                target = self.target_pose
                self.target_pose = None # consume direct target
            
            # Case A: Absolute command pose chase (/virtualhome/cmd_pose)
            if target is not None:
                target_x, target_z = target
                arrived = False
                tolerance = 0.05
                max_steps = 200
                step_cnt = 0
                
                while not arrived and step_cnt < max_steps:
                    with self.sim_lock:
                        pos_tuple = self.nav.get_position()
                    if not pos_tuple:
                        break
                    pos, _ = pos_tuple
                    curr_x, curr_z = pos[0], pos[2]
                    dx = target_x - curr_x
                    dz = target_z - curr_z
                    dist = (dx**2 + dz**2)**0.5
                    if dist < tolerance:
                         arrived = True
                         break
                    new_x = curr_x + (dx / dist) * 0.1
                    new_z = curr_z + (dz / dist) * 0.1
                    with self.sim_lock:
                        self.nav.move_character([new_x, pos[1], new_z])
                    step_cnt += 1
                    time.sleep(0.05)

            elif abs(self.v_x) > 0.01 or abs(self.omega) > 0.01:
                # Read from continuous state buffers updated by timer_callback
                curr_x = self.curr_x
                curr_z = self.curr_y # Note: Unity Z is mapped to ROS Y in our buffers
                curr_yaw = self.curr_yaw
                height = getattr(self, 'agent_height', 0.0)
                
                step_size = 0.1 # Enforce grid snapping step for discrete walk
                
                self.get_logger().info(f"Case B Loop Ticking: v_x={self.v_x:.2f}, omega={self.omega:.2f}")
                if abs(self.v_x) > 0.01:
                    angle_offset = self.omega * 0.2 # Scaling rotational mesh updates
                    dx = math.cos(curr_yaw + angle_offset) * step_size if self.v_x >= 0 else -math.cos(curr_yaw + angle_offset) * step_size
                    dz = math.sin(curr_yaw + angle_offset) * step_size if self.v_x >= 0 else -math.sin(curr_yaw + angle_offset) * step_size
                else:
                    # Pure rotation-in-place: Do NOT translate coordinate mesh
                    # Since update_character_camera() rigidly rotates the viewport, we don't need mesh rotations!
                    dx = 0.0 #np.random.uniform(-0.1, 0.1) # random (+0.1 - -0.1)
                    dz = 0.0
                    
                new_x = curr_x + dx
                new_z = curr_z + dz
                self.get_logger().info(f"Moving: new_x={new_x:.2f}, new_z={new_z:.2f} (from {curr_x:.2f},{curr_z:.2f})")
                with self.sim_lock:
                    succ = self.nav.move_character([new_x, height, new_z])
                self.get_logger().info(f"Move success: {succ}")
                        
                # Wait speed physical align velocities
                speed_val = max(abs(self.v_x), abs(self.omega), 0.01)
                sleep_dt = step_size / speed_val
                
                # --- Simulated Yaw continuous integrations ---
                if hasattr(self, 'simulated_yaw'):
                    self.simulated_yaw += self.omega * min(sleep_dt, 0.5)
                    unity_rot_y = 90.0 - math.degrees(self.simulated_yaw)
                    with self.sim_lock:
                        if hasattr(self, 'scene_cam_idx'):
                            self.nav.comm.update_camera(
                                self.scene_cam_idx, 
                                position=[new_x, 1.0, new_z], 
                                rotation=[0, unity_rot_y, 0]
                            )
                        else:
                            self.nav.comm.update_character_camera(rotation=[0, unity_rot_y, 0], name='agent_view')
                
                time.sleep(min(sleep_dt, 0.5))
            else:
                # No speeds, no queued pose. Relax and exit thread securely
                time.sleep(0.05)
                with self.move_thread_lock:
                    if self.target_pose is None and abs(self.v_x) <= 0.01 and abs(self.omega) <= 0.01:
                        setattr(self, '_loop_active', False)
                        break

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        # 1. Publish Pose
        with self.sim_lock:
            pos_tuple = self.nav.get_position()
        if pos_tuple:
            pos, rot = pos_tuple 
            unity_yaw = float(rot[1])
            ros_yaw_live = math.radians(90.0 - unity_yaw)
            
            # Simulated Yaw Fusion Buffer Setup (Initialize once)
            if not getattr(self, 'simulated_yaw_initialized', False):
                self.simulated_yaw = ros_yaw_live
                self.simulated_yaw_initialized = True
                
            ros_yaw = self.simulated_yaw
            
            p = Pose2D()
            p.x = float(pos[0])
            p.y = float(pos[2]) 
            p.theta = ros_yaw
            self.pub_pose.publish(p)
            
            # --- Kinematic Integrals ---
            self.curr_x = p.x
            self.curr_y = p.y
            self.curr_yaw = ros_yaw
            self.agent_height = float(pos[1])
            
            dt = 0.05 # 20Hz interval
            if abs(self.v_x) > 0.01 or abs(self.omega) > 0.01:
                tgt_x = self.curr_x + self.v_x * math.cos(self.curr_yaw) * dt
                tgt_y = self.curr_y + self.v_x * math.sin(self.curr_yaw) * dt
                # continuous setups feeds targeting forwards 
                with self.move_thread_lock:
                    self.target_pose = (float(tgt_x), float(tgt_y))
            
            # --- Odom & Dynamic TF ---
            now = self.get_clock().now().to_msg()
            
            # Odom
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = p.x
            odom.pose.pose.position.y = p.y
            odom.pose.pose.position.z = 0.0
            
            qz = math.sin(ros_yaw / 2.0)
            qw = math.cos(ros_yaw / 2.0)
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            self.pub_odom.publish(odom)
            
            # Dynamic TF: odom -> base_link
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = p.x
            t.transform.translation.y = p.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)
            
        self.tick_count += 1
        
        # 2. Publish Images (RGB & Depth) at ~5Hz (every 4 ticks @ 20Hz)
        if self.tick_count % 4 == 0:
            cams = []
            if hasattr(self, 'scene_cam_idx'):
                 cams.append(self.scene_cam_idx)
            elif self.nav.cam_idx is not None:
                 cams.append(self.nav.cam_idx)
            if hasattr(self.nav, 'top_cam_idx') and self.nav.top_cam_idx is not None:
                 cams.append(self.nav.top_cam_idx)
                 
            if cams and self.get_parameter('enable_cameras').value:
                with self.sim_lock:
                    succ_rgb, rgb_list = self.nav.comm.camera_image(cams, mode='normal')
                    succ_depth, depth_list = self.nav.comm.camera_image(cams, mode='depth')
                    succ_seg, seg_list = self.nav.comm.camera_image(cams, mode='seg_class')

                # RGB Publish
                if succ_rgb and rgb_list:
                    try:
                        msg = self.bridge.cv2_to_imgmsg(rgb_list[0], encoding='rgb8')
                        self.pub_rgb.publish(msg)
                        if len(rgb_list) > 1:
                            msg_top = self.bridge.cv2_to_imgmsg(rgb_list[1], encoding='rgb8')
                            self.pub_topview.publish(msg_top)
                    except Exception as e:
                        self.get_logger().error(f"RGB Publish error: {e}")

                # Depth Publish
                if succ_depth and depth_list:
                    try:
                        depth_single = depth_list[0][:, :, 0]
                        msg = self.bridge.cv2_to_imgmsg(depth_single.astype('float32'), encoding='32FC1')
                        self.pub_depth.publish(msg)
                        
                        # --- Depth Band To LaserScan ---
                        scan_msg = LaserScan()
                        scan_msg.header.stamp = now
                        scan_msg.header.frame_id = 'base_scan'
                        
                        fov_deg = 60.0
                        fov_rad = math.radians(fov_deg)
                        H, W = depth_single.shape
                        
                        scan_msg.angle_min = -fov_rad / 2.0
                        scan_msg.angle_max = fov_rad / 2.0
                        scan_msg.angle_increment = fov_rad / W
                        scan_msg.time_increment = 0.0
                        scan_msg.scan_time = 0.2
                        scan_msg.range_min = 0.1
                        scan_msg.range_max = 10.0
                        
                        h_center = H // 2
                        band = depth_single[h_center - 4 : h_center + 5, :]
                        center_row = np.min(band, axis=0)
                        scan_msg.ranges = [float(r) for r in center_row]
                        self.pub_scan.publish(scan_msg)
                    except Exception as e:
                        self.get_logger().error(f"Depth/Scan Publish error: {e}")

                # Seg Publish
                if succ_seg and seg_list:
                    try:
                        msg = self.bridge.cv2_to_imgmsg(seg_list[0], encoding='rgb8')
                        self.pub_seg.publish(msg)
                    except Exception as e:
                        self.get_logger().error(f"Seg Publish error: {e}")

    def publish_static_transforms(self):
        now = self.get_clock().now().to_msg()
        
        # 1. base_link -> camera_link (Static Eye Level)
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'base_link'
        t1.child_frame_id = 'camera_link'
        t1.transform.translation.x = 0.1
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 1.7 # eye level height
        t1.transform.rotation.w = 1.0 # 0 rotation
        
        # 2. camera_link -> camera_color_optical_frame (Standard Optical Offset)
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'camera_link'
        t2.child_frame_id = 'camera_color_optical_frame'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = -0.5
        t2.transform.rotation.y = 0.5
        t2.transform.rotation.z = -0.5
        t2.transform.rotation.w = 0.5
        
        # 3. base_link -> base_scan (Static Laser Scan Eye Level elevation offset)
        t3 = TransformStamped()
        t3.header.stamp = now
        t3.header.frame_id = 'base_link'
        t3.child_frame_id = 'base_scan'
        t3.transform.translation.x = 0.1
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 1.7 # eye level height
        t3.transform.rotation.w = 1.0 # 0 rotation
        
        self.static_tf_broadcaster.sendTransform([t1, t2, t3])

    def cmd_vel_callback(self, msg: Twist):
        self.v_x = msg.linear.x
        self.omega = msg.angular.z
        self.last_cmd_vel_time = self.get_clock().now()
        self.get_logger().info(f"Received Twist Callback: vx={self.v_x:.2f}, omega={self.omega:.2f}")
        
        # Trigger worker thread for continuous velocity integrals
        with self.move_thread_lock:
            if not getattr(self, '_loop_active', False):
                setattr(self, '_loop_active', True)
                threading.Thread(target=self.control_loop_worker).start()
        

def main(args=None):
    rclpy.init(args=args)
    node = VHBridgeNode()
    rclpy.spin(node)
    node.nav.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
