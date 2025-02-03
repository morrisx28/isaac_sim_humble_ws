import argparse
from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

import omni
import carb
import numpy as np
import sys
from omni.isaac.core import World
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.nucleus import get_assets_root_path
from go2.go2_ctrl import Go2FlatTerrainPolicy
from go2.go2_sensors import SensorManager
from go2.go2_action_graph import ActionGraphManager
# from ros2.go2_ros2_bridge import RobotDataManager
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.extensions import enable_extension

from nav_msgs.msg import Odometry
# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

import time

# Note that this is not the system level rclpy, but one compiled for omniverse
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dataclasses import dataclass

@dataclass
class Pose:
    position = np.zeros(3)
    orientation = np.array([1.0, 0.0, 0.0, 0.0])



class Go2Nav(Node):
    def __init__(self, env_usd_path, spot_init_pose: Pose):
        super().__init__("Go2_Nav")

        # setting up environment
        self.loadEnv(env_usd_path)

        self.first_step = True
        self.reset_needed = False
        self.cmd_scale = 1
        self.cmd = np.zeros(3) # [v_x, v_y, w_z]
        self.initSpot(spot_init_pose)

        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.go2CmdCallback, 1)
        # self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        # self.create_timer(0.1, self.pubOdomData)
        self.world.reset()

    def go2CmdCallback(self, msg: Twist):
        if self.world.is_playing():
            self.cmd[0] = msg.linear.x * self.cmd_scale
            self.cmd[2] = msg.angular.z * self.cmd_scale

    def generateOdomMsg(self, base_pos, base_rot, base_lin_vel_b, base_ang_vel_b):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base"
        odom_msg.pose.pose.position.x = base_pos[0].item()
        odom_msg.pose.pose.position.y = base_pos[1].item()
        odom_msg.pose.pose.position.z = base_pos[2].item()
        odom_msg.pose.pose.orientation.x = base_rot[1].item()
        odom_msg.pose.pose.orientation.y = base_rot[2].item()
        odom_msg.pose.pose.orientation.z = base_rot[3].item()
        odom_msg.pose.pose.orientation.w = base_rot[0].item()
        odom_msg.twist.twist.linear.x = base_lin_vel_b[0].item()
        odom_msg.twist.twist.linear.y = base_lin_vel_b[1].item()
        odom_msg.twist.twist.linear.z = base_lin_vel_b[2].item()
        odom_msg.twist.twist.angular.x = base_ang_vel_b[0].item()
        odom_msg.twist.twist.angular.y = base_ang_vel_b[1].item()
        odom_msg.twist.twist.angular.z = base_ang_vel_b[2].item()
        self.odom_pub.publish(odom_msg)
    
    def pubOdomData(self):
        self.generateOdomMsg(self.base_pos, self.base_rot, self.base_lin_vel, self.base_ang_vel)

    def loadEnv(self, env_usd_path):
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            simulation_app.close()
            sys.exit()
        self.world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=1 / 50)

        prim = define_prim("/World/Environment", "Xform")
        asset_path = env_usd_path
        prim.GetReferences().AddReference(asset_path)
    
    def onPhysicStep(self, step_size):
        if self.first_step:
            self.go2.initialize()
            self.first_step = False
        elif self.reset_needed:
            self.world.reset(True)
            self.reset_needed = False
            self.first_step = True
        else:
            self.go2.advance(step_size, self.cmd)
            self.base_pos, self.base_rot, self.base_lin_vel, self.base_ang_vel = self.go2.get_odom_info()
        

    def initSpot(self, spot_init_pose: Pose):
        self.go2 = Go2FlatTerrainPolicy(
            prim_path="/World/Go2",
            name="Go2",
            usd_path="omniverse://localhost/Library/go2.usd",
            position=spot_init_pose.position,
            orientation=spot_init_pose.orientation
        )
        sm = SensorManager()
        go2_lidar = sm.add_camera()
        go2_cam = sm.add_rtx_lidar()
        am = ActionGraphManager()
        am.create_camera_graph()
        am.create_lidar_graph()
        # am.create_odom_graph()
        self.world.add_physics_callback("physics_step", callback_fn=self.onPhysicStep)
        self.world.reset()

    def run_simulation(self):
        self.reset_needed = False
        while simulation_app.is_running():
            self.world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.world.is_stopped():
                self.reset_needed = True

        # Cleanup
        self.destroy_node()
        simulation_app.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--env",
        type=str,
        choices=["lobby", "b1"],
        default="b1",
        help="Choice of navigation environment.",
    )
    args, _ = parser.parse_known_args()

    B1_USD_PATH = "omniverse://localhost/Library/tsmc_b1_map.usd"
    LOBBY_USD_PATH = "omniverse://localhost/Library/tsmc_1f_map.usd"
    go2_init_pose = Pose()
    if args.env == "b1":
        ENV_USD_PATH = B1_USD_PATH
        go2_init_pose.position = np.array([6.65, -66.1, 0.8])
    elif args.env == "lobby":
        ENV_USD_PATH = LOBBY_USD_PATH
        go2_init_pose.position = np.array([214, -257.46, 0.8])
        go2_init_pose.orientation = np.array([0.707, 0, 0, 0.707])

    rclpy.init()
    subscriber = Go2Nav(ENV_USD_PATH, go2_init_pose)
    subscriber.run_simulation()