import rclpy
from rclpy.node import Node
import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import subprocess
import time

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)
from omni.isaac.ros2_bridge import read_camera_info

class ActionGraphManager():

    def __init__(self):
        self.create_ros_time_graph()

    def create_ros_time_graph(self):
        og.Controller.edit(
            {"graph_path": "/ClockGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                    ("OnPlayBack", "omni.graph.action.OnPlaybackTick"),
                ],
                og.Controller.Keys.CONNECT: [
                    # Connecting execution of OnImpulseEvent node to PublishClock so it will only publish when an impulse event is triggered
                    ("OnPlayBack.outputs:tick", "PublishClock.inputs:execIn"),
                    # Connecting simulationTime data of ReadSimTime to the clock publisher node
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # Assigning topic name to clock publisher
                    ("PublishClock.inputs:topicName", "/clock"),
                ],
            },
        ) 

    def create_lidar_graph(self):
        og.Controller.edit(
            {"graph_path": "/RTXLidarGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("IsaacCreateRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("LidarHelper", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
                    ("OnPlayBack", "omni.graph.action.OnPlaybackTick"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlayBack.outputs:tick", "IsaacCreateRenderProduct.inputs:execIn"),
                    ("IsaacCreateRenderProduct.outputs:execOut", "LidarHelper.inputs:execIn"),
                    ("IsaacCreateRenderProduct.outputs:renderProductPath", "LidarHelper.inputs:renderProductPath"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("IsaacCreateRenderProduct.inputs:cameraPrim", f"/World/Go2/base/rtx_lidar"),
                    ("IsaacCreateRenderProduct.inputs:enabled", True),
                    ("IsaacCreateRenderProduct.inputs:height", 720),
                    ("IsaacCreateRenderProduct.inputs:width", 1280),

                    ("LidarHelper.inputs:frameId", f"rtx_lidar"),
                    ("LidarHelper.inputs:enabled", True),
                    ("LidarHelper.inputs:topicName", f"point_cloud"),
                    ("LidarHelper.inputs:type", f"point_cloud"),
                ],
            },
        ) 

    def create_camera_graph(self):
        og.Controller.edit(
            {
                "graph_path": "/CameraROS2Graph",
                "evaluator_name": "execution",
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("IsaacCreateRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("ROS2CameraHelperColor", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("ROS2CameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("ROS2CameraHelperDepthCloud", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ],

                og.Controller.Keys.SET_VALUES: [
                    ("IsaacCreateRenderProduct.inputs:cameraPrim", f"/World/Go2/base/front_cam"),
                    ("IsaacCreateRenderProduct.inputs:enabled", True),
                    ("IsaacCreateRenderProduct.inputs:height", 480),
                    ("IsaacCreateRenderProduct.inputs:width", 640),
                    
                    # color camera
                    ("ROS2CameraHelperColor.inputs:type", "rgb"),
                    ("ROS2CameraHelperColor.inputs:topicName", f"go2/front_cam/color_image"),
                    ("ROS2CameraHelperColor.inputs:frameId", f"front_cam"),
                    ("ROS2CameraHelperColor.inputs:useSystemTime", True),

                    # depth camera
                    ("ROS2CameraHelperDepth.inputs:type", "depth"),
                    ("ROS2CameraHelperDepth.inputs:topicName", f"go2/front_cam/depth_image"),
                    ("ROS2CameraHelperDepth.inputs:frameId", f"front_cam"),
                    ("ROS2CameraHelperDepth.inputs:useSystemTime", True),
                ],

                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "IsaacCreateRenderProduct.inputs:execIn"),
                    ("IsaacCreateRenderProduct.outputs:execOut", "ROS2CameraHelperColor.inputs:execIn"),
                    ("IsaacCreateRenderProduct.outputs:renderProductPath", "ROS2CameraHelperColor.inputs:renderProductPath"),
                    ("IsaacCreateRenderProduct.outputs:execOut", "ROS2CameraHelperDepth.inputs:execIn"),
                    ("IsaacCreateRenderProduct.outputs:renderProductPath", "ROS2CameraHelperDepth.inputs:renderProductPath"),
                ],

            },
        )
    
    def create_odom_graph(self):
        og.Controller.edit(
            {"graph_path": "/OdomGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("IsaacComputeOdometry", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                    ("IsaacReadSimulationTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ROS2PublishOdometry", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                    ("ROS2PublishRawTransformTree", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                    ("ROS2PublishTransformTree_1", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ("ROS2PublishTransformTree_2", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ("OnPlayBack", "omni.graph.action.OnPlaybackTick"),
                ],
                og.Controller.Keys.CONNECT: [
                    # exec in
                    ("OnPlayBack.outputs:tick", "ROS2PublishOdometry.inputs:execIn"),
                    ("OnPlayBack.outputs:tick", "IsaacComputeOdometry.inputs:execIn"),
                    ("OnPlayBack.outputs:tick", "ROS2PublishRawTransformTree.inputs:execIn"),
                    ("OnPlayBack.outputs:tick", "ROS2PublishTransformTree_1.inputs:execIn"),
                    ("OnPlayBack.outputs:tick", "ROS2PublishTransformTree_2.inputs:execIn"),
                    # sim time
                    ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishRawTransformTree.inputs:timeStamp"),
                    ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishTransformTree_1.inputs:timeStamp"),
                    ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishTransformTree_2.inputs:timeStamp"),
                    # 
                    ("IsaacComputeOdometry.outputs:orientation", "ROS2PublishRawTransformTree.inputs:rotation"),
                    ("IsaacComputeOdometry.outputs:position", "ROS2PublishRawTransformTree.inputs:translation"),
                    #
                    ("IsaacComputeOdometry.outputs:orientation", "ROS2PublishRawTransformTree.inputs:orientation"),
                    ("IsaacComputeOdometry.outputs:position", "ROS2PublishRawTransformTree.inputs:position"),
                    ("IsaacComputeOdometry.outputs:angularVelocity", "ROS2PublishRawTransformTree.inputs:angularVelocity"),
                    ("IsaacComputeOdometry.outputs:linearVelocity", "ROS2PublishRawTransformTree.inputs:linearVelocity"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("IsaacComputeOdometry.inputs:chassisPrim", f"/World/Go2"),

                    ("ROS2PublishTransformTree_1.inputs:parentPrim", f"/World/Go2/base"),
                    ("ROS2PublishTransformTree_1.inputs:targetPrims", f"/World/Go2"),

                    ("ROS2PublishRawTransformTree.inputs:childFrameId", f"base"),
                    ("ROS2PublishRawTransformTree.inputs:parentFrameId", f"odom"),
                    ("ROS2PublishRawTransformTree.inputs:topicName", f"tf"),

                    ("ROS2PublishOdometry.inputs:chassisFrameId", f"base"),
                    ("ROS2PublishOdometry.inputs:odomFrameId", f"odom"),
                    ("ROS2PublishOdometry.inputs:topicName", f"odom"),

                    ("ROS2PublishTransformTree_2.inputs:parentPrim", f"/World/Go2/base"),
                    ("ROS2PublishTransformTree_2.inputs:targetPrims", f"/World/Go2/base/rtx_lidar"),
                ],
            },
        ) 
    

