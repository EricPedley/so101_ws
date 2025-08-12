# -*- coding: utf-8 -*-
# Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# Copyright (c) 2023 PickNik, LLC. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
import re
import os

import carb
import numpy as np
from pathlib import Path

# In older versions of Isaac Sim (prior to 4.0), SimulationApp is imported from
# omni.isaac.kit rather than isaacsim.
from isaacsim import SimulationApp

ROBOT_STAGE_PATH = "/SO101"
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Room/simple_room.usd"
GRAPH_PATH = "/ActionGraph"
REALSENSE_VIEWPORT_NAME = "realsense_viewport"

CONFIG = {"renderer": "RayTracedLighting", "headless": False}

simulation_app = SimulationApp(CONFIG)

from isaacsim.core.api import SimulationContext  # noqa E402
from isaacsim.core.utils.prims import set_targets, set_prim_property  # noqa E402
from isaacsim.core.utils import (  # noqa E402
    extensions,
    prims,
    rotations,
    stage,
    viewports,
)
from isaacsim.storage.native import nucleus

from pxr import Gf, UsdGeom  # noqa E402
import omni.graph.core as og  # noqa E402
import omni
import isaacsim.core.utils.stage as stage_utils
from isaacsim.core.prims import SingleArticulation

# enable ROS2 bridge extension
# In older versions of Isaac Sim (prior to 4.5), the ROS 2 bridge is loaded from
# omni.isaac.ros2_bridge rather than isaacsim.ros2.bridge
extensions.enable_extension("isaacsim.ros2.bridge")

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

joint_names = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Preparing stage
viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))

# Loading the simple_room environment
stage.add_reference_to_stage(
    assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH
)

so101_usd_path = str(Path(__file__).parent / 'so101_new_calib.usd')

stage_utils.add_reference_to_stage(so101_usd_path, ROBOT_STAGE_PATH)

# wrap the prim as an articulation

ARTICULATION_NAME = "so101_articulation"
prim = SingleArticulation(prim_path=ROBOT_STAGE_PATH, name=ARTICULATION_NAME)

# add some objects, spread evenly along the X axis
# with a fixed offset from the robot in the Y and Z
prims.create_prim(
    "/cracker_box",
    "Xform",
    position=np.array([-0.2, -0.25, 0.15]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(1, 0, 0), -90)),
    usd_path=assets_root_path
    + "/Isaac/Props/YCB/Axis_Aligned_Physics/003_cracker_box.usd",
)
prims.create_prim(
    "/sugar_box",
    "Xform",
    position=np.array([-0.07, -0.25, 0.1]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 1, 0), -90)),
    usd_path=assets_root_path
    + "/Isaac/Props/YCB/Axis_Aligned_Physics/004_sugar_box.usd",
)
prims.create_prim(
    "/soup_can",
    "Xform",
    position=np.array([0.1, -0.25, 0.10]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(1, 0, 0), -90)),
    usd_path=assets_root_path
    + "/Isaac/Props/YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd",
)
prims.create_prim(
    "/mustard_bottle",
    "Xform",
    position=np.array([0.0, 0.15, 0.12]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(1, 0, 0), -90)),
    usd_path=assets_root_path
    + "/Isaac/Props/YCB/Axis_Aligned_Physics/006_mustard_bottle.usd",
)

simulation_app.update()

try:
    ros_domain_id = int(os.environ["ROS_DOMAIN_ID"])
    print("Using ROS_DOMAIN_ID: ", ros_domain_id)
except ValueError:
    print("Invalid ROS_DOMAIN_ID integer value. Setting value to 0")
    ros_domain_id = 0
except KeyError:
    print("ROS_DOMAIN_ID environment variable is not set. Setting value to 0")
    ros_domain_id = 0
try:
    og_keys_set_values = [
        ("Context.inputs:domain_id", ros_domain_id),
        ("PublishJointState.inputs:nodeNamespace", "isaac"),
        ("SubscribeJointState.inputs:nodeNamespace", "isaac"),
    ]

    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                (
                    "SubscribeJointState",
                    "isaacsim.ros2.bridge.ROS2SubscribeJointState",
                ),
                (
                    "ArticulationController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("OnTick", "omni.graph.action.OnTick"),
            ],
            og.Controller.Keys.CONNECT: [
                (
                    "OnImpulseEvent.outputs:execOut",
                    "PublishJointState.inputs:execIn",
                ),
                (
                    "OnImpulseEvent.outputs:execOut",
                    "SubscribeJointState.inputs:execIn",
                ),
                ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                (
                    "OnImpulseEvent.outputs:execOut",
                    "ArticulationController.inputs:execIn",
                ),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishJointState.inputs:timeStamp",
                ),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishClock.inputs:timeStamp",
                ),
                (
                    "SubscribeJointState.outputs:jointNames",
                    "ArticulationController.inputs:jointNames",
                ),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),
                ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                (
                    "createViewport.outputs:execOut",
                    "getRenderProduct.inputs:execIn",
                ),
                (
                    "createViewport.outputs:viewport",
                    "getRenderProduct.inputs:viewport",
                ),
            ],
            og.Controller.Keys.SET_VALUES: og_keys_set_values,
        },
    )
except Exception as e:
    print(e)


set_targets(
    prim=stage.get_current_stage().GetPrimAtPath("/ActionGraph/PublishJointState"),
    attribute="inputs:targetPrim",
    target_prim_paths=[ROBOT_STAGE_PATH + f'/so101_new_calib/joints/{joint_name}' for joint_name in joint_names],
)

set_targets(
    prim=stage.get_current_stage().GetPrimAtPath("/ActionGraph/ArticulationController"),
    attribute="inputs:targetPrim",
    target_prim_paths=[ROBOT_STAGE_PATH],
)

set_prim_property(
    prim_path="/ActionGraph/PublishJointState",
    property_name="inputs:nodeNamespace", 
    property_value = "isaac"
)


# Run app update for multiple frames to re-initialize the ROS action graph after setting new prim inputs
simulation_app.update()
simulation_app.update()

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()

simulation_context.play()
simulation_app.update()

while simulation_app.is_running():

    # Run with a fixed step size
    simulation_context.step(render=True)

    # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
    og.Controller.set(
        og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True
    )

simulation_context.stop()
simulation_app.close()
