import numpy as np
import time

from pydrake.geometry import (
    Box, IllustrationProperties, ProximityProperties
    )
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import (
    BallRpyJoint, FixedOffsetFrame, SpatialInertia,
    UnitInertia, UniversalJoint,
    )
from pydrake.math import RigidTransform
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer

builder = DiagramBuilder()
mbp, sg = AddMultibodyPlantSceneGraph(builder, 0.001)

inertia = SpatialInertia(1., np.zeros(3), UnitInertia(0.5, 0.5, 0.5))
child = mbp.AddRigidBody("Child", inertia)
mbp.RegisterVisualGeometry(child, RigidTransform(), Box(0.5, 0.2, 1.),
                             "visual", [0.3, 0.3, 0.3, 1.])
# mbp.RegisterCollisionGeometry(child, RigidTransform(), Box(0.5, 0.2, 1.),
#                                 "collision", ProximityProperties())
# geometry AddContactMaterial

pivot_frame = FixedOffsetFrame("pivot", child.body_frame(), RigidTransform([0, 0, 1.]))
mbp.AddFrame(pivot_frame)
# joint = BallRpyJoint("joint", mbp.world_frame(), pivot_frame)
joint = UniversalJoint("joint", mbp.world_frame(), pivot_frame)
joint_out = mbp.AddJoint(joint)

mbp.Finalize()

meshcat = builder.AddSystem(MeshcatVisualizer(sg, zmq_url="new"))
builder.Connect(sg.get_pose_bundle_output_port(), meshcat.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)

context = simulator.get_mutable_context()
mbp_context = diagram.GetMutableSubsystemContext(mbp, context)
# joint.set_angles(context, [1., 1., 0])
joint.set_angles(context, [1., 1])

time.sleep(1)
simulator.Initialize()
simulator.set_target_realtime_rate(1.0)
simulator.AdvanceTo(4.0)
