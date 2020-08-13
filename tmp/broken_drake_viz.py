from pydrake.geometry import ConnectDrakeVisualizer, SceneGraph
from pydrake.lcm import DrakeLcm
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

builder = DiagramBuilder()
scene_graph = builder.AddSystem(SceneGraph())
ConnectDrakeVisualizer(builder=builder, scene_graph=scene_graph, lcm=DrakeLcm())

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.Initialize()
