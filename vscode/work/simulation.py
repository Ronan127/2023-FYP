import sys
print("\n".join(sys.path))

# Import some basic libraries and functions for this tutorial.
import numpy as np
import os
from underactuated.meshcat_utils import MeshcatSliders

from pydrake.common import temp_directory
from pydrake.geometry import (
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Role,
    StartMeshcat,
)
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import ModelVisualizer

# Start the visualizer. The cell will output an HTTP link after the execution.
# Click the link and a MeshCat tab should appear in your browser.
meshcat = StartMeshcat()

# First we'll choose one of Drake's example model files, a KUKA iiwa arm.
myrobot_url = (
    "file:///home/ronan/2023-FYP/vscode/my-robot/robot.sdf")

# Create a model visualizer and add the robot arm.
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModels(url=myrobot_url)
# When this notebook is run in test mode it needs to stop execution without
# user interaction. For interactive model visualization you won't normally
# need the 'loop_once' flag.
test_mode = True if "TEST_SRCDIR" in os.environ else False

# Start the interactive visualizer.
# Click the "Stop Running" button in MeshCat when you're finished.
visualizer.Run(loop_once=test_mode)

# Define a simple cylinder model.
cylinder_sdf = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="cylinder">
    <pose>0 0 0 0 0 0</pose>
    <link name="cylinder_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.005833</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.005833</iyy>
          <iyz>0.0</iyz>
          <izz>0.005</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.2</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.2</length>
          </cylinder>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

# Create a Drake temporary directory to store files.
# Note: this tutorial will create a temporary file (table_top.sdf)
# in the `/tmp/robotlocomotion_drake_xxxxxx` directory.
temp_dir = temp_directory()

# Create a table top SDFormat model.
table_top_sdf_file = os.path.join(temp_dir, "table_top.sdf")
table_top_sdf = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="table_top">
    <link name="table_top_link">
      <visual name="visual">
        <pose>0 0 0.445 0 0 0</pose>
        <geometry>
          <box>
            <size>25 25 0.05</size>
          </box>
        </geometry>
        <material>
         <diffuse>0.9 0.8 0.7 1.0</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <pose>0 0 0.445  0 0 0</pose>
        <geometry>
          <box>
            <size>25 25 0.05</size>
          </box>
        </geometry>
      </collision>
    </link>
    <frame name="table_top_center">
      <pose relative_to="table_top_link">0 0 0.47 0 0 0</pose>
    </frame>
  </model>
</sdf>

"""

with open(table_top_sdf_file, "w") as f:
    f.write(table_top_sdf)


def create_scene(sim_time_step):
    # Clean up the Meshcat instance.
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=sim_time_step)
    parser = Parser(plant)

    # Loading models.
    # Load the table top and the cylinder we created.
    parser.AddModelsFromString(cylinder_sdf, "sdf")
    parser.AddModels(table_top_sdf_file)
    # Load a cracker box from Drake. 
    # parser.AddModels(
    #     url="package://drake/manipulation/models/ycb/sdf/003_cracker_box.sdf")
    # Load an OBJ file from Drake, with no SDFormat wrapper file. In this case,
    # the mass and inertia are inferred based on the volume of the mesh as if
    # it were filled with water, and the mesh is used for both collision and
    # visual geometry.
    # parser.AddModels(
    #     url="package://drake_models/ycb/meshes/004_sugar_box_textured.obj")

    onshape = parser.AddModels(url=myrobot_url)    

    # Weld the table to the world so that it's fixed during the simulation.
    table_frame = plant.GetFrameByName("table_top_center")
    plant.WeldFrames(plant.world_frame(), table_frame)
    # Finalize the plant after loading the scene.
    plant.Finalize()
    # We use the default context to calculate the transformation of the table
    # in world frame but this is NOT the context the Diagram consumes.
    plant_context = plant.CreateDefaultContext()

    # Set the initial pose for the free bodies, i.e., the custom cylinder,
    # the cracker box, and the sugar box.
    cylinder = plant.GetBodyByName("cylinder_link")
    X_WorldTable = table_frame.CalcPoseInWorld(plant_context)
    X_TableCylinder = RigidTransform(
        RollPitchYaw(np.asarray([90, 0, 0]) * np.pi / 180), p=[0,0,0.5])
    X_WorldCylinder = X_WorldTable.multiply(X_TableCylinder)
    plant.SetDefaultFreeBodyPose(cylinder, X_WorldCylinder)

    # cracker_box = plant.GetBodyByName("base_link_cracker")
    # X_TableCracker = RigidTransform(
    #     RollPitchYaw(np.asarray([45, 30, 0]) * np.pi / 180), p=[0,0,0.8])
    # X_WorldCracker = X_WorldTable.multiply(X_TableCracker)
    # plant.SetDefaultFreeBodyPose(cracker_box, X_WorldCracker)

    # sugar_box = plant.GetBodyByName("004_sugar_box_textured")
    # X_TableSugar = RigidTransform(p=[0,-0.25,0.8])
    # X_WorldSugar = X_WorldTable.multiply(X_TableSugar)
    # plant.SetDefaultFreeBodyPose(sugar_box, X_WorldSugar)

    wheelbody = plant.GetBodyByName("body")
    X_Tablebody = RigidTransform(p=[0,-0.25,0.8])
    X_Worldbody = X_WorldTable.multiply(X_Tablebody)
    plant.SetDefaultFreeBodyPose(wheelbody, X_Worldbody)

    meshcat.AddSlider('a', min=-5, max=5, step=.1, value=0.0)
    meshcat.AddSlider('b', min=-5, max=5, step=.1, value=0.0)
    meshcat.AddSlider('c', min=-5, max=5, step=.1, value=0.0)
    meshcat.AddSlider('d', min=-5, max=5, step=.1, value=0.0)
    meshcat.AddSlider('e', min=-5, max=5, step=.1, value=0.0)
    meshcat.AddSlider('F', min=-5, max=5, step=.1, value=0.0)
    meshcat.AddSlider('g', min=-5, max=5, step=.1, value=0.0)
    meshcat.AddSlider('h', min=-5, max=5, step=.1, value=0.0)
    meshcat.AddSlider('i', min=-5, max=5, step=.1, value=0.0)
    meshcat.AddSlider('j', min=-5, max=5, step=.1, value=0.0)
    meshcat.AddSlider('k', min=-5, max=5, step=.1, value=0.0)
    meshcat.AddSlider('L', min=-5, max=5, step=.1, value=0.0)

    
    torque_system = builder.AddSystem(MeshcatSliders(meshcat,['abcdeFghijkL']))
    builder.Connect(torque_system.get_output_port(), plant.get_input_port(7))
    
    # Add visualizer to visualize the geometries.
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat,
        MeshcatVisualizerParams(role=Role.kPerception, prefix="visual"))

    diagram = builder.Build()

    # plant.GetInputPort("wheel2_speed").FixValue(plant_context, [0])

    return diagram, visualizer

def initialize_simulation(diagram):
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.)
    return simulator

def run_simulation(sim_time_step):
    diagram, visualizer = create_scene(sim_time_step)
    simulator = initialize_simulation(diagram)
    meshcat.AddButton('Stop Simulation')
    while meshcat.GetButtonClicks('Stop Simulation') < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)

# Run the simulation with a small time step. Try gradually increasing it!
run_simulation(sim_time_step=0.0001)