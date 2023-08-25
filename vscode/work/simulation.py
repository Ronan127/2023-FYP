import sys
import re
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
from pydrake.systems.framework import DiagramBuilder, BasicVector, LeafSystem
from pydrake.visualization import ModelVisualizer
from pydrake.all import JointIndex
# Start the visualizer. The cell will output an HTTP link after the execution.
# Click the link and a MeshCat tab should appear in your browser.
meshcat = StartMeshcat()

# First we'll choose one of Drake's example model files, a KUKA iiwa arm.
myrobot_url = (
    "file:///home/ronan/2023-FYP/vscode/my-robot/robot.sdf")

# Create a model visualizer and add the robot arm.
# visualizer = ModelVisualizer(meshcat=meshcat)
# visualizer.parser().AddModels(url=myrobot_url)
# # When this notebook is run in test mode it needs to stop execution without
# # user interaction. For interactive model visualization you won't normally
# # need the 'loop_once' flag.
# test_mode = True if "TEST_SRCDIR" in os.environ else False

# # Start the interactive visualizer.
# # Click the "Stop Running" button in MeshCat when you're finished.
# visualizer.Run(loop_once=test_mode)

FRICTION_COEFFICIENT = 1
STEP_HEIGHT = 0.10
STEP_PITCH = 0.20

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
        <pose>0 0 0 0 0 0</pose>
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
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>25 25 0.05</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>{}</mu>
              <mu2>{}</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
    <frame name="table_top_center">
      <pose relative_to="table_top_link">0 0 0.025 0 0 0</pose>
    </frame>
  </model>
</sdf>

""".format(FRICTION_COEFFICIENT, FRICTION_COEFFICIENT)

with open(table_top_sdf_file, "w") as f:
    f.write(table_top_sdf)

#Create box to climb
box_sdf = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="box">
    <link name="box_link">
      <visual name="visual">
        <pose>0 0 0.445 0 0 0</pose>
        <geometry>
          <box>
            <size>1 1 0.25</size>
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
            <size>1 1 0.25</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>{}</mu>
              <mu2>{}</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
    <frame name="box_center">
      <pose relative_to="box_link">0 0 0.47 0 0 0</pose>
    </frame>
  </model>
</sdf>

""".format(FRICTION_COEFFICIENT,FRICTION_COEFFICIENT)

#Create stairs to climb
stair_sdf = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="box">
    <link name="box_link">
      <visual name="step_1_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>1 1 {h}</size>
          </box>
        </geometry>
        <material>
         <diffuse>0.9 0.8 0.7 1.0</diffuse>
        </material>
      </visual>
      <collision name="step_1_collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
           <size>1 1 {h}</size>
          </box>
        </geometry>
      </collision>
      <visual name="step_2_visual">
        <pose>0 -{l} {h} 0 0 0</pose>
        <geometry>
          <box>
            <size>1 1 {h}</size>
          </box>
        </geometry>
        <material>
         <diffuse>0.9 0.8 0.7 1.0</diffuse>
        </material>
      </visual>
      <collision name="step_2_collision">
        <pose>0 -{l} {h} 0 0 0</pose>
        <geometry>
          <box>
            <size>1 1 {h}</size>
          </box>
        </geometry>
      </collision>
            <visual name="step_3_visual">
        <pose>0 -{l2} {h2} 0 0 0</pose>
        <geometry>
          <box>
            <size>1 1 {h}</size>
          </box>
        </geometry>
        <material>
         <diffuse>0.9 0.8 0.7 1.0</diffuse>
        </material>
      </visual>
      <collision name="step_3_collision">
        <pose>0 -{l2} {h2} 0 0 0</pose>
        <geometry>
          <box>
            <size>1 1 {h}</size>
          </box>
        </geometry>
      </collision>
            <visual name="step_4_visual">
        <pose>0 -{l3} {h3} 0 0 0</pose>
        <geometry>
          <box>
            <size>1 1 {h}</size>
          </box>
        </geometry>
        <material>
         <diffuse>0.9 0.8 0.7 1.0</diffuse>
        </material>
      </visual>
      <collision name="step_4_collision">
        <pose>0 -{l3} {h3} 0 0 0</pose>
        <geometry>
          <box>
            <size>1 1 {h}</size>
          </box>
        </geometry>
      </collision>
    </link>
    <frame name="box_center">
      <pose relative_to="box_link">0 0 -{halfh}/2 0 0 0</pose>
    </frame>
  </model>
</sdf>

""".format(h=STEP_HEIGHT, h2=STEP_HEIGHT*2, h3=STEP_HEIGHT*3, halfh = STEP_HEIGHT/2, l=STEP_PITCH , l2=STEP_PITCH*2, l3=STEP_PITCH*3)

class DC_motor(LeafSystem):
    def __init__(self, InputVector, OutputVector):
        super().__init__()  # Don't forget to initialize the base class.
        num_motors = max(InputVector)
        num_input_states = len(InputVector)
        num_output_states = len(OutputVector)
        self._a_port = self.DeclareVectorInputPort(name="Voltage", size=num_motors)
        self._b_port = self.DeclareVectorInputPort(name="ModelState", size=num_input_states)
        self.DeclareVectorOutputPort(name="torque", size=num_output_states, calc=self.CalcTorque)
        self.TorqueVector = [0]*num_output_states
        self.Speed = [0]*num_motors
        self.SpeedFiltered = [0]*num_motors
        self.InputVector = InputVector
        self.OutputVector = OutputVector
        self.StallTorque = 6 #Nm
        self.NLSpeed = 0.61 #6 rpm
        self.Torque = [0]*num_motors

    
    def CalcTorque(self, context, output):
        Voltage = self._a_port.Eval(context)
        ModelState = self._b_port.Eval(context)
        index = 0
        for i, val in enumerate(self.InputVector):
            if val != 0:
                self.Speed[index] = ModelState[i]
                self.SpeedFiltered[index] = 0.99*self.SpeedFiltered[index]+0.01*self.Speed[index]
                index += 1

        for i, speed in enumerate(self.SpeedFiltered):
            self.Torque[i] = self.StallTorque*(Voltage[i]/12.0-speed/self.NLSpeed)
            # self.Torque[i] =Voltage[i]

        for i, val in enumerate(self.OutputVector):
            if val == 0:
                self.TorqueVector[i] = 0.0
            else: 
                self.TorqueVector[i] = self.Torque[val-1]
        #torquevector = [0.0, 0.0, 0.0, 0.0, 0.0, Voltage[0], 0.0, 0.0, 0.0, 0.0, 0.0, -Voltage[1]]
        output.SetFromVector(self.TorqueVector)

class ControlSystem(LeafSystem):
    def __init__(self, names):
        super().__init__()  # Don't forget to initialize the base class.
        self._a_port = self.DeclareVectorInputPort(name="Input", size=1)
        self._b_port = self.DeclareVectorInputPort(name="ModelState", size=37)
        self.DeclareVectorOutputPort(name="Voltage", size=2, calc=self.CalcVoltage)
        self.names = names
        JointNames={"motor_a_q","motor_b_q","motor_a_w","motor_b_w","sun_a_q","sun_b_q","sun_a_w","sun_b_w"}
        self.NameVector={}
        for i, name in enumerate(names):
            for j, JointName in enumerate(JointNames):
                if name == JointName:
                    self.NameVector[JointName]=i
        pass
      
    def clamp(self, n, minn, maxn):
      return max(min(maxn, n), minn)

    
    def CalcVoltage(self, context, output):
        Voltage = self._a_port.Eval(context)
        ModelState = self._b_port.Eval(context)
        States={}
        for name in self.NameVector:
            States[name] = ModelState[self.NameVector[name]]
        lim_a_q=States["motor_a_q"]-States["sun_a_q"]
        lim_b_q=States["motor_b_q"]+States["sun_b_q"]
        lim_a_w=States["motor_a_w"]-States["sun_a_w"]
        lim_b_w=States["motor_b_w"]+States["sun_b_w"]
        kp = 0
        kd = 0
        control_a = Voltage+kp*(-lim_a_q+lim_b_q)/2 + kd*(-lim_a_w+lim_b_w)
        control_b = Voltage+kp*(+lim_a_q-lim_b_q)/2 + kd*(+lim_a_w-lim_b_w)
        control_a = self.clamp(control_a[0],-12,12)
        control_b = self.clamp(control_b[0],-12,12)

        output.SetFromVector([control_b, control_a])
        # print("LIM A: %0.4f, LIM B: %0.4f, DIFF: %0.4f, CONTR A: %0.4f, CONTR B: %0.4f" %(lim_a_q, lim_b_q, lim_a_q-lim_b_q, control_a, control_b))

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
    # parser.AddModelsFromString(cylinder_sdf, "sdf")
    parser.AddModelsFromString(stair_sdf, "sdf")
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

    # onshape = parser.AddModels(url=myrobot_url)
    onshape = Parser(plant, scene_graph).AddModelsFromUrl(
    myrobot_url)[0]

    # Weld the table to the world so that it's fixed during the simulation.
    table_frame = plant.GetFrameByName("table_top_center")
    plant.WeldFrames(plant.world_frame(), table_frame)
    box_frame = plant.GetFrameByName("box_center")
    plant.WeldFrames(plant.world_frame(), box_frame)

    # Finalize the plant after loading the scene.
    plant.Finalize()
    # We use the default context to calculate the transformation of the table
    # in world frame but this is NOT the context the Diagram consumes.
    plant_context = plant.CreateDefaultContext()

    # Set the initial pose for the free bodies, i.e., the custom cylinder,
    # the cracker box, and the sugar box.
    # cylinder = plant.GetBodyByName("cylinder_link")
    X_WorldTable = table_frame.CalcPoseInWorld(plant_context)
    # X_TableCylinder = RigidTransform(
    #     RollPitchYaw(np.asarray([90, 0, 0]) * np.pi / 180), p=[0,0,0.5])
    # X_WorldCylinder = X_WorldTable.multiply(X_TableCylinder)
    # plant.SetDefaultFreeBodyPose(cylinder, X_WorldCylinder)

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
    X_Tablebody = RigidTransform(p=[0,0.75,0.2])
    X_Worldbody = X_WorldTable.multiply(X_Tablebody)
    plant.SetDefaultFreeBodyPose(wheelbody, X_Worldbody)
    # plant.SetDefaultFreeBodyPose(box_frame, X_Worldbody)

    meshcat.AddSlider('V', min=-12, max=12, step=.01, value=0.0)

    motor_system = builder.AddSystem(DC_motor(Get_motor_input_positions(plant, onshape), Get_motor_output_positions(plant, onshape)))
    motor_context = motor_system.CreateDefaultContext()
    
    control_system = builder.AddSystem(ControlSystem(plant.GetStateNames(onshape)))   

    torque_system = builder.AddSystem(MeshcatSliders(meshcat,['V']))
    builder.Connect(control_system.GetOutputPort("Voltage"), motor_system.GetInputPort("Voltage"))
    builder.Connect(plant.get_state_output_port(onshape), motor_system.GetInputPort("ModelState"))
    builder.Connect(motor_system.GetOutputPort("torque"), plant.get_actuation_input_port(onshape))   
    
    builder.Connect(plant.get_state_output_port(onshape), control_system.GetInputPort("ModelState"))
    builder.Connect(torque_system.get_output_port(), control_system.GetInputPort("Input"))


    # Add visualizer to visualize the geometries.
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat,
        MeshcatVisualizerParams(role=Role.kPerception, prefix="visual"))

    diagram = builder.Build()

    # plant.GetInputPort("wheel2_speed").FixValue(plant_context, [0])

    return diagram, visualizer

def Get_motor_input_positions(plant, object):
    names=plant.GetStateNames(object)
    input_projection = [0 for i in range(len(names))]
    index=1
    for i, name in enumerate(names):
        if re.search(r"motor_._w", name):
            input_projection[i] = index
            index += 1
        else: input_projection[i] = 0
    return input_projection

def Get_motor_output_positions(plant, object):
    names=plant.GetActuatorNames(object)
    output_projection = [0 for i in range(len(names))]
    index=1
    for i, name in enumerate(names):
        if re.search(r"motor_.", name):
            output_projection[i] = index
            index += 1
        else: output_projection[i] = 0
    return output_projection

def initialize_simulation(diagram):
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.)
    return simulator

def run_simulation(sim_time_step):
    diagram, visualizer = create_scene(sim_time_step)
    simulator = initialize_simulation(diagram)
    meshcat.AddButton('Stop Simulation')
    visualizer.StartRecording()
    while meshcat.GetButtonClicks('Stop Simulation') < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)
    visualizer.PublishRecording()

# Run the simulation with a small time step. Try gradually increasing it!
run_simulation(sim_time_step=0.0001)
