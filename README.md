# 2023-FYP

Using this repository:

This repository contains the full project folder for my final year project. If you are continuing this project, I recommend cloning this repository, or downloading the MATLAB/ and vscode/ folders.

Using the maths model:

The maths model was developed in MATLAB and can be found in MATLAB/v2/. The Calculations.mlx contains the code I used to produce the plots in the report. The section loads the stored sets of equations. These equations are produced by the scripts SetCoreEquations.m and SetDesignParameters.m. 
To model devices other than the one I used in this project, edit SetDesignParameters.m with your device's parameters and run it.

The next sections in Calculations.mlx builds a set of equations from the core equations, the design parametes, the boundary conditions as described in the report, and the position of the device. The equations are fed to mysolver(), and the resulting solution contains T_z, the motor torque required to cause motion. 
The solutions are used to creaste a visualisation of the device, which can be useful to ensure that all the angles lie in the correct quadrants and that the tail is contacting the correct steps. These steps are repeated for each stage of motion.

Next, the model is used to find solutions for a range of positions across a stage of motion. This is how I produced the torque - angle plot in the report. To do this, it first attempts to solve an incomplete set of equations, that does not contain the position. The solver will find solutions to what it can, saving time in future steps. 
Then, looping through all the positions you want to solve for, the positions are added to the remaining equations and solved. This takes some time, despite making use of parallelisation.

Next, the model is used to determine how changing the gear ratios affects the required torque and friction. To do this, it first creates a new set of design parameters that do not define the gear ration, then the the solver loops through different values for the number of teeth on the planet gear. The highet torque and friction required are in Stage 4 and are not at a set angle, so the solver attempts several angles and finds the one that produces the highest torque.
This example can be expanded to explore any design parameter, just make sure to check whether the increasing the size of the tail or LIMs changes whcih steps the LIMS make contact with.

The one major weakness of mysolver() is that it cannot identify whether the equations are overdefined. if you have x == 2 and x == 3, it will just pick the first one, so bear that in mind when using this tool.

Using the converter:

Drake does not support windows, so linux or osx is needed. I set up the conversion pipeline and simulation in ubuntu using the Windows Subsystem for Linux. I recommend installing Visual studio code for WSL for these next steps.

This stage primarily uses onshape-to-robot, which can be found at https://github.com/Rhoban/onshape-to-robot
They have a well documented program, however if you just want to use my scripts, this is what you would need to do:
1. Clone or download vscode/ into your linux installation
2. Create a pythion virtual environment in this folder
3. run pip install -r requirements.txt
4. Get an Onshape account
5. Get an access key and secret key from https://dev-portal.onshape.com/keys
6. Edit convert.sh with your keys (Note that this is not secure, and there are better ways of doing it. Do not publish this file while it has your keys in it. I have deactivated mine.)
7. Now you need a file to convert. onshape-to-robot has a list of requirements for the onshape file, I recommend you ensure whatever device you build conforms their guidelines. My design, https://cad.onshape.com/documents/9f7ac96b4bf1f871a2a6bcc9/, can be used as a guideline for naming conventions and subassembly grouping. My code specifically looks for joints named dof_motor_a and dof_motor_b_inv, and links named Tail, Wheel, and Frame.
8. Once you have a device ready, create a new folder containing a file named config.json, with the contents:
   {
    "documentId": "Your document ID",
    "outputFormat": "sdf"
   }
   "Your document ID" is the first set of hex code in the devices URL. For my design it is 9f7ac96b4bf1f871a2a6bcc9.
9. Then run "convert.sh yourFolder" where yourFolder is the name of the folder you created in step 8.
10. The folder should now populare with a .sdf file and all of the object files.

Using the simulator:

To run the simulation, run simulation.py. This will open a meshcat instance, which can be opened at http://localhost:7000/. The code here is pramarily based on the tutorial https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/notebook/authoring_multibody_simulation-add293478aac40a984845aa3705eefdd?
simulation.py has many private defines that can be edited, such as the .sdf file location, stair sizes, and motor properties.
