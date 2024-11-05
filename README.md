
# bibazu_simulate_poses

A Blender-based simulation tool designed to model and visualize workpieces being dropped onto flat surfaces from a chute. This repository contains scripts and assets that enable the creation of dynamic simulations for evaluating how various workpieces behave under controlled drop conditions.

Features:
+ Simulates the dynamic behavior of various workpieces as they drop onto different surfaces.
+ Visualizes and analyzes how workpieces interact with flat and tilted surfaces based on physical properties.
+ Supports customizable configurations, allowing users to modify workpieces and surface parameters.

Installation Instructions:
__1. Install Required Software:__

Blender
+ Download and install Blender from the official website: Blender Download
  + Ensure you have Blender 2.9x or newer installed, as the tool relies on Blender's Python API.

Python Libraries

You'll need the following Python libraries for running the simulations and handling 3D files. Install them via pip:

+ numpy: A fundamental library for numerical computations in Python.
+ numpy-stl: For handling STL (stereolithography) files, a common format for 3D models.
+ mpl_toolkits.mplot3d.art3d: Part of Matplotlib, this library is used for 3D visualization in Python.

You can install all the necessary libraries with the following command:

    pip install numpy numpy-stl matplotlib

__2. Unzip and Organize the Assets:__

The repository includes several zipped folders containing assets needed for the simulation. Follow these steps to ensure they are set up correctly:

+ Surfaces: This folder contains various surface models that the workpieces will drop onto.
+ Workpieces: This folder contains 3D models of the workpieces being simulated.
+ SimulationData: Stores data generated during the simulations, including parameters, results, and configurations.

Important: Ensure that you unzip each of these folders and place the contents directly in the main repository folder. Make sure they are not inside other zipped folders.

__3. Running the Simulation:__

Once everything is installed and unzipped, you're ready to run the simulation. You can either run the simulation script from the Blender Text Editor or execute it via Blender's command-line interface in background mode.
Option 1: Running in Blender's Text Editor

+ Open Blender.
+ Go to the Text Editor (located in the top bar under the scripting tab).
+ Open the main Python script file (e.g., simulate_workpieces.py).
+ Press Run Script to start the simulation.

Option 2: Running via Command Line (Background Mode) __NOT YET WORKING__

+ Open a terminal or command prompt.
+ Navigate to the directory where Blender is installed.
+ Use the following command to run the script in background mode:

      blender -b --python simulate_workpieces.py

__4. Docker:__

sudo docker build -t BiBaZuDocker .
sudo docker run -it pBiBaZuDocker