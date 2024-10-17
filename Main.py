from pathlib import Path
import PoseFinder as pf
import DroptestsFaster as dtf
from STLtoOBJConverter import stl_to_obj_converter

#--------------------------------------------------------------------------
# 1. Set up the paths to the stl files and the simulation data
#--------------------------------------------------------------------------

# Get the current script's directory
script_dir = Path(__file__).parent

# Get the current script's directory and then add the file path to the folders containing the position and orientation data of the stable poses recorded in the simulations
data_path = Path(__file__).parent / 'SimulationData'/'Dashas_Testing_Bullet'#'Dashas_Testing_Blender'

# Get the current script's directory and then add the file path to the folders containing the workpiece stls
workpiece_path =  Path(__file__).parent / 'Workpieces'

# Get the current script's directory and then add the file path to the folders containing the workpiece stls
surface_path =  Path(__file__).parent / 'Surfaces'

#--------------------------------------------------------------------------
# 2. Create a DroptestsFaster instance to generate the simulation data
#--------------------------------------------------------------------------

# This is the name of the workpieces and of their models
workpiece_name = 'Teil_5'

#This is the name of the surface used
surface_name = 'Slide_Long'

# This is the number of simulations
simulation_number = 10

# MODIFIABLE SURFACE PARAMETERS:

Alpha = 0 # degrees (set this to 90 when using the plane surface so that it is perpendicular to the gravity vector)

Beta = 45 # degrees

#Create an .obj file if it does not already exist for the bullet engine

#if not (workpiece_path / (workpiece_name + '.obj')).exists():
stl_to_obj_converter(str(workpiece_path / (workpiece_name + '.STL')), str(workpiece_path / (workpiece_name + '.obj')),0.01)

#if not (surface_path / (surface_name + '.obj')).exists():
stl_to_obj_converter(str(surface_path / (surface_name + '.STL')), str(surface_path / (surface_name + '.obj')),0.1)

# Create an instance to generate simulation data
drop_tests_simulator = dtf.DroptestsFaster()

# Define the workpiece name and filepath locations for the drop tests simulator instance
drop_tests_simulator.config(
    workpiece_name=workpiece_name,
    data_path=data_path,
    workpiece_path=workpiece_path,
    surface_path=surface_path,
    surface_name=surface_name,
    simulation_number=simulation_number,
    Alpha = Alpha,
    Beta = Beta,
)

# Generate the simulation data and write to csv files to store simulation data
drop_tests_simulator.drop_tests()

#--------------------------------------------------------------------------
# 3. Create a PoseFinder instance to process the simulation data
#--------------------------------------------------------------------------

# Create a pose finder instance to process the simulation data
pose_finder = pf.PoseFinder()

# Define the workpiece name and filepath locations for the pose finder instance
pose_finder.config(
    workpiece_name=workpiece_name,
    data_path=data_path,
    workpiece_path=workpiece_path,
    simulation_number=simulation_number
)

# Import csv data from simulations
pose_finder.import_orientation_csv()

# Process the data to find stable poses
pose_finder.find_poses() 

# Plot the resulting stable poses
pose_finder.plot_poses_quat() # only got the quaternion outputs to output properly for pose finder so this only uses quaternions