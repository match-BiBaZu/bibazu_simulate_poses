from pathlib import Path
import PoseFinder as pf

# Get the current script's directory
script_dir = Path(__file__).parent

# This is the name of the workpieces and of their models
workpiece_name = 'Teil_1'

# Get the current script's directory and then add the file path to the folders containing the position and orientation data of the stable poses recorded in the simulations
data_path = Path(__file__).parent / 'MHI_Data'

# Get the current script's directory and then add the file path to the folders containing the workpiece stls
workpiece_path =  Path(__file__).parent / 'Workpieces'

# Create a pose finder instance to process the simulation data
poseFindObject = pf.PoseFinder()

# Define the workpiece name and filepath locations for the pose finder instance
poseFindObject.config(workpiece_name = workpiece_name,data_path = data_path,workpiece_path = workpiece_path)
 
# Import csv data from simulations
poseFindObject.import_orientation()

# Process the data to find stable poses
poseFindObject.find_poses()

# Plot the resulting stable poses
poseFindObject.plot_poses()

#self.config(workpiece_name="Alice", relative_path=30, city="New York")