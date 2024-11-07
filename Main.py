from pathlib import Path
import PoseFinder as pf
import DroptestsFaster as dtf
#from STLtoOBJConverter import stl_to_obj_converter
import numpy as np
from multiprocessing import Pool
import itertools
from multiprocessing import Pool
import itertools

#--------------------------------------------------------------------------
# 1. Set up the paths to the stl files and the simulation data
#--------------------------------------------------------------------------

# Get the current script's directory
script_dir = Path(__file__).parent

# Get the current script's directory and then add the file path to the folders containing the position and orientation data of the stable poses recorded in the simulations
data_path = script_dir / 'Simulation_Data' / 'Bullet_Raw_Data' / 'Temporary'

# This is the current file path of the logged data stored relative to the script
log_path = script_dir / 'Simulation_Data' / 'Bullet_Raw_Data' / 'Logged_Simulations'

# Get the current script's directory and then add the file path to the folders containing the workpiece stls
workpiece_path =  script_dir / 'Workpieces'

# Get the current script's directory and then add the file path to the folders containing the workpiece stls
surface_path =  script_dir / 'Surfaces'

#--------------------------------------------------------------------------
# 2. Create a DroptestsFaster instance to generate the simulation data
#--------------------------------------------------------------------------

# This is the name of the workpieces and of their models
workpiece_name = 'Teil_2'

# This is the name of the surface used
# This is the name of the surface used
surface_name = 'Slide_Long'

# This is the number of simulations
simulation_number = 10

#--------------------------------------------------------------------------
# MODIFIABLE SURFACE AND WORKPIECE PARAMETERS:

alpha_array = np.arange(5, 45, 5) # degrees (set this to 90 when using the plane surface so that it is perpendicular to the gravity vector)

beta_array = np.arange(5, 45, 5) # degrees
beta_array = np.arange(5, 45, 5) # degrees

workpiece_feed_speed_array = np.arange(0, 5, 1) # initial feed of the workpiece before it slides down the surface- mimics a conveyor belt feeder

hitpoint_offset_parallel_array = np.arange(0, 0.03, 0.005) # offset of the force application hitpoint on the workpiece from the geometric center of the workpiece parallel to the sliding axis

nozzle_offset_parallel = 0.5 # offset of the nozzle on one of the slide surfaces parallel to the sliding axis from the input end of the surface

nozzle_offset_perpendicular_array = np.arange(0, 0.06, 0.01) # offset of the nozzle on one of the slide surface perpendicular from the sliding axis

nozzle_impulse_force_array = np.arange(0, 5, 1) # impulse force applied by the nozzle to the workpiece to reorient it
nozzle_impulse_force_array = np.arange(0, 5, 1) # impulse force applied by the nozzle to the workpiece to reorient it

# Define the CSV file name with the workpiece name
csv_file_name = script_dir / 'Simulation_Data' / 'Bullet_Raw_Data' / (workpiece_name + '_simulation_outcomes.csv')
csv_file_name = script_dir / 'Simulation_Data' / 'Bullet_Raw_Data' / (workpiece_name + '_simulation_outcomes.csv')

# Write the header line to the CSV file
with open(csv_file_name, 'w') as f:
    f.write('alpha,beta,workpiece_feed_speed,hitpoint_offset_parallel,nozzle_offset_perpendicular,nozzle_impulse_force,sliding_distance,Successfully_Re-oriented,Unsuccessfully_Re-oriented,Not_Reoriented,Not_Settled,Fell_Off_Slide\n')
    f.write('alpha,beta,workpiece_feed_speed,hitpoint_offset_parallel,nozzle_offset_perpendicular,nozzle_impulse_force,sliding_distance,Successfully_Re-oriented,Unsuccessfully_Re-oriented,Not_Reoriented,Not_Settled,Fell_Off_Slide\n')

# Combine all parameter arrays for multiprocessing
parameter_combinations = list(itertools.product(
    alpha_array,
    beta_array,
    workpiece_feed_speed_array,
    hitpoint_offset_parallel_array,
    nozzle_offset_perpendicular_array,
    nozzle_impulse_force_array
))

def run_simulation(params):
    alpha, beta, workpiece_feed_speed, hitpoint_offset_parallel, nozzle_offset_perpendicular, nozzle_impulse_force = params
# Combine all parameter arrays for multiprocessing
parameter_combinations = list(itertools.product(
    alpha_array,
    beta_array,
    workpiece_feed_speed_array,
    hitpoint_offset_parallel_array,
    nozzle_offset_perpendicular_array,
    nozzle_impulse_force_array
))

def run_simulation(params):
    alpha, beta, workpiece_feed_speed, hitpoint_offset_parallel, nozzle_offset_perpendicular, nozzle_impulse_force = params

    # Create an instance to generate simulation data
    drop_tests_simulator = dtf.DroptestsFaster()
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
        Alpha=alpha,
        Beta=beta,
        workpiece_feed_speed=workpiece_feed_speed,
        hitpoint_offset_parallel=hitpoint_offset_parallel,
        nozzle_offset_parallel=nozzle_offset_parallel,
        nozzle_offset_perpendicular=nozzle_offset_perpendicular,
        nozzle_impulse_force=nozzle_impulse_force,
        mode=1
    )
    # Define the workpiece name and filepath locations for the drop tests simulator instance
    drop_tests_simulator.config(
        workpiece_name=workpiece_name,
        data_path=data_path,
        workpiece_path=workpiece_path,
        surface_path=surface_path,
        surface_name=surface_name,
        simulation_number=simulation_number,
        Alpha=alpha,
        Beta=beta,
        workpiece_feed_speed=workpiece_feed_speed,
        hitpoint_offset_parallel=hitpoint_offset_parallel,
        nozzle_offset_parallel=nozzle_offset_parallel,
        nozzle_offset_perpendicular=nozzle_offset_perpendicular,
        nozzle_impulse_force=nozzle_impulse_force,
        mode=1
    )

    # Generate the simulation data
    drop_tests_simulator.drop_tests()
    # Generate the simulation data
    drop_tests_simulator.drop_tests()

    # Create a pose finder instance to process the simulation data
    pose_finder = pf.PoseFinder()
    # Create a pose finder instance to process the simulation data
    pose_finder = pf.PoseFinder()

    # Configure the pose finder
    pose_finder.config(
        workpiece_name=workpiece_name,
        data_path=data_path,
        log_path=log_path,
        workpiece_path=workpiece_path,
        simulation_number=simulation_number,
        mode=3
    )
    # Configure the pose finder
    pose_finder.config(
        workpiece_name=workpiece_name,
        data_path=data_path,
        log_path=log_path,
        workpiece_path=workpiece_path,
        simulation_number=simulation_number,
        mode=3
    )

    # Import csv data from simulations
    pose_finder.import_temp_csv()
    # Import csv data from simulations
    pose_finder.import_temp_csv()

    # Find and process poses
    pose_finder.find_poses_main()
    # Find and process poses
    pose_finder.find_poses_main()

    # Export the raw data to a log file
    log_file_name_base = (
        'alpha' + str(alpha) +
        '_beta' + str(beta) +
        '_feed_speed' + str(workpiece_feed_speed) +
        '_nozzle_offset' + str(nozzle_offset_perpendicular) +
        '_nozzle_force' + str(nozzle_impulse_force)
    )
    pose_finder.export_raw_data_csv(log_file_name_base)
    # Export the raw data to a log file
    log_file_name_base = (
        'alpha' + str(alpha) +
        '_beta' + str(beta) +
        '_feed_speed' + str(workpiece_feed_speed) +
        '_nozzle_offset' + str(nozzle_offset_perpendicular) +
        '_nozzle_force' + str(nozzle_impulse_force)
    )
    pose_finder.export_raw_data_csv(log_file_name_base)

    # Get simulation outcomes and sliding distance
    simulation_outcomes = pose_finder.get_simulation_outcome_frequency()
    sliding_distance = pose_finder.get_sliding_distance_average()
    # Get simulation outcomes and sliding distance
    simulation_outcomes = pose_finder.get_simulation_outcome_frequency()
    sliding_distance = pose_finder.get_sliding_distance_average()

    # Append the input parameters and output parameters to the CSV file
    with open(csv_file_name, 'a') as f:
        f.write(
            f"{alpha},{beta},{workpiece_feed_speed},{hitpoint_offset_parallel},{nozzle_offset_perpendicular},{nozzle_impulse_force},{sliding_distance},{simulation_outcomes[0]},{simulation_outcomes[1]},{simulation_outcomes[2]},{simulation_outcomes[3]},{simulation_outcomes[4]}\n"
        )

    # Run simulations in parallel using 32 pools
if __name__ == "__main__":
    with Pool(32) as pool:
        pool.map(run_simulation, parameter_combinations)