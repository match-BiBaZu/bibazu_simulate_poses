from pathlib import Path
import PoseFinder as pf
import DroptestsFaster as dtf
from STLtoOBJConverter import stl_to_obj_converter
import numpy as np

#--------------------------------------------------------------------------
# 1. Set up the paths to the stl files and the simulation data
#--------------------------------------------------------------------------

# Get the current script's directory
script_dir = Path(__file__).parent

# Get the current script's directory and then add the file path to the folders containing the position and orientation data of the stable poses recorded in the simulations
data_path = script_dir / 'Simulation_Data' / 'Bullet_Raw_Data' / 'Logged_Simulations'

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
workpiece_name = 'Teil_5'

#This is the name of the surface used
surface_name = 'Slide_Long'

# This is the number of simulations
simulation_number = 10

#--------------------------------------------------------------------------
# MODIFIABLE SURFACE AND WORKPIECE PARAMETERS:

alpha_array = np.arange(45, 5, -20) # degrees (set this to 90 when using the plane surface so that it is perpendicular to the gravity vector)

beta_array= np.arange(5, 45, 20) # degrees

workpiece_feed_speed_array = np.arange(0, 5, 1) # initial feed of the workpiece before it slides down the surface- mimics a conveyor belt feeder

hitpoint_offset_parallel_array = np.arange(0, 0.03, 0.01) # offset of the force application hitpoint on the workpiece from the geometric center of the workpiece parallel to the sliding axis

nozzle_offset_parallel = 0.5 # offset of the nozzle on one of the slide surfaces parallel to the sliding axis from the input end of the surface

nozzle_offset_perpendicular_array = np.arange(0, 0.06, 0.01) # offset of the nozzle on one of the slide surface perpendicular from the sliding axis

nozzle_impulse_force_array = np.arange(0, 5, 1) # impulse force applied by the nozzle to the workpiece  to reorient it

orientation_array = np.zeros((simulation_number, 6))

angular_velocity_array = np.zeros((simulation_number, 8))

location_array = np.zeros((simulation_number, 6))

#Create an .obj file if it does not already exist for the bullet engine

#if not (workpiece_path / (workpiece_name + '.obj')).exists():
stl_to_obj_converter(str(workpiece_path / (workpiece_name + '.STL')), str(workpiece_path / (workpiece_name + '.obj')),0.001)

#if not (surface_path / (surface_name + '.obj')).exists():
stl_to_obj_converter(str(surface_path / (surface_name + '.STL')), str(surface_path / (surface_name + '.obj')),0.05)

# Define the CSV file name with the workpiece name
csv_file_name = Path(__file__).parent / 'Simulation_Data' / 'Bullet_Raw_Data' / (workpiece_name + '_simulation_outcomes.csv')

# Write the header line to the CSV file
with open(csv_file_name, 'w') as f:
     f.write('alpha,beta,workpiece_feed_speed,hitpoint_offset_parallel,nozzle_offset_perpendicular,nozzle_impulse_force,sliding_distance,Successfully_Re-oriented,Unsuccessfully_Re-oriented,Not_Reoriented,Not_Settled,Fell_Off_Slide\n')

# Loop through the modifiable parameters and generate simulation data for each combination of parameters
for alpha in alpha_array:
    for beta in beta_array:
        for workpiece_feed_speed in workpiece_feed_speed_array:
            for hitpoint_offset_parallel in hitpoint_offset_parallel_array:
                for nozzle_offset_perpendicular in nozzle_offset_perpendicular_array:
                    for nozzle_impulse_force in nozzle_impulse_force_array:

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
                            Alpha = alpha,
                            Beta = beta,
                            workpiece_feed_speed = workpiece_feed_speed,
                            hitpoint_offset_parallel = hitpoint_offset_parallel,
                            nozzle_offset_parallel = nozzle_offset_parallel,
                            nozzle_offset_perpendicular = nozzle_offset_perpendicular,
                            nozzle_impulse_force = nozzle_impulse_force,
                            mode = 0 # 0 for slowed down testing with GUI and 1 for running headless 
                        )

                        # Generate the simulation data and write to csv files to store simulation data
                        drop_tests_simulator.drop_tests()

                        orientation_array = drop_tests_simulator.get_orientation_array()
                        angular_velocity_array = drop_tests_simulator.get_angular_velocity_array()
                        location_array = drop_tests_simulator.get_location_array()

                        #--------------------------------------------------------------------------
                        # 3. Create a PoseFinder instance to process the simulation data
                        #--------------------------------------------------------------------------

                        # Create a pose finder instance to process the simulation data
                        pose_finder = pf.PoseFinder()

                        # Define the workpiece name and filepath locations for the pose finder instance
                        pose_finder.config(
                            workpiece_name=workpiece_name,
                            data_path=data_path,
                            log_path= log_path,
                            workpiece_path=workpiece_path,
                            simulation_number=simulation_number,
                            mode = 3 # 0 is for Torge, 1 is for Quaternions at end only, 2 is for comparison at end and pre impulse, 3 is for outputting simulation outcomes
                            )

                        # Import csv data from simulations
                        # pose_finder.import_temp_csv()

                        pose_finder.set_angular_velocity_array(angular_velocity_array)
                        pose_finder.set_orientation_array(orientation_array)
                        pose_finder.set_location_array(location_array)

                        # Master function to find and plot the poses (if desired)
                        pose_finder.find_poses_main()

                        # Export the raw data to a log file
                        #log_file_name_base = (
                        #    'alpha' + str(alpha) + 
                        #    '_beta' + str(beta) + 
                        #    '_feed_speed' + str(workpiece_feed_speed) + 
                        #    '_nozzle_offset' + str(nozzle_offset_perpendicular) + 
                        #    '_nozzle_force' + str(nozzle_impulse_force)
                        #)

                        #pose_finder.export_raw_data_csv(log_file_name_base)

                        simulation_outcomes = pose_finder.get_simulation_outcome_frequency()
                        sliding_distance = pose_finder.get_sliding_distance_average()

                        # Append the input parameters and output parameters to a csv file
                        with open(csv_file_name, 'a') as f:
                            f.write(
                                f"{alpha},"
                                f"{beta},"
                                f"{workpiece_feed_speed},"
                                f"{hitpoint_offset_parallel},"
                                f"{nozzle_offset_perpendicular},"
                                f"{nozzle_impulse_force},"
                                f"{sliding_distance},"
                                f"{simulation_outcomes[0]},"
                                f"{simulation_outcomes[1]},"
                                f"{simulation_outcomes[2]},"
                                f"{simulation_outcomes[3]},"
                                f"{simulation_outcomes[4]}\n"
                            )
#--------------------------------------------------------------------------

