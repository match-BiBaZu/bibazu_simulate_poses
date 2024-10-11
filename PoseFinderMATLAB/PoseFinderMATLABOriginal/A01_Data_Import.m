%---------------
%A01_Data_Import
%---------------

%% Teil_1: .txt-Import Location

file_name = 'Teil_'+string(bauteil_code)+'_simulated_data_export_matlab_location.txt';
fileID = fopen(file_name,'rt');
formatSpec = '%f';
size_mat = [3 Inf];

mat_location = fscanf(fileID,formatSpec,size_mat);
mat_location = mat_location';

fclose(fileID);

clear fileID file_name 
%% Teil 2: .txt-Import Rotation

file_name = 'Teil_'+string(bauteil_code)+'_simulated_data_export_matlab_rotation.txt';
fileID = fopen(file_name,'rt');

mat_rotation_blend = fscanf(fileID,formatSpec,size_mat);
mat_rotation_blend = mat_rotation_blend';
mat_rotation_blend = mat_rotation_blend*pi/180;
fclose(fileID);

clear fileID file_name size_mat
%% Teil 3: .txt-Import Quaternion

file_name = 'Teil_'+string(bauteil_code)+'_simulated_data_export_matlab_quaternion.txt';
fileID = fopen(file_name,'rt');
size_mat = [4 Inf];

mat_quaternion_blend = fscanf(fileID,formatSpec,size_mat);
mat_quaternion_blend = mat_quaternion_blend';
fclose(fileID);

clear fileID file_name size_mat formatSpec