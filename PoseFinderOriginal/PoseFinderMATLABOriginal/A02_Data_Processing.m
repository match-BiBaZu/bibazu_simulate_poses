%--------------------
%A02_Data_Processing
%--------------------

% https://de.mathworks.com/matlabcentral/answers/415936-angle-between-2-quaternions
% https://de.mathworks.com/help/robotics/ref/quaternion.rotvec.html?searchHighlight=vector

%% Teil 1: Orientierungserkennung anhand Quaternione
mat_quaternion_blend(:,5) = 0;
n = 1;
eps = 0.3;

for i = 1:length(mat_quaternion_blend)
    
    if mat_quaternion_blend(i,5) == 0
        mat_quaternion_blend(i,5) = n;                      
        
        x = mat_quaternion_blend(i,1:4);                    % KS1 - Reihenfolge (w,x,y,z)
        for j = i+1:length(mat_quaternion_blend)
            y = mat_quaternion_blend(j,1:4);                % KS2 - Reihenfolge (w,x,y,z)
            
            x = x/norm(x);                                  % x-Quaternion normieren
            y = y/norm(y);                                  % y-Qauternion normieren
            
            z = quatmultiply(quatconj(x),y);                % z-Quaternion
            
            r_x_KS1 = z(2)/sin(acos(z(1)));                 % RotationAxis.x.KS1
            r_y_KS1 = z(3)/sin(acos(z(1)));                 % RotationAxis.y.KS1
            r_z_KS1 = z(4)/sin(acos(z(1)));                 % RotationAxis.z.KS1
            
            vector_r_1 = [r_x_KS1 r_y_KS1 r_z_KS1]';        % Vektor r (Rotationsachse) in KS1
            
            rotm_x = quat2rotm(x);                          % Rotationsmatrix rotm_x von KS0 in KS1
            rotm_y = quat2rotm(y);                          % Rotationsmatrix rotm_x von KS0 in KS2
            rotm_z = quat2rotm(z);                          % Rotationsmatrix rotm_x von KS1 in KS2
            
            vector_r_0 = rotm_x*vector_r_1;
            
            r_x_KS0 = vector_r_0(1);                        % RotationAxis.x.KS0
            r_y_KS0 = vector_r_0(2);                        % RotationAxis.x.KS0
            r_z_KS0 = vector_r_0(3);                        % RotationAxis.x.KS0
            
            if mat_quaternion_blend(j,5) == 0
                if ((r_x_KS0 >= 0-eps)  && (r_x_KS0 <= 0+eps)) && ((r_y_KS0 >= 0-eps)  && (r_y_KS0 <= 0+eps))
                    mat_quaternion_blend(j,5) = n;
                end
            end
        end
        n=n+1;
    end
end

%% Teil 2: Sort and Histogram
% Umwandlung Quaternions-Matrix zu EULER-Winkeln

mat_rotation_blend(:,4) = mat_quaternion_blend(:,5);

[mat_quaternion_blend,index] = sortrows(mat_quaternion_blend,5);
mat_rotation_blend = sortrows(mat_rotation_blend,4);

anz_posen = max(mat_quaternion_blend(:,5));
 
h = histogram(mat_rotation_blend(:,4));
N = histcounts(mat_rotation_blend(:,4));
