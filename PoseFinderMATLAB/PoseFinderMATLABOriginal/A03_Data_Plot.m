%-------------
%A03_Data_Plot
%-------------
%% Teil_1: STL-Abbildung

% Points und Conectivity Matrizen auslesen
points = fv.Points;
cList = fv.ConnectivityList;

figure('WindowState', 'maximize')

t = tiledlayout(ceil(anz_posen/2),4); % ANPASSEN (Wie viele verschiedene Posen gibt es??)

m = 1;
a = 1;

for i = 1:length(mat_rotation_blend)
    if mat_rotation_blend(i,4) == m
        
        ax = mat_rotation_blend(i,1);
        ay = mat_rotation_blend(i,2);
        az = mat_rotation_blend(i,3);
        
        % Rotationsmatrix aus Eulerwinkeln bestimmen
        Rx = [1 0 0;0 cos(ax) -sin(ax); 0 sin(ax) cos(ax)]; % rotation um X-Achse mit ax
        Ry = [cos(ay) 0 sin(ay); 0 1 0; -sin(ay) 0 cos(ay)]; % rotation um Y-Achse mit ay
        Rz = [cos(az) -sin(az) 0; sin(az) cos(az) 0; 0 0 1]; % rotation um Z-Achse mit az
        
        % Point Matrix mit Rotationsmatrix Multipilieren
        
        rotm = Rx'*Ry'*Rz';
        
        pointsR = points*rotm;
        m_str = string(m);
        
        nexttile(a)
        trimesh(cList, pointsR(:,1), pointsR(:,2), pointsR(:,3),'EdgeColor','k','FaceColor',[0.6 0.6 0.6]); % rotated model
        axis vis3d equal off;
        
        title('Pose '+ m_str);
        m = m+1;
        
        if rem(a,2)
            a = a+1;
        else
            a = a+3;
        end
        
    end
end

%% Teil 2: Pie Chart

nexttile([ceil((anz_posen/2)) 2])

clear m m_str;

labels = strings([anz_posen,1]);
labels_pie = strings([anz_posen,1]);

m = 1;

for m = 1:anz_posen

    m_str = string(m);
    abs_str = string(N(1,m));
    percentage_sring = string(floor(N(1,m)/(length(mat_quaternion_blend)/100)*100)/100);
    label_legend = append('Pose ', m_str,' (n = ', abs_str,')');
    label_pie = append('Pose ', m_str,' (', percentage_sring,' %)');
    labels(m,1) = {label_legend};
    labels_pie(m,1) = {label_pie};
    m = m+1;

end

[val, idx] = max(N);

explode = zeros(1,anz_posen);
explode(1,idx) = 1;

pie(N,explode,labels_pie);

legend(labels,'Location','northeastoutside');
legend('boxoff');

str_bauteil_code = string(bauteil_code);
tit = append('Natural Resting Position - Workpiece No. ',str_bauteil_code);
tn = title(t,tit);
tn.FontSize = 16;
tn.FontWeight = 'bold';