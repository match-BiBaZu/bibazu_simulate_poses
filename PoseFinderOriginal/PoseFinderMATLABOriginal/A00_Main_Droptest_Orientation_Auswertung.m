%-----------------------------------------------------
% A00 Main_Algorithmus_Droptest_Orientation_Auswertung
%-----------------------------------------------------

%% Teil 1: Einstellungen & STL-Import

%Einstellungen:
bauteil_code = 1;

% STL Import:
switch bauteil_code
    case 1
        fv = stlread('Teil1.stl');
    case 2
        fv = stlread('Teil2.stl');
    case 3
        fv = stlread('Teil3.stl');
    case 4
        fv = stlread('Teil4.stl');
    case 5
        fv = stlread('Teil5.stl');
    case 6
        fv = stlread('Teil6.stl');
end
%% Teil 2: Programm-Ablauf

A01_Data_Import;
A02_Data_Processing;
A03_Data_Plot;