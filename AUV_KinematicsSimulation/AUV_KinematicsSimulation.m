% Autonomous Underwater Vehicle (AUV)
% Kinematics Simulation
% July 2022

%%%%%%%%%%%%%%% NOTES
% This simulation only considers basic kinematics.

% BODY frame is the local vehicle's perspective
% EARTH frame is the global perspective

% Degrees of Freedom:
    % xyz (position)        
    % rpy (orientation)
    % no constraints on movement!
    
% States:
    % xyz, rpy (positions AKA attitude or pose)
    % uvw, pqr (velocities AKA twist)
     
%%

%%%%%%%%%%%%%%% INITIAL CONDITIONS
clc, clearvars, close all

% EARTH FRAME position + velocity
AUV.xyz = [0;0;0]; %m
AUV.dot_xyz = [0;0;0]; %m/s

% EARTH FRAME starting Lat/Long
AUV.latlong = [30; 50];

% EARTH FRAME angle + angular velocity
AUV.rpy = [0; 0; 0]; %degrees
AUV.dot_rpy = [0;0;0]; %deg/s

% BODY FRAME velocity
AUV.uvw = [1;0;0]; %m/s

% BODY FRAME angular velocity
AUV.pqr = [0;0;0]; %deg/s

% Define data storage struct
Data = AUV; % store current AUV into Data log
Data.t = 0; % set initial time to 0


%%%%%%%%%%%%%%% SIMULATION
dt = 2; % time step

for t = 1:dt:200
    
    %%%% KINEMATICS UPDATE
    AUV = kinematics_update(AUV,dt);         
     
    
    %%%% LOG DATA
    
    % Log EARTH Frame Data
    Data.t = [Data.t, t];
    Data.xyz = [Data.xyz, AUV.xyz];
    Data.rpy = [Data.rpy, AUV.rpy];
    Data.dot_xyz = [Data.dot_xyz, AUV.dot_xyz];
    Data.dot_rpy = [Data.dot_rpy, AUV.dot_rpy];
    
    % Log VEHICLE BODY Frame Data
    Data.uvw = [Data.uvw, AUV.uvw];
    Data.pqr = [Data.pqr, AUV.pqr];
    
    % Calculate LAT/LONG
    AUV = latlong_update(AUV,Data.xyz(:,end-1:end),Data.latlong(:,end));
    
    % Log Lat/Long
    Data.latlong = [Data.latlong, AUV.latlong];
    
end


%%%%%%%%%%%%%%% PLOTTING
% Plot 1: X, Y, Z
figure(1)
plot3(Data.xyz(1,:),Data.xyz(2,:),Data.xyz(3,:),'.k'), hold on
L1 = plot3(Data.xyz(1,1), Data.xyz(2,1), Data.xyz(3,1),'ko','MarkerSize',10,...
    'MarkerFaceColor','g');
L2 = plot3(Data.xyz(1,end), Data.xyz(2,end), Data.xyz(3,end),'ko','MarkerSize',10,...
    'MarkerFaceColor','r');
grid on
title('Position X, Y, Z (Earth Frame)')
set(gca,'ZDir','Rev')
xlabel('X_{North} [m]')
ylabel('Y_{East} [m]')
zlabel('Z_{Depth} [m]')
legend('Path','Start','End')
hold off


% Plot 2: Lat, Long, Depth
figure(2)
plot3(Data.latlong(1,:),Data.latlong(2,:),Data.xyz(3,:),'.b'), hold on
plot3(Data.latlong(1,1), Data.latlong(2,1), Data.xyz(3,1),'ko','MarkerSize',10,...
    'MarkerFaceColor','g')
plot3(Data.latlong(1,end), Data.latlong(2,end), Data.xyz(3,end),'ko','MarkerSize',10,...
    'MarkerFaceColor','r')
grid on
title('Lat and Long vs Depth')
set(gca,'ZDir','Rev')
xlabel('Latitude [deg]')
ylabel('Longitude [deg]')
zlabel('Depth [m]')
legend('Path','Start','End')
hold off
