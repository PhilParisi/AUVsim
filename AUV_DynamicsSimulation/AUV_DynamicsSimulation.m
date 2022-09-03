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
    % uvw, pqr (velocities AKA twist)%% AUV Dynamics

%%
clc, clearvars, tic, close all


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% INITIALIZE VARIABLES & AUV
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

AUV = init_AUV();               % initialize AUV w/ (pose, mprops, dpts)
Data = AUV;                     % initialize Data


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% SIMULATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = 0.1;                        % simulation time step

for t = 0:dt:200
       
    %%%% DYNAMICS UPATE         (calcs accel, updates velocity)
    AUV = dynamics_update(AUV,dt, 0, 0);
    
    %%%% KINEMATICS UPDATE V2   (calcs velocity, updates position)
    AUV = kinematics_update(AUV,dt);         
    
    %%%% LAT/LONG UPDATE        (uses position + lat/long to update lat/long)
    AUV = update_latlong_v2(AUV);
    
    %%%% LOG DATA
    % Log EARTH Frame Data
    Data.t = [Data.t, AUV.t(:,end)];
    Data.xyz = [Data.xyz, AUV.xyz(:,end)];
    Data.rpy = [Data.rpy, AUV.rpy(:,end)];
    Data.dot_xyz = [Data.dot_xyz, AUV.dot_xyz(:,end)];
    Data.dot_rpy = [Data.dot_rpy, AUV.dot_rpy(:,end)];
    % Log VEHICLE BODY Frame Data
    Data.uvw = [Data.uvw, AUV.uvw(:,end)];
    Data.pqr = [Data.pqr, AUV.pqr(:,end)];
    % Log LAT/LONG
    Data.latlong = [Data.latlong, AUV.latlong(:,end)];
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PLOT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
plot3(Data.xyz(1,1:30),Data.xyz(2,1:30),Data.xyz(3,1:30),'.k')
set(gca,'ZDir','Rev')
grid on
%%
% Earth Frame X Y Z 
figure(1)
plot3(Data.xyz(1,:),Data.xyz(2,:),Data.xyz(3,:),'.k');
hold on
plot3(Data.xyz(1,1), Data.xyz(2,1), Data.xyz(3,1),...
    'go','MarkerSize',10,'MarkerFaceColor','g')
hold on
plot3(Data.xyz(1,end), Data.xyz(2,end), Data.xyz(3,end),...
    'ro','MarkerSize',10,'MarkerFaceColor','r')
grid on
title('Position X, Y, Z (Earth Frame)')
set(gca,'ZDir','Rev')
xlabel('X_{North} [m]')
ylabel('Y_{East} [m]')
zlabel('Z_{Depth} [m]')
legend('Path','Start','End')

% Latitude Longitude Depth
figure(2)
plot3(Data.latlong(1,:),Data.latlong(2,:),Data.xyz(3,:),'.b'), hold on
plot3(Data.latlong(1,1), Data.latlong(2,1), Data.xyz(3,1),...
    'go','MarkerSize',10,'MarkerFaceColor','g'), hold on
plot3(Data.latlong(1,end), Data.latlong(2,end), Data.xyz(3,end),...
    'ro','MarkerSize',10,'MarkerFaceColor','r')
title('Latitutde, Longitude, and Depth')
set(gca,'ZDir','Rev')
xlabel('Latitude [deg]')
ylabel('Longitude [deg]')
zlabel('Depth [m]')
legend('Path','Start','End')
grid on

% Pitch Change over Time
figure(3)
plot(Data.t, Data.rpy(2,:))
title('Pitch vs. Time')
xlabel('Time [secs]')
ylabel('Pitch [rads]')
grid on

% Heading (Earth) over Time
figure(4)
plot(Data.t,Data.rpy(3,:))
title('Heading vs. Time')
xlabel('Time [secs]')
ylabel('Heading [rads]')
grid on

toc
disp('...successful!...')