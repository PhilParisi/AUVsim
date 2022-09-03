% Autonomous Underwater Vehicle (AUV)
% Dynamics Simulation


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% NOTES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This simulation considers kinematics and dynamics.

% BODY frame is the local vehicle's perspective
% EARTH frame is the global perspective

% Degrees of Freedom:
    % xyz (position)        
    % rpy (orientation)
    % no constraints on movement!
    
% States:
    % xyz, rpy (positions AKA attitude or pose)
    % uvw, pqr (velocities AKA twist)%% AUV Dynamics


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% INITIALIZE VARIABLES & AUV
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc, clearvars, tic, close all

addpath AUV_functions           % source functions
AUV = init_AUV();               % initialize AUV
Data = AUV;                     % initialize Data

% Change AUV Variables


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% SIMULATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt = 0.1;                        % simulation time step
sim_time = 200;                  % total simulation time
disp('...Running simulation...') % alert user of simulation start

for t = 0:dt:sim_time

    %%%% UPDATE TIME
    AUV.t = AUV.t + dt;
       
    %%%% DYNAMICS UPATE         (calcs accel, updates velocity)
    AUV = dynamics_update(AUV,dt);
    
    %%%% KINEMATICS UPDATE      (calcs velocity, updates position)
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
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% OUTPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('...Simulation complete!...')
toc

% other outputs here


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp("...Making plots...")

% Plot 1: X, Y, Z Position
plot_positionXYZ(Data);

% other plots here






disp("...The script has ended...")