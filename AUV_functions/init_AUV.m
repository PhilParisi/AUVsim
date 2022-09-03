function [AUV] = init_AUV()
% Initialize an AUV with initial conditions

 % TIME
 AUV.t =          0;                     % time

 % EARTH FRAME
 AUV.xyz =        [0;0;0];               % position (m)
 AUV.dot_xyz =    [0;0;0];               % velocity (m/s)
 AUV.rpy =        [0;0;0];               % angle (rad)
 AUV.dot_rpy =    [0;0;0];               % angular velocity (rad/s)
 
 % BODY FRAME
 AUV.uvw =        [0;0;0];               % linear velocity (m/s)
 AUV.dot_uvw =    [0;0;0];               % linear accel (m/s2)
 AUV.pqr =        [0;0;0];               % angular velocity (rad/s)
 AUV.dot_pqr =    [0;0;0];               % angular accel (rad/s2)
 
 % UNIVERSAL CONSTANTS
 AUV.grav = 9.81;   %[m/s2]
 
 % MASS AND DIMENSIONS
 AUV.m = 50;   %mass [m]
 AUV.L = 1.5;  %length [m]
 AUV.D = 0.25; %diameter [m] (vehicle is cylinder)
<<<<<<< HEAD:AUV_DynamicsSimulation/init_AUV.m
 AUV.W = AUV.m*g;  %weight [N]
=======
 AUV.Wt = AUV.m*AUV.grav;  %weight [N]
>>>>>>> main:AUV_functions/init_AUV.m
 
 % BOUYANCY
 AUV.rho = 1000; %density [kg/m3] seawater
 %volume can be calculated using other params or inputted directly
<<<<<<< HEAD:AUV_DynamicsSimulation/init_AUV.m
 AUV.B = 50.1; %kg 'buoyant mass', Buoyant Force = AUV.b*gravity, equiv to rho_h20*Vol_submerged --> wt of h20
 AUV.Bf = AUV.B*g; %buoyant force [N]
=======
 AUV.b = 50.1; %kg 'buoyant mass', Buoyant Force = AUV.b*gravity, equiv to rho_h20*Vol_submerged --> wt of h20
 AUV.Bf = AUV.b*AUV.grav; %buoyant force [N]
>>>>>>> main:AUV_functions/init_AUV.m
 
 % MOMENTS OF INERTIA [units?]
 AUV.Ixx = 1/2*AUV.m*(AUV.D/2)^2;       AUV.Ixy = 0;                                                AUV.Ixz = 0;
 AUV.Ixy = 0;                           AUV.Iyy = (1/4*AUV.m*(AUV.D/2)^2) + (1/12*AUV.m*AUV.L^2);   AUV.Iyz = 0;
 AUV.Ixz = 0;                           AUV.Iyz = 0;                                                AUV.Izz = (1/4*AUV.m*(AUV.D/2)^2) + (1/12*AUV.m*AUV.L^2);
 
 % HYDRODYNAMIC DAMPING [units?]
 AUV.Cusq = 0.4;   AUV.Cvsq = 0.5;   AUV.Cwsq = 0.5;
 AUV.Cpsq = 0.1;   AUV.Cqsq = 0.5;   AUV.Crsq = 0.5;
 
 % CENTER OF GRAVITY (dist to body frame origin) [meters]
 AUV.XG = 0;       AUV.YG = 0;       AUV.ZG = 0.02;
 
 % CENTER OF BUOYANCY (dist to body frame origin) [meters]
 AUV.XB = 0;       AUV.YB = 0;       AUV.ZB = 0; % all zeroes means body frame origin = center buoyancy
 
 % Added Mass
 AUV.X_udot = -6;   AUV.Y_vdot = -10;  AUV.Z_wdot = -10;  %[kg]
 AUV.K_pdot = -10;  AUV.M_qdot = -10;  AUV.N_rdot = -10;  %[kg*m2]
 
 % AREA [m2]
 AUV.Au = pi*(AUV.D/2)^2;     AUV.Av = 2*(AUV.D/2)*AUV.L;   AUV.Aw = 2*(AUV.D/2)*AUV.L;
 AUV.Ap = 2*(AUV.D/2)*AUV.L;    AUV.Aq = 2*(AUV.D/2)*AUV.L;   AUV.Ar = 2*(AUV.D/2)*AUV.L;
 
 % DRAG TERMS
 AUV.X_u2 = 1/2*AUV.rho*AUV.Cusq*AUV.Au;     AUV.Y_v2 = 1/2*AUV.rho*AUV.Cvsq*AUV.Av;     AUV.Z_w2 = 1/2*AUV.rho*AUV.Cwsq*AUV.Aw;
 AUV.K_p2 = 1/2*AUV.rho*AUV.Cpsq*AUV.Ap;     AUV.M_q2 = 1/2*AUV.rho*AUV.Cqsq*AUV.Aq;     AUV.N_r2 = 1/2*AUV.rho*AUV.Crsq*AUV.Ar;
 
disp("...AUV created...")

end