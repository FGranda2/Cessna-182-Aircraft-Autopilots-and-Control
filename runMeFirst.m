%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LOAD REQUIRED VARIABLES FOR SIMULATION
% STATE = [u,alpha,q,theta,h,v,p,r,phi,psi]'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear

% Load data from design
load Auto_long2.mat
load Auto_latr2.mat

% Organize matrices
% First linear dynamics
A_final = [A1_long,zeros(5);zeros(5),A1_latr];
B_final = [B1_long,zeros(5,2);zeros(5,2),B1_latr];
C_final = [C1_long,zeros(5);zeros(5),C1_latr];
D_final = zeros(10,4);

% Then LQR Gains
K_deltas = [K_deltas_long,zeros(2);zeros(2),K_deltas_latr];
K_eps = [K_eps_long,zeros(2,2);zeros(2,2),K_eps_latr];
K_lqr = [K_lqr_long,zeros(2,5);zeros(2,5),K_lqr_latr];

% For coordinated turn | Specific Radius
Yaw_des =90;  % deg
R = 2000;    % m
Ue = 67;  % m/s
[Des_roll,Des_rate] = Rtool(R,Ue);

% For maneuvers (Takeoff at 0 secs. Steady-Flight at 500 secs)
% Start recording data of maneuver at 500 secs
Start_time = 500; % sec

% Start ascend at 1000 sec
Step_timeh = 1000; % sec
Initial_h = 1000; % m
Second_h = 2000; % m

% Start turn at 1500 sec
Start_turn = 1500; % sec
Turn_time = deg2rad(Yaw_des)/Des_rate;

