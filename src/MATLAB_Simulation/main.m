%% main script
% Distributed formation control of a Swarm of UGVs
% This script runs the simulation based on Consensus Algorithms and
% kinematics and control of differential driven robots

%% Initialize
clear all;
close all;

% No. of vehicles
N = 6;

% specify to virtual leader connected vehicles. Two vecotr with elements
% from 1-N
connections = [1];

% check if valid connections input
if ((max(connections) > N))
    error('Index of VL connected vehicle may not exceed number of vehicles.');
end

disp('Adjacency matrixes 1 and 2 with and without VL connections');
[Adj_VL, Adj] = graph_create(connections, N);

% check if communication network is strongly connected - if not -> new
% graphs
while (any(sum([Adj_VL]) == 0))
    disp('Unconnected vehicle. Creating new random graphs.');
    close all;
    [Adj_VL, Adj] = graph_create(connections, N);
end

% init of reference frame values (x,y,theta)
xi_init = zeros(3,N);

% desired relative position / formation around virtual center
[r_rel_1, r_rel_2] = create_r_relative(N);
r_init = r_rel_2;

% Path tracking parameters
Kp_rho = 9;
Kp_alpha = 15;
Kp_beta = -6;


%% Simulation
sim TB3_Formation_Simulation;
simOut = ans;
disp('Simulation run succesfully...');


%% Results Visualization

% creates new figure in the right display half with 6 subplots
scrsz = get(groot,'ScreenSize');
result = figure('OuterPosition',[scrsz(3)/2 0 scrsz(3)/2 scrsz(4)]);
set(result, 'Name', 'Simulation Results Consensus', 'NumberTitle', 'off');

% splits up simulation timeseries results into the different states
time = simOut.xi_i.time;
x_ref_i = timeseries(simOut.xi_i.data(1,:,:), time, 'Name', 'x-coordinate');
y_ref_i = timeseries(simOut.xi_i.data(2,:,:), time, 'Name', 'y-coordinate');
theta_ref_i = timeseries(simOut.xi_i.data(3,:,:), time, 'Name', 'theta-orientation');

subplot(3,2,1), plot(simOut.xi_ref);
legend('x_{ref}','y_{ref}','theta_{ref}');
title('Plot of the reference state (leader)');
ylabel('pos/angle in meters/rad');

