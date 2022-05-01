%% Imports

% Reset Workspace
clc; clear;
addpath('matlab');
[u, reg, quat] = util().reset();

% Show plots?
show_plots = true;

% Save plots to figures/?
save_plots = false;

J = [3.1019279e+10  2.2047148e+07  2.2994735e+08;
     2.2047148e+07  4.0036354e+10 -1.9779199e+07;
     2.2994735e+08 -1.9779199e+07  3.9828543e+10] * 0.00142233;


%% Disturbance Torques (SRP, GG, etc.)

% Reset Workspace
[u, reg, quat] = util().reset();

params = {};
params.J = J;
params.T = 20 * 60 * 60;
params.M_ax = [0, 1, 0]';
params.M = 1.10e-5;
params.M_time = [];
params.w_0 = [0; 0; 0];
params.q_c = [0; 0; 0; 1];
params.q_time = [];
params.k_p = 5; params.k_d = 1e4;

results = reg.regulate(params);
wheels = reg.decompose(results);
reg.analysis(results, wheels);

plots = [
    reg.plot_errors(results);
    reg.plot_momenta(results);
    reg.plot_wheel_momenta(wheels);
    reg.plot_rotations(results);
];

u.save_plots(save_plots, 'Disturbance', plots);
u.show_plots(show_plots, plots);


%% Detumble (Thruster Misfire, Launch Vehicle, etc.)

% Reset Workspace
[u, reg, quat] = util().reset();

params = {};
params.J = J;
params.T = 20 * 60 * 60;
params.M_ax = [0, 0, 1]';
params.M = 10;
params.M_time = [0, 5];
params.w_0 = [0; 0; 0];
params.q_c = [0; 0; 0; 1];
params.q_time = [1 * 60 * 60, params.T];
params.k_p = 10; params.k_d = 1e5;

results = reg.regulate(params);
wheels = reg.decompose(results);
reg.analysis(results, wheels);

plots = [
    reg.plot_errors(results);
    reg.plot_momenta(results);
    reg.plot_wheel_momenta(wheels);
    reg.plot_rotations(results);
];

u.save_plots(save_plots, 'Detumble', plots);
u.show_plots(show_plots, plots);


%% Belly Flop (180 degree turn)

% Reset Workspace
[u, reg, quat] = util().reset();

params = {};
params.J = J;
params.T = 16 * 60 * 60;
params.M = 0;
params.w_0 = [0; 0; 0];
e_hat = [0, 1, 0]';
e_hat = e_hat / norm(e_hat);
params.q_c = quat.q(e_hat, pi);
params.k_p = 25; params.k_d = 1e5;

results = reg.regulate(params);
wheels = reg.decompose(results);
reg.analysis(results, wheels);

plots = [
    reg.plot_errors(results);
    reg.plot_momenta(results);
    reg.plot_wheel_momenta(wheels);
    reg.plot_rotations(results);
];

u.save_plots(save_plots, 'BellyFlop', plots);
u.show_plots(show_plots, plots);


%% Thruster Misalignment

% Reset Workspace
[u, reg, quat] = util().reset();

params = {};
params.J = J;
params.T = 3 * 60 * 60;
params.M = 5.52;
params.M_ax = [1, 1, 1]';
params.M_time = [0, 90 * 60];
params.w_0 = [0; 0; 0];
params.q_c = [0; 0; 0; 1];
params.k_p = 1e3; params.k_d = 1e5;

results = reg.regulate(params);
wheels = reg.decompose(results);
reg.analysis(results, wheels);

plots = [
    reg.plot_errors(results);
    reg.plot_momenta(results);
    reg.plot_wheel_momenta(wheels);
    reg.plot_rotations(results);
];

u.save_plots(save_plots, 'ThrusterMisalignment', plots);
u.show_plots(show_plots, plots);


%% Roll Maneuver

% Reset Workspace
[u, reg, quat] = util().reset();

arm = sqrt((1.65 / 2)^2 + (1.27 / 2)^2);
M_each = arm * sin(pi / 4) * 10; % Thrust from each thruster
M_all = M_each * 8;

params = {};
params.J = J;
params.T = 20 * 60 * 60;
params.n = 50000;
params.M = 5.52;
params.M_ax = [0, 1, 0]';
params.M_time = [0, 100 * 60];
params.error_time = params.M_time;
% spinrate = 0.0089;
spinrate = 0.0065;
params.w_0 = [1; 0; 0] * spinrate;
params.q_c = [0; 0; 0; 1];
params.q_time = [100 * 60, params.T];
params.k_p = 0; params.k_d = 5e3;

results = reg.regulate(params);
wheels = reg.decompose(results);
reg.analysis(results, wheels);

plots = [
    reg.plot_errors(results);
    reg.plot_momenta(results);
    reg.plot_wheel_momenta(wheels);
    reg.plot_rotations(results);
];

u.save_plots(save_plots, 'Roll', plots);
u.show_plots(show_plots, plots);

