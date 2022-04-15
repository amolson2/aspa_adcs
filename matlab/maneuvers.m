%% Imports

% Reset Workspace
clc; clear;
reg = regulate;
quat = quaternion;

% Show plots?
show_plots = true;

% Save plots to figures/?
save_plots = true;

% (old)
% Spacecraft Moment of Inertia (wet) (kg m^2)
% J = [4.7921880e+06  8.4185790e+03  1.5313532e+04;
%      8.4185790e+03  9.3203071e+06 -1.2106130e+04;
%      1.5313532e+04 -1.2106130e+04  1.1217469e+07] * 0.00142233;

J = [3.1019279e+10  2.2047148e+07  2.2994735e+08;
     2.2047148e+07  4.0036354e+10 -1.9779199e+07;
     2.2994735e+08 -1.9779199e+07  3.9828543e+10] * 0.00142233;


%% Disturbance Torques (SRP, GG, etc.)

clc;

T = 20 * 60 * 60;           % Time to simulate (s)
M = 1.10e-5;                % SRP Torque (Nm)
w_0 = [0; 0; 0];            % Initial Angular Rotation (rad/s)
q_c = [0; 0; 0; 1];         % Command Quaternion
k_p = 10; k_d = 150;        % Control Gains

[times, ~, momenta, h_dots, X] = reg.regulate(J, w_0, q_c, M, T, T, k_p, k_d);
[pyramid, nasa] = reg.decompose(times, momenta);

f1 = reg.plot_momenta(times, momenta, h_dots);
f2 = reg.plot_wheel_momenta(times, pyramid, nasa);
f3 = reg.plot_rotations(X);

if save_plots
    saveas(f1, 'figures/Disturbance_Momenta.png');
    saveas(f2, 'figures/Disturbance_Wheel_Momenta.png');
    saveas(f3, 'figures/Disturbance_Rotations.png');
end

if show_plots
    set(f1, 'visible', 'on');
    set(f2, 'visible', 'on');
    set(f3, 'visible', 'on');
end

%% Detumble (Thruster Misfire, Launch Vehicle, etc.)

clc;

T = 2 * 60 * 60;            % Time to simulate (s)
M = 1 * 10;                 % RCS Thruster Torque (Nm)
M_time = 10;                % Time of RCS Thruster Misfire (s)
w_0 = [0; 0; 0];            % Initial Angular Rotation (rad/s)
q_c = [0; 0; 0; 1];         % Command Quaternion
k_p = 800; k_d = 1e5;       % Control Gains

[times, ~, momenta, h_dots, X] = reg.regulate(J, w_0, q_c, M, M_time, T, k_p, k_d);
[pyramid, nasa] = reg.decompose(times, momenta);

f1 = reg.plot_momenta(times, momenta, h_dots);
f2 = reg.plot_wheel_momenta(times, pyramid, nasa);
f3 = reg.plot_rotations(X);

if save_plots
    saveas(f1, 'figures/Detumble_Momenta.png');
    saveas(f2, 'figures/Detumble_Wheel_Momenta.png');
    saveas(f3, 'figures/Detumble_Rotations.png');
end

if show_plots
    set(f1, 'visible', 'on');
    set(f2, 'visible', 'on');
    set(f3, 'visible', 'on');
end

%% Belly Flop (180 degree turn)

clc;

T = 12 * 60 * 60;               % Time to simulate (s)
w_0 = [0; 0; 0];                % Initial Angular Rotation (rad/s)
e_hat = [1; 1; 0];              % Axis of rotation
e_hat = e_hat / norm(e_hat);    % Normalizing axis of rotation
q_c = quat.q(e_hat, pi);        % Command Quaternion
k_p = 30; k_d = 1e5;            % Control Gains

[times, ~, momenta, h_dots, X] = reg.regulate(J, w_0, q_c, 0, 0, T, k_p, k_d);
[pyramid, nasa] = reg.decompose(times, momenta);

f1 = reg.plot_momenta(times, momenta, h_dots);
f2 = reg.plot_wheel_momenta(times, pyramid, nasa);
f3 = reg.plot_rotations(X);

if save_plots
    saveas(f1, 'figures/BellyFlop_Momenta.png');
    saveas(f2, 'figures/BellyFlop_Wheel_Momenta.png');
    saveas(f3, 'figures/BellyFlop_Rotations.png');
end

if show_plots
    set(f1, 'visible', 'on');
    set(f2, 'visible', 'on');
    set(f3, 'visible', 'on');
end

%% Thruster Misalignment

clc;

T = 2 * 60 * 60;                % Time to simulate (s)
% M = 0.742;                    % Thruster Misalignment Torque (Nm)
M = 5.52;                       % Thruster Misalignment Torque (Nm)
M_time = 90 * 60;               % Thruster burn time (s)
w_0 = [0; 0; 0];                % Initial Angular Rotation (rad/s)
q_c = [0; 0; 0; 1];             % Command Quaternion
k_p = 1e3; k_d = 1e5;           % Control Gains

[times, ~, momenta, h_dots, X] = reg.regulate(J, w_0, q_c, M, M_time, T, k_p, k_d);
[pyramid, nasa] = reg.decompose(times, momenta);

f1 = reg.plot_momenta(times, momenta, h_dots);
f2 = reg.plot_wheel_momenta(times, pyramid, nasa);
f3 = reg.plot_rotations(X);

if save_plots
    saveas(f1, 'figures/ThrusterMisalignment_Momenta.png');
    saveas(f2, 'figures/ThrusterMisalignment_Wheel_Momenta.png');
    saveas(f3, 'figures/ThrusterMisalignment_Rotations.png');
end

if show_plots
    set(f1, 'visible', 'on');
    set(f2, 'visible', 'on');
    set(f3, 'visible', 'on');
end
