%% Imports

% Reset Workspace
clc; clear;
reg = regulate;
quat = quaternion;

% Show plots?
show_plots = true;

% Save plots to figures/?
save_plots = true;

% Spacecraft Moment of Inertia (wet) (kg m^2)
J = [4.7921880e+06  8.4185790e+03  1.5313532e+04;
     8.4185790e+03  9.3203071e+06 -1.2106130e+04;
     1.5313532e+04 -1.2106130e+04  1.1217469e+07] * 0.00142233;


%% Disturbance Torques (SRP, GG, etc.)

T = 10 * 60;                % Time to simulate (s)
SRP = 3.46e-5;              % SRP Torque (Nm)
w_0 = [0; 0; 0];            % Initial Angular Rotation (rad/s)
q_c = [0; 0; 0; 1];         % Command Quaternion

[times, errors, momenta, X] = reg.regulate(J, w_0, q_c, SRP, T, T);
[pyramid, nasa] = reg.decompose(times, momenta);

f1 = reg.plot_momenta(times, errors, momenta);
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

T = 40 * 60;                % Time to simulate (s)
M = 10;                     % RCS Thruster Torque (Nm)
M_time = 10;                % Time of RCS Thruster Misfire (s)
w_0 = [0; 0; 0];            % Initial Angular Rotation (rad/s)
q_c = [0; 0; 0; 1];         % Command Quaternion

[times, errors, momenta, X] = reg.regulate(J, w_0, q_c, M, M_time, T);
[pyramid, nasa] = reg.decompose(times, momenta);

f1 = reg.plot_momenta(times, errors, momenta);
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

%% Detumble 2 (Use RCS)

T = 40 * 60;                % Time to simulate (s)
M = 4 * 10;                 % RCS Thruster Torque (Nm)
M_time = 10;                % Time of RCS Thruster Misfire (s)
w_0 = [0; 0; 0];            % Initial Angular Rotation (rad/s)
q_c = [0; 0; 0; 1];         % Command Quaternion

[times, errors, momenta, X] = reg.regulate(J, w_0, q_c, M, M_time, T);
[pyramid, nasa] = reg.decompose(times, momenta);

f1 = reg.plot_momenta(times, errors, momenta);
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

T = 40 * 60;                    % Time to simulate (s)
w_0 = [0; 0; 0];                % Initial Angular Rotation (rad/s)
q_c = quat.q([0; 0; 1], pi);    % Command Quaternion

[times, errors, momenta, X] = reg.regulate(J, w_0, q_c, 0, 0, T);
[pyramid, nasa] = reg.decompose(times, momenta);

f1 = reg.plot_momenta(times, errors, momenta);
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

T = 60 * 60;                    % Time to simulate (s)
M = 0.742;                      % Thruster Misalignment Torque (Nm)
w_0 = [0; 0; 0];                % Initial Angular Rotation (rad/s)
q_c = [0; 0; 0; 1];             % Command Quaternion

[times, errors, momenta, X] = reg.regulate(J, w_0, q_c, M, T, T);
[pyramid, nasa] = reg.decompose(times, momenta);

f1 = reg.plot_momenta(times, errors, momenta);
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
