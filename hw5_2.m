% =========================================================
% Vector Field Guidance & Simulation (Complete)
% =========================================================
clear; close all; clc;
set(0, 'DefaultFigureWindowStyle', 'normal');

% --- Physical Constraints & Parameters ---
g = 9.81;
V = 30;                 % Constant speed (m/s)
a_max = 0.4 * g;        % Max lateral acceleration
r_min = V^2 / a_max;    % Minimum turning radius (approx 229.36 m)
R = 2 * r_min;          % Corner radius
L = 10 * r_min;         % Square side length
half_L = L / 2;
phi_max = atan(0.4);    % Maximum allowed bank angle

% --- Simulation Parameters ---
dt = 0.1;               
time = 0:dt:450;        % Total simulation time
p = [0; 0];             % Start at (0,0) North-East
chi = 0;                % Start flying North (0 rad)
state = 1;              % Initial State (1 = Line 1)

% Data logging
path_history = zeros(length(time), 2);
chi_history = zeros(length(time), 1);
phi_history = zeros(length(time), 1);

for i = 1:length(time)
    
    % 1. Determine commanded heading and update state
    [chi_c, state] = getGuidance(p, state, L, R, r_min);
    
    % 2. P-Controller for turn rate
    diff = chi_c - chi; 
    diff = mod(diff + pi, 2*pi) - pi; % wrap to [-pi, pi]
    
    chi_dot_cmd = diff * 2; % Proportional gain for turn rate
    
    % 3. Calculate Commanded Acceleration & Bank Angle
    ay_cmd = V * chi_dot_cmd; 
    phi_c = atan(ay_cmd / g); 
    
    % Apply constraints
    phi_c = max(-phi_max, min(phi_max, phi_c)); 
    
    % 4. Actual turn rate produced by bounded bank angle
    chi_dot = (g / V) * tan(phi_c); 
    
    % 5. Kinematic Update
    chi = chi + chi_dot * dt;
    chi = mod(chi + pi, 2*pi) - pi;
    p(1) = p(1) + V * cos(chi) * dt; % North
    p(2) = p(2) + V * sin(chi) * dt; % East
    
    % Log data
    path_history(i, :) = p';
    chi_history(i) = chi;
    phi_history(i) = phi_c;
end

% =========================================================
% Plotting
% =========================================================

% 1. Path Plot
figure('Name', 'Drone Trajectory', 'Position', [100, 100, 700, 600]);
plot(path_history(:,2), path_history(:,1), 'LineWidth', 2); hold on;
plotIdealPath(L, R); % Helper to plot the ideal rounded square
axis equal; grid on;
xlabel('East (m)'); ylabel('North (m)');
title('Drone Path: Vector Field Guidance');
legend('Drone Trajectory', 'Ideal Path', 'Location', 'southwest');

% 2. Time Histories
figure('Name', 'Time Histories', 'Position', [850, 100, 600, 800]);
subplot(3,1,1);
plot(time, path_history(:,1), time, path_history(:,2), 'LineWidth', 1.5);
grid on; ylabel('Position (m)'); title('Position vs Time'); legend('North', 'East');

subplot(3,1,2);
plot(time, rad2deg(chi_history), 'k', 'LineWidth', 1.5);
grid on; ylabel('Heading (deg)'); title('Heading vs Time');

subplot(3,1,3);
plot(time, rad2deg(phi_history), 'LineWidth', 1.5); hold on;
yline(rad2deg(phi_max), 'r', '+0.4g Bank');
yline(-rad2deg(phi_max), 'r', '-0.4g Bank');
grid on; xlabel('Time (s)'); ylabel('Bank Angle (deg)'); title('Commanded Bank Angle');

% 3. Vector Field Visualization
figure('Name', 'Vector Field', 'Position', [1500, 100, 700, 600]);
[E_grid, N_grid] = meshgrid(linspace(-0.6*L, 0.6*L, 35), linspace(-0.6*L, 0.6*L, 35));
U = zeros(size(N_grid)); V_vf = zeros(size(N_grid));

for i = 1:numel(N_grid)
    pt = [N_grid(i); E_grid(i)];
    st = determineStateFromPos(pt, L, R); % Static region check
    xc = getGuidance(pt, st, L, R, r_min);
    U(i) = sin(xc); % East component
    V_vf(i) = cos(xc); % North component
end

quiver(E_grid, N_grid, U, V_vf, 0.5, 'color', [0.5 0.5 0.5]); hold on;
plotIdealPath(L, R);
axis equal; grid on;
xlabel('East (m)'); ylabel('North (m)');
title('Vector Field With Switching');

% =========================================================
% Functions
% =========================================================

function [X_c, state] = getGuidance(p, current_state, L, R, r_min)
    % Manages switching between lines and orbits using Half-Planes
    half_L = L/2;
    state = current_state;
    
    % Corner Centers
    c1 = [half_L - R; half_L - R];   % Top-Right
    c2 = [-half_L + R; half_L - R];  % Bottom-Right
    c3 = [-half_L + R; -half_L + R]; % Bottom-Left
    c4 = [half_L - R; -half_L + R];  % Top-Left

    % State Machine / Switching Logic
    if state == 1 && p(2) >= half_L - R && p(1) >= 0; state = 2; end
    if state == 2 && p(1) <= half_L - R && p(2) >= 0; state = 3; end
    if state == 3 && p(1) <= -half_L + R && p(2) >= 0; state = 4; end
    if state == 4 && p(2) <= half_L - R && p(1) <= 0; state = 5; end
    if state == 5 && p(2) <= -half_L + R && p(1) <= 0; state = 6; end
    if state == 6 && p(1) >= -half_L + R && p(2) <= 0; state = 7; end
    if state == 7 && p(1) >= half_L - R && p(2) <= 0; state = 8; end
    if state == 8 && p(2) >= -half_L + R && p(1) >= 0; state = 1; end

    % Vector Field Commands
    switch state
        case 1; X_c = followLine([half_L; 0], [0; 1], p, r_min);  % North Edge, Fly East
        case 2; X_c = followOrbit(c1, R, 1, p);                   % TR Orbit (CW = 1)
        case 3; X_c = followLine([0; half_L], [-1; 0], p, r_min); % East Edge, Fly South
        case 4; X_c = followOrbit(c2, R, 1, p);                   % BR Orbit (CW = 1)
        case 5; X_c = followLine([-half_L; 0], [0; -1], p, r_min);% South Edge, Fly West
        case 6; X_c = followOrbit(c3, R, 1, p);                   % BL Orbit (CW = 1)
        case 7; X_c = followLine([0; -half_L], [1; 0], p, r_min); % West Edge, Fly North
        case 8; X_c = followOrbit(c4, R, 1, p);                   % TL Orbit (CW = 1)
    end
end

function X_c = followLine(r, q, p, r_min)
    k_path = 1 / r_min; 
    X_inf = 0.8 * (pi/2); 
    X_q = atan2(q(2), q(1)); 
    e_py = -sin(X_q)*(p(1) - r(1)) + cos(X_q)*(p(2) - r(2));
    X_c = X_q - X_inf * (2/pi) * atan(k_path * e_py);
end

function X_c = followOrbit(c, radius, dir, p)
    k_orbit = 2.0; % Tuned for smooth convergence
    d = norm(p - c);
    psi = atan2(p(2) - c(2), p(1) - c(1)); 
    X_c = psi + dir * ( (pi/2) + atan(k_orbit * ((d - radius)/radius)) );
end

function st = determineStateFromPos(p, L, R)
    % Helper exclusively for plotting the vector field map
    half_L = L/2;
    if p(1) >= half_L - R
        if p(2) >= half_L - R; st = 2; elseif p(2) <= -half_L + R; st = 8; else; st = 1; end
    elseif p(1) <= -half_L + R
        if p(2) >= half_L - R; st = 4; elseif p(2) <= -half_L + R; st = 6; else; st = 5; end
    else
        if p(2) >= 0; st = 3; else; st = 7; end
    end
end

function plotIdealPath(L, R)
    half_L = L/2;
    centers = [half_L-R, half_L-R; -half_L+R, half_L-R; -half_L+R, -half_L+R; half_L-R, -half_L+R];
    angs = [0, pi/2; pi/2, pi; pi, 3*pi/2; 3*pi/2, 2*pi];
    for i = 1:4
        th = linspace(angs(i,1), angs(i,2), 50);
        plot(centers(i,2) + R*sin(th), centers(i,1) + R*cos(th), 'r--', 'LineWidth', 1.5);
    end
    plot([half_L-R, -half_L+R], [half_L, half_L], 'r--', 'LineWidth', 1.5);
    plot([half_L-R, -half_L+R], [-half_L, -half_L], 'r--', 'LineWidth', 1.5);
    plot([half_L, half_L], [half_L-R, -half_L+R], 'r--', 'LineWidth', 1.5);
    plot([-half_L, -half_L], [half_L-R, -half_L+R], 'r--', 'LineWidth', 1.5);
end
