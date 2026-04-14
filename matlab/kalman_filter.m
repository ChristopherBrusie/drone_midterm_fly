%% Pt A - Modeling
% 1. assuming constant velocity, simple integrator dynamics. 
% states 3, 4 are constant.

clear all
close all

sigma = 0.5;

R = eye(2).*(sigma^2);

Q = eye(4);

Ts = 1;

% 2 continuous state space model
A = [0 0 1 0;
     0 0 0 1;
     0 0 0 0;
     0 0 0 0];

C = [1 0 0 0;
     0 1 0 0];

% 3 discrete time state space model
sys_cont = ss(A, zeros(4,4), C, zeros(2,4));
sys_disc = c2d(sys_cont, Ts, 'zoh');
A_d = sys_disc.A;
B_d = sys_disc.B;


% 4 initial state and covariance
x0 = [0;0;1;1];
P0 = Q;
% justification: i don't know

%% Part B - KF implementation

y_trial_1 = [ 4.2484 4.9309 6.3238 7.7615 7.8829;
            2.8829 5.7896 7.3837 8.7653 11.2713];

y_trial_2 = [3.7683 4.7671 6.1210 6.0434 7.1375;
            2.7189 4.4936 7.1571 8.5460 10.2938];



% propogate discrete synamics (exponent)
state_pred = [];
for i = 1:5
    state_pred(:, i) = A_d * x0;
    x0 = state_pred(:, i); % Update state for next iteration
end


% plotting x position for trial 1
tspace = 1:Ts:5;
plot(tspace, y_trial_2(1,:), 'o', 'LineWidth', 2, 'MarkerSize', 12)
hold on
plot(state_pred(1,:), tspace, 'LineWidth', 2)

