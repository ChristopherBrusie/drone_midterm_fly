
 
  %% === PASTE INTO MATLAB AND RUN ===
  drone_mass_kg = 0.033;
  g = 9.81;
  D = pi/180;  % degree-to-radian: multiply angle/rate gains by this
 
  J = [16.571710  0.830806  0.718277;
        0.830806 16.655602  1.800197;
        0.718277  1.800197 29.261652] * 1e-6; % [kg·m^2]
 
  % --- ALTITUDE: states [z, zdot], input = force perturbation [N] ---
  A_z = [0 1; 0 0];
  B_z = [0; 1/drone_mass_kg];
  Q_z = diag([100, 10]);    % tune: increase [1,1] for tighter z tracking
  R_z = 1;
  K_z = lqr(A_z, B_z, Q_z, R_z);  % 1x2: [K_z_pos, K_z_vel]
 
  % --- PITCH / X-POSITION: states [x, vx, theta_rad, q_rad] ---
  A_x = [0 1 0 0; 0 0 -9.81 0; 0 0 0 1; 0 0 0 0];
  B_x = [0; 0; 0; 1/J(2,2)];
  Q_x = diag([10, 1, 100, 10]);
  R_x = 1;
  K_x = lqr(A_x, B_x, Q_x, R_x);  % 1x4: [K_x, K_vx, K_theta, K_q]
 
  % --- ROLL / Y-POSITION: states [y, vy, phi_rad, p_rad] ---
  A_y = [0 1 0 0; 0 0 9.81 0; 0 0 0 1; 0 0 0 0];
  B_y = [0; 0; 0; 1/J(1,1)];
  Q_y = diag([10, 1, 100, 10]);
  R_y = 1;
  K_y = lqr(A_y, B_y, Q_y, R_y);  % 1x4: [K_y, K_vy, K_phi, K_p]
 
  % --- YAW: states [psi_rad, r_rad] ---
  A_psi = [0 1; 0 0];
  B_psi = [0; 1/J(3,3)];
  Q_psi = diag([1, 10]);
  R_psi = 1e6;
  K_psi = lqr(A_psi, B_psi, Q_psi, R_psi);  % 1x2: [K_psi, K_r]
 
  % --- Assemble the 4x12 K matrix (angles in deg, rates in deg/s) ---
  K = zeros(4, 12);
  %   Columns: [x, y, z, vx, vy, vz, phi, theta, psi, p, q*, r]
  %   where q* = negated pitch rate (index 10)
 
  % Row 0 (Z thrust): z and zdot only
  K(1,3)  =  K_z(1);        % z position
  K(1,6)  =  K_z(2);        % z velocity
 
  % Row 1 (L roll torque): y, vy, phi, p
  K(2,2)  =  K_y(1);        % y position
  K(2,5)  =  K_y(2);        % y velocity
  K(2,7)  =  K_y(3) * D;   % roll angle  (rad gain → deg gain)
  K(2,10) =  K_y(4) * D;   % roll rate   (rad/s gain → deg/s gain)
 
  % Row 2 (M pitch torque): x, vx, theta, q*
  % NOTE: index 11 is the NEGATED pitch rate, so we flip the sign of K_q
  K(3,1)  =  K_x(1);        % x position
  K(3,4)  =  K_x(2);        % x velocity
  K(3,8)  =  K_x(3) * D;   % pitch angle (rad gain → deg gain)
  K(3,11) = -K_x(4) * D;   % negated pitch rate (sign flip!)
 
  % Row 3 (N yaw torque): psi, r
  K(4,9)  =  K_psi(1) * D;  % yaw angle  (rad gain → deg gain)
  K(4,12) =  K_psi(2) * D;  % yaw rate   (rad/s gain → deg/s gain)
 
  disp('=== Copy this into controller_pp.c K[][] ===');
  for row = 1:4
      fprintf('{ ');
      for col = 1:12
          fprintf('%12.6ff, ', K(row,col));
      end
      fprintf('},\n');
  end
