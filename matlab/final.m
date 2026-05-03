%% System Params

A_thrust = 0.091492681; 
B_thrust = 0.067673604; % thrust map params
L = 0.046; % drone arm length
C_tau = 0.005964552; % torque
d = L * (sqrt(2)/2); % distance between motor and drone COM

% Physical Force Limits per motor 0-1 range
F_min = 0;
F_max = A_thrust*(1)^2 + B_thrust*(1); 

Fc_cmd = 0.323; % ~Hover force

drone_mass_kg = 0.033;


%% Motor Mixer 
function [pwm_output_int, F_des] = quad_mixer(Fc, Lc, Mc, Nc, A, B, d, C_tau, F_min, F_max)
    
    % 1. Linear Mapping Matrix
    M_mix = [1,      1,      1,      1; 
            -d,     -d,      d,      d; 
             d,     -d,     -d,      d; 
             C_tau, -C_tau,  C_tau, -C_tau];
         
    % Calculate desired forces
    U_cmd = [Fc; Lc; Mc; Nc];
    F_des = M_mix \ U_cmd;
    
    % 2 & 3. Actuator Saturation & Infeasible Command Handling
    % Strategy: Shift forces to preserve moments, scale if impossible.
    
    if max(F_des) > F_max
        % Shift down (Sacrifice altitude to keep attitude)
        shift = max(F_des) - F_max;
        F_des = F_des - shift;
    end
    
    if min(F_des) < F_min
        % Shift up
        shift = F_min - min(F_des);
        F_des = F_des + shift;
    end
    
    % If the spread is wider than physical limits, we must scale moments down
    if (max(F_des) - min(F_des)) > (F_max - F_min)
        mid_req = mean(F_des);
        mid_phys = (F_max + F_min) / 2;
        scale = (F_max - F_min) / (max(F_des) - min(F_des));
        
        % Center and scale
        F_des = mid_phys + scale * (F_des - mid_req);
    end
    
    % Final safety clamp against floating point errors
    F_des = max(min(F_des, F_max), F_min);
    
    % 4. Nonlinear Inversion (Force to PWM Ratio)
    % Solve: A*x^2 + B*x - F_des = 0 for positive root
    % ratio (0-1)
    pwm_ratio = (-B + sqrt(B^2 + 4*A.*F_des)) ./ (2*A);
    
    pwm_pct = pwm_ratio * 100;

    pwm_output_int = round(655.35*pwm_pct);
end


%% System Model


