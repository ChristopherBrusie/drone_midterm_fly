% =========================================================
%  Crazyflie Flight Log Plotter
%  - Auto-detects attitude-rate, attitude, position,
%    velocity, and motor CSVs from a folder
%  - Detects takeoff from motor data; all plots share a
%    common time axis where t = 0 is the moment of takeoff
%  - Works with WSL2 UNC paths (wsl.localhost / wsl$)
%
%  USAGE: set LOG_DIR below, then run.
% =========================================================

clear; clc; close all;

% ----------------------------------------------------------
% >>>  SET THIS TO YOUR LOG SESSION FOLDER  <<<
% ----------------------------------------------------------
LOG_DIR = '\\wsl.localhost\Ubuntu-22.04\home\chris\.config\cfclient\logdata\20260429T12-45-19\';

% Motor PWM threshold above which the drone is considered
% spooling up for takeoff (0-65535 range). Adjust if needed.
MOTOR_TAKEOFF_THRESHOLD = 1000;

% Number of consecutive samples that must exceed the threshold
% before takeoff is declared (guards against noise spikes).
TAKEOFF_CONFIRM_SAMPLES = 3;

% ----------------------------------------------------------
% RESOLVE WSL PATH
% ----------------------------------------------------------
if ~isfolder(LOG_DIR)
    alt = strrep(LOG_DIR, '\\wsl.localhost\', '\\wsl$\');
    if isfolder(alt)
        LOG_DIR = alt;
        fprintf('Note: using legacy WSL path: %s\n', LOG_DIR);
    else
        error('Cannot find folder:\n  %s\nCheck LOG_DIR at the top of the script.', LOG_DIR);
    end
end

% ----------------------------------------------------------
% HELPER FUNCTIONS
% ----------------------------------------------------------
function s = pick(a, b)
    if ~isempty(a); s = a; else; s = b; end
end

function T = load_cf_table(filepath)
    % Read CSV via a temp copy to avoid UNC-path issues with readtable.
    % Returns the raw table — time alignment is done by the caller.
    tmp = [tempname '.csv'];
    copyfile(filepath, tmp);
    T = readtable(tmp, 'VariableNamingRule', 'preserve');
    delete(tmp);
end

function t_s = align_time(T, takeoff_ms)
    % Convert absolute Timestamp (ms) to seconds relative to takeoff.
    % Values before takeoff will be negative.
    t_s = (T.Timestamp - takeoff_ms) / 1000;
end

function tf = has_col(T, name)
    tf = any(strcmp(T.Properties.VariableNames, name));
end

function safe_xlim(t)
    if numel(t) > 1; xlim([t(1) t(end)]); end
end

function add_takeoff_line()
    % Draw a vertical dashed line at t = 0 (takeoff moment).
    yl = ylim;
    plot([0 0], yl, 'k--', 'LineWidth', 1, 'DisplayName', 'Takeoff', ...
         'HandleVisibility', 'on');
end

% ----------------------------------------------------------
% AUTO-DETECT FILES BY HEADER
% ----------------------------------------------------------
csv_files = dir(fullfile(LOG_DIR, '*.csv'));
if isempty(csv_files)
    error('No CSV files found in:\n  %s', LOG_DIR);
end

rates_file = ''; att_file = ''; pos_file = '';
vel_file   = ''; motor_file = '';

for i = 1:numel(csv_files)
    fpath = fullfile(LOG_DIR, csv_files(i).name);
    fid   = fopen(fpath, 'r');
    header = fgetl(fid);
    fclose(fid);

    if contains(header, 'pitchRate') || contains(header, 'gyro.')
        rates_file = fpath;
    elseif contains(header, 'controller.pitch') && ~contains(header, 'Rate')
        att_file = fpath;
    elseif contains(header, 'posCtl.targetX') || contains(header, 'stateEstimate.x,')
        pos_file = fpath;
    elseif contains(header, 'posCtl.targetVX') || contains(header, 'stateEstimate.vy') || ...
           contains(header, 'velocity', 'IgnoreCase', true)
        vel_file = fpath;
    elseif contains(header, 'motor.m1')
        motor_file = fpath;
    end
end

fprintf('Files identified:\n');
fprintf('  Attitude Rate : %s\n', pick(rates_file,  '(not found)'));
fprintf('  Attitude      : %s\n', pick(att_file,    '(not found)'));
fprintf('  Position      : %s\n', pick(pos_file,    '(not found)'));
fprintf('  Velocity      : %s\n', pick(vel_file,    '(not found)'));
fprintf('  Motors        : %s\n', pick(motor_file,  '(not found)'));
fprintf('\n');

% ----------------------------------------------------------
% LOAD ALL DATA
% ----------------------------------------------------------
if ~isempty(rates_file);  rates    = load_cf_table(rates_file);  end
if ~isempty(att_file);    attitude = load_cf_table(att_file);    end
if ~isempty(pos_file);    position = load_cf_table(pos_file);    end
if ~isempty(vel_file);    velocity = load_cf_table(vel_file);    end
if ~isempty(motor_file);  motors   = load_cf_table(motor_file);  end

% ----------------------------------------------------------
% DETECT TAKEOFF FROM MOTOR DATA
% ----------------------------------------------------------
takeoff_ms = [];   % will remain empty if motor file unavailable

if ~isempty(motor_file)
    % Sum all four motor PWM values — more robust than any single motor.
    motor_cols = {'motor.m1','motor.m2','motor.m3','motor.m4'};
    present    = motor_cols(cellfun(@(c) has_col(motors,c), motor_cols));

    if ~isempty(present)
        total_pwm = zeros(height(motors), 1);
        for c = present
            total_pwm = total_pwm + motors.(c{1});
        end

        % Also check stabilizer.thrust as an alternative signal
        if has_col(motors, 'stabilizer.thrust')
            total_pwm = max(total_pwm, motors.('stabilizer.thrust'));
        end

        % Find first run of TAKEOFF_CONFIRM_SAMPLES consecutive samples
        % all above MOTOR_TAKEOFF_THRESHOLD.
        above   = total_pwm > MOTOR_TAKEOFF_THRESHOLD;
        run_len = 0;
        takeoff_idx = [];
        for idx = 1:numel(above)
            if above(idx)
                run_len = run_len + 1;
                if run_len >= TAKEOFF_CONFIRM_SAMPLES
                    takeoff_idx = idx - TAKEOFF_CONFIRM_SAMPLES + 1;
                    break;
                end
            else
                run_len = 0;
            end
        end

        if ~isempty(takeoff_idx)
            takeoff_ms = motors.Timestamp(takeoff_idx);
            fprintf('Takeoff detected at %.3f s into motor log (absolute timestamp: %d ms).\n', ...
                    (takeoff_ms - motors.Timestamp(1)) / 1000, takeoff_ms);
        else
            fprintf('Warning: motors never exceeded threshold — no takeoff detected.\n');
            fprintf('  Plots will use the start of each file as t=0 instead.\n');
        end
    end
end

% Fallback: use earliest timestamp across all loaded files as t=0
if isempty(takeoff_ms)
    all_t0 = [];
    if exist('rates',    'var'); all_t0(end+1) = rates.Timestamp(1);    end
    if exist('attitude', 'var'); all_t0(end+1) = attitude.Timestamp(1); end
    if exist('position', 'var'); all_t0(end+1) = position.Timestamp(1); end
    if exist('velocity', 'var'); all_t0(end+1) = velocity.Timestamp(1); end
    if exist('motors',   'var'); all_t0(end+1) = motors.Timestamp(1);   end
    takeoff_ms = min(all_t0);
end

% Compute aligned time vectors (t=0 at takeoff)
if exist('rates',    'var'); t_rates = align_time(rates,    takeoff_ms); end
if exist('attitude', 'var'); t_att   = align_time(attitude, takeoff_ms); end
if exist('position', 'var'); t_pos   = align_time(position, takeoff_ms); end
if exist('velocity', 'var'); t_vel   = align_time(velocity, takeoff_ms); end
if exist('motors',   'var'); t_mot   = align_time(motors,   takeoff_ms); end

% ----------------------------------------------------------
% COLOUR PALETTE
% ----------------------------------------------------------
C_SP  = [0.20 0.45 0.80];   % blue   – setpoint / controller
C_EST = [0.85 0.20 0.20];   % red    – state estimate
C_XYZ = [0.10 0.65 0.35;    % green  – x / pitch
          0.90 0.55 0.10;    % amber  – y / roll
          0.55 0.10 0.75];   % purple – z / yaw
C_MOT = [0.00 0.45 0.70;    % motor colours (ColorBrewer accessible)
          0.90 0.35 0.00;
          0.00 0.62 0.45;
          0.80 0.47 0.65];
LW = 1.5;

% ==========================================================
% FIGURE 1 — Angular Rates
% ==========================================================
if exist('rates', 'var')
    ctrl_cols = {'controller.pitchRate','controller.rollRate','controller.yawRate'};
    gyro_cols = {'gyro.x','gyro.y','gyro.z'};
    ylabels   = {'Pitch Rate (rad/s)','Roll Rate (rad/s)','Yaw Rate (rad/s)'};

    figure('Name','Angular Rates','NumberTitle','off','Color','w','Position',[30 530 1100 560]);
    for k = 1:3
        subplot(3,1,k); hold on; grid on; box on;
        if has_col(rates, ctrl_cols{k})
            plot(t_rates, rates.(ctrl_cols{k}), '--', 'Color', C_SP, ...
                 'LineWidth', LW, 'DisplayName', 'Controller cmd');
        end
        if has_col(rates, gyro_cols{k})
            plot(t_rates, rates.(gyro_cols{k}), '-', 'Color', C_XYZ(k,:), ...
                 'LineWidth', LW, 'DisplayName', 'Gyro');
        end
        add_takeoff_line();
        ylabel(ylabels{k}); legend('Location','best','FontSize',8);
        if k == 1; title('Angular Rates — Controller Setpoint vs Gyro'); end
        if k == 3; xlabel('Time relative to takeoff (s)'); end
        safe_xlim(t_rates);
    end
else
    fprintf('Skipping Figure 1 — attitude rate file not found.\n');
end

% ==========================================================
% FIGURE 2 — Attitude
% ==========================================================
if exist('attitude', 'var')
    sp_cols  = {'controller.pitch','controller.roll','controller.yaw'};
    est_cols = {'stateEstimate.pitch','stateEstimate.roll','stateEstimate.yaw'};
    ylabels2 = {'Pitch (deg)','Roll (deg)','Yaw (deg)'};

    figure('Name','Attitude','NumberTitle','off','Color','w','Position',[30 30 1100 560]);
    for k = 1:3
        subplot(3,1,k); hold on; grid on; box on;
        if has_col(attitude, sp_cols{k})
            plot(t_att, rad2deg(attitude.(sp_cols{k})), '--', 'Color', C_SP, ...
                 'LineWidth', LW, 'DisplayName', 'Setpoint');
        end
        if has_col(attitude, est_cols{k})
            plot(t_att, rad2deg(attitude.(est_cols{k})), '-', 'Color', C_EST, ...
                 'LineWidth', LW, 'DisplayName', 'State Estimate');
        end
        add_takeoff_line();
        ylabel(ylabels2{k}); legend('Location','best','FontSize',8);
        if k == 1; title('Attitude — Setpoint vs State Estimate'); end
        if k == 3; xlabel('Time relative to takeoff (s)'); end
        safe_xlim(t_att);
    end
else
    fprintf('Skipping Figure 2 — attitude file not found.\n');
end

% ==========================================================
% FIGURE 3 — Position time-series
% FIGURE 4 — XY top-down path
% FIGURE 5 — 3D path
% ==========================================================
if exist('position', 'var')
    tgt_cols = {'posCtl.targetX','posCtl.targetY','posCtl.targetZ'};
    pos_cols = {'stateEstimate.x','stateEstimate.y','stateEstimate.z'};
    ylabels3 = {'X (m)','Y (m)','Z (m)'};

    figure('Name','Position','NumberTitle','off','Color','w','Position',[580 530 1100 560]);
    for k = 1:3
        subplot(3,1,k); hold on; grid on; box on;
        if has_col(position, tgt_cols{k})
            plot(t_pos, position.(tgt_cols{k}), '--', 'Color', C_SP, ...
                 'LineWidth', LW, 'DisplayName', 'Target');
        end
        if has_col(position, pos_cols{k})
            plot(t_pos, position.(pos_cols{k}), '-', 'Color', C_EST, ...
                 'LineWidth', LW, 'DisplayName', 'Estimate');
        end
        add_takeoff_line();
        ylabel(ylabels3{k}); legend('Location','best','FontSize',8);
        if k == 1; title('Position — Target vs State Estimate'); end
        if k == 3; xlabel('Time relative to takeoff (s)'); end
        safe_xlim(t_pos);
    end

    % Only plot path figures from takeoff onwards
    post_idx = t_pos >= 0;

    % XY top-down
    figure('Name','XY Flight Path','NumberTitle','off','Color','w','Position',[580 30 500 480]);
    hold on; grid on; box on; axis equal;
    if has_col(position,'posCtl.targetX')
        plot(position.('posCtl.targetX')(post_idx), position.('posCtl.targetY')(post_idx), ...
             '--', 'Color', C_SP, 'LineWidth', LW, 'DisplayName', 'Target');
    end
    if has_col(position,'stateEstimate.x')
        ex = position.('stateEstimate.x')(post_idx);
        ey = position.('stateEstimate.y')(post_idx);
        plot(ex, ey, '-', 'Color', C_EST, 'LineWidth', LW, 'DisplayName', 'Estimate');
        scatter(ex(1),   ey(1),   80, [0.1 0.7 0.1], 'filled', 'DisplayName', 'Takeoff');
        scatter(ex(end), ey(end), 80, [0.8 0.1 0.1], 'filled', 'DisplayName', 'End');
    end
    xlabel('X (m)'); ylabel('Y (m)'); title('Top-Down Flight Path (from takeoff)');
    legend('Location','best','FontSize',8);

    % 3D path
    figure('Name','3D Flight Path','NumberTitle','off','Color','w','Position',[1110 30 540 480]);
    hold on; grid on; box on;
    if has_col(position,'posCtl.targetX')
        plot3(position.('posCtl.targetX')(post_idx), position.('posCtl.targetY')(post_idx), ...
              position.('posCtl.targetZ')(post_idx), '--', 'Color', C_SP, ...
              'LineWidth', LW, 'DisplayName', 'Target');
    end
    if has_col(position,'stateEstimate.x')
        ex = position.('stateEstimate.x')(post_idx);
        ey = position.('stateEstimate.y')(post_idx);
        ez = position.('stateEstimate.z')(post_idx);
        plot3(ex, ey, ez, '-', 'Color', C_EST, 'LineWidth', LW, 'DisplayName', 'Estimate');
        scatter3(ex(1),   ey(1),   ez(1),   80, [0.1 0.7 0.1], 'filled', 'DisplayName', 'Takeoff');
        scatter3(ex(end), ey(end), ez(end), 80, [0.8 0.1 0.1], 'filled', 'DisplayName', 'End');
    end
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('3D Flight Path (from takeoff)');
    legend('Location','best','FontSize',8); view([-45 30]);
else
    fprintf('Skipping Figures 3/4/5 — position file not found.\n');
end

% ==========================================================
% FIGURE 6 — Velocity
% ==========================================================
if exist('velocity', 'var')
    sp_candidates  = {'posCtl.targetVX','posCtl.targetVY','posCtl.targetVZ', ...
                      'ctrltarget.vx','ctrltarget.vy','ctrltarget.vz'};
    est_candidates = {'stateEstimate.vx','stateEstimate.vy','stateEstimate.vz', ...
                      'kalman.statePX','kalman.statePY','kalman.statePZ'};

    vel_sp_cols  = sp_candidates( cellfun(@(c) has_col(velocity,c), sp_candidates));
    vel_est_cols = est_candidates(cellfun(@(c) has_col(velocity,c), est_candidates));

    if isempty(vel_sp_cols) && isempty(vel_est_cols)
        vel_est_cols = velocity.Properties.VariableNames( ...
                           ~strcmp(velocity.Properties.VariableNames,'Timestamp'));
    end

    axes_lbl = {'vx (m/s)','vy (m/s)','vz (m/s)'};
    n_sub    = min(numel(vel_est_cols), 3);

    figure('Name','Velocity','NumberTitle','off','Color','w','Position',[30 30 1100 560]);
    for k = 1:n_sub
        subplot(n_sub,1,k); hold on; grid on; box on;
        if k <= numel(vel_sp_cols)
            plot(t_vel, velocity.(vel_sp_cols{k}), '--', 'Color', C_SP, ...
                 'LineWidth', LW, 'DisplayName', strrep(vel_sp_cols{k},'.','\_'));
        end
        plot(t_vel, velocity.(vel_est_cols{k}), '-', 'Color', C_XYZ(k,:), ...
             'LineWidth', LW, 'DisplayName', strrep(vel_est_cols{k},'.','\_'));
        add_takeoff_line();
        if k <= 3; ylabel(axes_lbl{k}); end
        legend('Location','best','FontSize',8);
        if k == 1; title('Velocity'); end
        if k == n_sub; xlabel('Time relative to takeoff (s)'); end
        safe_xlim(t_vel);
    end
else
    fprintf('Skipping Figure 6 — velocity file not found.\n');
end

% ==========================================================
% FIGURE 7 — Motors
% ==========================================================
if exist('motors', 'var')
    mot_cols   = {'motor.m1','motor.m2','motor.m3','motor.m4'};
    mot_labels = {'M1','M2','M3','M4'};
    present    = mot_cols(cellfun(@(c) has_col(motors,c), mot_cols));

    n_rows = double(has_col(motors,'stabilizer.thrust')) + 1;

    figure('Name','Motors','NumberTitle','off','Color','w','Position',[30 30 1100 560]);

    % ---- Subplot 1: individual motor PWM ----
    subplot(n_rows,1,1); hold on; grid on; box on;
    for k = 1:numel(present)
        col_idx = strcmp(mot_cols, present{k});
        lbl_idx = find(col_idx);
        plot(t_mot, motors.(present{k}), '-', 'Color', C_MOT(lbl_idx,:), ...
             'LineWidth', LW, 'DisplayName', mot_labels{lbl_idx});
    end
    % Shade the pre-takeoff region
    yl = ylim;
    patch([t_mot(1) 0 0 t_mot(1)], [yl(1) yl(1) yl(2) yl(2)], ...
          [0.85 0.85 0.85], 'FaceAlpha', 0.35, 'EdgeColor', 'none', ...
          'DisplayName', 'Pre-takeoff');
    plot([0 0], ylim, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Takeoff');
    ylabel('PWM (0–65535)');
    title('Motor Commands');
    legend('Location','best','FontSize',8,'NumColumns',3);
    safe_xlim(t_mot);

    % ---- Subplot 2: stabilizer thrust (if present) ----
    if has_col(motors, 'stabilizer.thrust')
        subplot(n_rows,1,2); hold on; grid on; box on;
        plot(t_mot, motors.('stabilizer.thrust'), '-', 'Color', C_SP, ...
             'LineWidth', LW, 'DisplayName', 'Thrust');
        yl = ylim;
        patch([t_mot(1) 0 0 t_mot(1)], [yl(1) yl(1) yl(2) yl(2)], ...
              [0.85 0.85 0.85], 'FaceAlpha', 0.35, 'EdgeColor', 'none', ...
              'HandleVisibility', 'off');
        plot([0 0], ylim, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Takeoff');
        ylabel('Thrust (PWM)');
        legend('Location','best','FontSize',8);
        xlabel('Time relative to takeoff (s)');
        safe_xlim(t_mot);
    else
        xlabel('Time relative to takeoff (s)');
    end
else
    fprintf('Skipping Figure 7 — motor file not found.\n');
end

fprintf('Done. t = 0 on all plots marks the detected takeoff moment.\n');