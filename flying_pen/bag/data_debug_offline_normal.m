%% data_debug_offline_normal.m
% Offline momentum-observer / normal-estimation dashboard.
%
% Reference logic:
% - mujoco_crazyflie/src/flyingpen/src/wrench_observer.cpp
% - mujoco_crazyflie/src/flyingpen/src/normal_vector_estimation.cpp
%
% This script follows the style of data_debug.m, but reconstructs
% - 2nd-order momentum observer
% - consistency-corrected momentum observer
% - normal estimation with / without I - v v^T projection
% directly from the logged pose / attitude / velocity / thrust signals.

clear; close all; clc;
set(groot, 'defaultFigureRenderer', 'painters');

%% 0) User config
sample_hz = 50.0;
default_dir = fullfile(getenv("HOME"), "hitl_ws", "src", "flying_pen", "bag", "logging");
if ~isfolder(default_dir)
    default_dir = pwd;
end

% Observer parameters (from flyingpen_interface/config/wrench_observer.yaml)
cfg.mass = 0.04338;
cfg.g = 9.81;
cfg.J = [2.3951e-5, 2.3951e-5, 3.2347e-5];
cfg.arm_xy = 0.035355;
cfg.k_tau_motor = 0.00569278844371417;
% Match Crazyflie powerDistributionForceTorque() yawPart signs:
% m1/m3 contribute negative tau_z, m2/m4 contribute positive tau_z.
cfg.motor_dir = [-1.0, 1.0, -1.0, 1.0];
cfg.ee_offset_body = [0.08; 0.0; 0.04];
cfg.Kf = 30.0;
cfg.Ktau = 30.0;
cfg.mob_alpha = 1.0;
cfg.Kp = 10.9544511501;
cfg.KpTau = 10.9544511501;
cfg.Ke = 900.0;
cfg.epsilon_tau = 1.0e-6;
cfg.observer_dt = 0.004;

% Optional velocity LPF before the offline observer (empty = disabled)
vel_lpf_cutoff_hz = [];
omega_lpf_cutoff_hz = 5.0;

% Normal estimation parameters (from normal_vector_estimation.yaml)
normal_cfg.force_threshold = 5.0e-4;
normal_cfg.epsilon_f = 0.005;
normal_cfg.epsilon_n = 0.005;
normal_cfg.projection_vel_lpf_cutoff_hz = [3];
normal_cfg.projection_gate_epsilon = 0.01;
normal_cfg.normal_output_lpf_cutoff_hz = 6.0 / (2.0 * pi);

% Global plot window for every panel (empty = auto)
global_xlim = [40 90];

%% 1) Pick CSV and read
[file, path] = uigetfile(fullfile(default_dir, "*.csv"), "Select debug logging CSV");
if isequal(file, 0)
    disp("Canceled.");
    return;
end
csv_path = fullfile(path, file);
fprintf("[INFO] Reading: %s\n", csv_path);

opts = detectImportOptions(csv_path, 'Delimiter', ',');
for i = 1:numel(opts.VariableTypes)
    opts.VariableTypes{i} = 'double';
end
T = readtable(csv_path, opts);
if isempty(T) || height(T) < 5
    error("CSV has too few rows.");
end

vars = string(T.Properties.VariableNames);
get1 = @(name) local_get1(T, vars, name);

if any(strcmp(vars, "t_sec"))
    time = T{:, "t_sec"};
    time = time - time(find(isfinite(time), 1, 'first'));
else
    time = (0:height(T)-1).' ./ sample_hz;
end

%% 2) Load signals
pose_xyz = [get1("pose_x"), get1("pose_y"), get1("pose_z")];
% Use measured Euler angles as logged; Crazyflie pitch sign flips exist in
% some command decoding paths, not in the attitude state itself.
pose_rpy = [get1("pose_roll"), get1("pose_pitch"), unwrap(get1("pose_yaw"))];
cmd_xyz = [get1("cmd_x"), get1("cmd_y"), get1("cmd_z")];
att_des = deg2rad([get1("attDes_roll"), get1("attDes_pitch"), get1("attDes_yaw")]);

motor_thrust = [get1("f1"), get1("f2"), get1("f3"), get1("f4")];
state_vel = [get1("stateVx"), get1("stateVy"), get1("stateVz")];

valid = isfinite(time) & ...
    all(isfinite(pose_xyz), 2) & ...
    all(isfinite(pose_rpy), 2) & ...
    all(isfinite(state_vel), 2) & ...
    all(isfinite(motor_thrust), 2);
time = time(valid);
pose_xyz = pose_xyz(valid, :);
pose_rpy = pose_rpy(valid, :);
cmd_xyz = cmd_xyz(valid, :);
att_des = att_des(valid, :);
state_vel = state_vel(valid, :);

N = numel(time);
fprintf("[INFO] Using %d rows.\n", N);

%% 3) Derived signals
dt = [1.0 / sample_hz; diff(time)];
dt(~isfinite(dt) | dt <= 0.0) = 1.0 / sample_hz;
dt = min(max(dt, 1.0e-4), 0.05);

sum_thrust = sum(motor_thrust, 2, 'omitnan');
body_rates_from_rotm = local_body_rates_from_rotm(pose_rpy, time);
body_rates_from_rotm = local_lowpass_first_order(body_rates_from_rotm, sample_hz, omega_lpf_cutoff_hz);

if ~isempty(vel_lpf_cutoff_hz) && isfinite(vel_lpf_cutoff_hz) && vel_lpf_cutoff_hz > 0
    state_vel_used = local_lowpass_first_order(state_vel, sample_hz, vel_lpf_cutoff_hz);
else
    state_vel_used = state_vel;
end
body_rates_used = body_rates_from_rotm;

%% 4) Offline momentum observer replay
pure_state = local_make_observer_state();
cons_state = local_make_consistency_state();

mob_force_none_off = zeros(N, 3);      % ee-applied sign, for normal estimation
mob_force_res_off = zeros(N, 3);       % ee-applied sign, for normal estimation
mob_force_none_drone_off = zeros(N, 3);
mob_force_res_drone_off = zeros(N, 3);
mob_force_base_drone_off = zeros(N, 3);
mob_torque_none_drone_off = zeros(N, 3);
mob_torque_res_drone_off = zeros(N, 3);
mob_tauhat_world_off = zeros(N, 3);
rxf_world_off = zeros(N, 3);
e_tau_world_off = zeros(N, 3);
ee_vel_world = zeros(N, 3);
u_force_world_hist = zeros(N, 3);
u_force_body_hist = zeros(N, 3);
u_tau_body_hist = zeros(N, 3);
v_world_hist = zeros(N, 3);
w_body_hist = zeros(N, 3);
p_lin_world_hist = zeros(N, 3);
p_ang_body_hist = zeros(N, 3);
cori_body_hist = zeros(N, 3);

for k = 1:N
    r_bw = local_rpy_to_rotm(pose_rpy(k, 1), pose_rpy(k, 2), pose_rpy(k, 3));
    v_world = state_vel_used(k, :).';
    w_body = body_rates_used(k, :).';
    if any(~isfinite(v_world))
        v_world(~isfinite(v_world)) = 0.0;
    end
    if any(~isfinite(w_body))
        w_body(~isfinite(w_body)) = 0.0;
    end
    [u_force_body, u_tau_body] = local_allocation_wrench_from_motor_thrust( ...
        motor_thrust(k, :), cfg.arm_xy, cfg.k_tau_motor, cfg.motor_dir);
    u_force_world = r_bw * u_force_body;
    p_lin_world_sample = cfg.mass * v_world;
    p_ang_body_sample = cfg.J(:) .* w_body;
    cori_body_sample = cross(w_body, cfg.J(:) .* w_body);
    local_assert_finite('r_bw', r_bw, k, 0);
    local_assert_finite('v_world', v_world, k, 0);
    local_assert_finite('w_body', w_body, k, 0);
    local_assert_finite('u_force_body', u_force_body, k, 0);
    local_assert_finite('u_tau_body', u_tau_body, k, 0);
    local_assert_finite('u_force_world', u_force_world, k, 0);
    local_assert_finite('p_lin_world', p_lin_world_sample, k, 0);
    local_assert_finite('p_ang_body', p_ang_body_sample, k, 0);
    local_assert_finite('cori_body', cori_body_sample, k, 0);
    n_sub = max(1, round(dt(k) / cfg.observer_dt));
    sub_dt = dt(k) / n_sub;
    grav_world = [0.0; 0.0; cfg.mass * cfg.g];
    ee_offset_world = r_bw * cfg.ee_offset_body;

    for j = 1:n_sub
        p_lin_world = p_lin_world_sample;
        p_ang_body = p_ang_body_sample;
        cori_body = cori_body_sample;

        pure_state = local_run_observer_variant(pure_state, p_lin_world, p_ang_body, ...
            u_force_world, u_tau_body, grav_world, cori_body, sub_dt, cfg, true, true);
        local_assert_state_finite('pure_state', pure_state, k, j);

        cons_state = local_run_consistency_observer(cons_state, p_lin_world, p_ang_body, ...
            u_force_world, u_tau_body, grav_world, cori_body, ee_offset_world, r_bw, sub_dt, cfg);
        local_assert_state_finite('cons_state', cons_state, k, j);
    end

    mob_force_none_drone_off(k, :) = pure_state.world_force_hat_ext.';
    mob_force_res_drone_off(k, :) = cons_state.world_force_hat_ext.';
    mob_force_base_drone_off(k, :) = cons_state.force_hat_base_world.';
    mob_torque_none_drone_off(k, :) = (r_bw * pure_state.body_torque_hat_ext).';
    mob_torque_res_drone_off(k, :) = (r_bw * cons_state.body_torque_hat_ext).';

    % End-effector-applied sign for normal estimation only.
    mob_force_none_off(k, :) = -mob_force_none_drone_off(k, :);
    mob_force_res_off(k, :) = -mob_force_res_drone_off(k, :);
    mob_tauhat_world_off(k, :) = cons_state.tau_hat_ext_world.';
    rxf_world_off(k, :) = cons_state.r_cross_f_hat_ext_world.';
    e_tau_world_off(k, :) = cons_state.e_tau_world.';

    w_world = r_bw * w_body;
    ee_vel_world(k, :) = (v_world + cross(w_world, ee_offset_world)).';
    u_force_world_hist(k, :) = u_force_world.';
    u_force_body_hist(k, :) = u_force_body.';
    u_tau_body_hist(k, :) = u_tau_body.';
    v_world_hist(k, :) = v_world.';
    w_body_hist(k, :) = w_body.';
    p_lin_world_hist(k, :) = p_lin_world_sample.';
    p_ang_body_hist(k, :) = p_ang_body_sample.';
    cori_body_hist(k, :) = cori_body_sample.';
end

%% 5) Offline normal estimation replay
raw_pure_state = local_make_normal_state();
proj_pure_state = local_make_normal_state();
raw_con_state = local_make_normal_state();
proj_con_state = local_make_normal_state();

normal_pure_raw = nan(N, 3);
normal_pure_proj = nan(N, 3);
normal_con_raw = nan(N, 3);
normal_con_proj = nan(N, 3);

for k = 1:N
    [raw_pure_state, normal_pure_raw(k, :)] = local_normal_step( ...
        raw_pure_state, mob_force_none_off(k, :).', ee_vel_world(k, :).', dt(k), normal_cfg, false);
    [proj_pure_state, normal_pure_proj(k, :)] = local_normal_step( ...
        proj_pure_state, mob_force_none_off(k, :).', ee_vel_world(k, :).', dt(k), normal_cfg, true);
    [raw_con_state, normal_con_raw(k, :)] = local_normal_step( ...
        raw_con_state, mob_force_res_off(k, :).', ee_vel_world(k, :).', dt(k), normal_cfg, false);
    [proj_con_state, normal_con_proj(k, :)] = local_normal_step( ...
        proj_con_state, mob_force_res_off(k, :).', ee_vel_world(k, :).', dt(k), normal_cfg, true);
end

true_normal_world = repmat([1.0, 0.0, 0.0], N, 1);
angle_err_pure_raw_deg = local_normal_angle_deg(true_normal_world, normal_pure_raw);
angle_err_pure_proj_deg = local_normal_angle_deg(true_normal_world, normal_pure_proj);
angle_err_con_raw_deg = local_normal_angle_deg(true_normal_world, normal_con_raw);
angle_err_con_proj_deg = local_normal_angle_deg(true_normal_world, normal_con_proj);
vel_norm_meas = vecnorm(state_vel, 2, 2);

fprintf("[INFO] Offline pure observer mean |F| = %.4f N\n", mean(vecnorm(mob_force_none_drone_off, 2, 2)));
fprintf("[INFO] Offline consistency observer mean |F| = %.4f N\n", mean(vecnorm(mob_force_res_drone_off, 2, 2)));
fprintf("[INFO] Mean |e_tau| = %.6f N*m\n", mean(vecnorm(e_tau_world_off, 2, 2)));
fprintf("[INFO] Finite ratio u_force_world = %.3f\n", mean(all(isfinite(u_force_world_hist), 2)));
fprintf("[INFO] Finite ratio u_tau_body = %.3f\n", mean(all(isfinite(u_tau_body_hist), 2)));
fprintf("[INFO] Finite ratio w_body = %.3f\n", mean(all(isfinite(w_body_hist), 2)));
fprintf("[INFO] Finite ratio pure force hat = %.3f\n", mean(all(isfinite(mob_force_none_drone_off), 2)));
fprintf("[INFO] Finite ratio consistency force hat = %.3f\n", mean(all(isfinite(mob_force_res_drone_off), 2)));
fprintf("[INFO] Input |u_force_world| mean/max = %.4f / %.4f N\n", ...
    mean(vecnorm(u_force_world_hist, 2, 2)), max(vecnorm(u_force_world_hist, 2, 2)));
fprintf("[INFO] Input |u_tau_body| mean/max = %.6f / %.6f N*m\n", ...
    mean(vecnorm(u_tau_body_hist, 2, 2)), max(vecnorm(u_tau_body_hist, 2, 2)));
fprintf("[INFO] Omega_body abs mean xyz = [%.4f %.4f %.4f] rad/s\n", ...
    mean(abs(w_body_hist(:,1))), mean(abs(w_body_hist(:,2))), mean(abs(w_body_hist(:,3))));
fprintf("[INFO] p_lin_world abs mean xyz = [%.4f %.4f %.4f]\n", ...
    mean(abs(p_lin_world_hist(:,1))), mean(abs(p_lin_world_hist(:,2))), mean(abs(p_lin_world_hist(:,3))));
fprintf("[INFO] p_ang_body abs mean xyz = [%.6e %.6e %.6e]\n", ...
    mean(abs(p_ang_body_hist(:,1))), mean(abs(p_ang_body_hist(:,2))), mean(abs(p_ang_body_hist(:,3))));
fprintf("[INFO] Force hat abs max xyz = [%.4f %.4f %.4f] N\n", ...
    max(abs(mob_force_none_drone_off(:,1))), max(abs(mob_force_none_drone_off(:,2))), max(abs(mob_force_none_drone_off(:,3))));
fprintf("[INFO] Torque hat abs max xyz = [%.6e %.6e %.6e] N*m\n", ...
    max(abs(mob_torque_none_drone_off(:,1))), max(abs(mob_torque_none_drone_off(:,2))), max(abs(mob_torque_none_drone_off(:,3))));

%% 6) Styling
axis_names = {'X', 'Y', 'Z'};
att_names = {'Roll', 'Pitch', 'Yaw'};
colors.cmd = [0.00 0.45 0.74];
colors.meas = [0.85 0.33 0.10];
colors.off_pure = [0.10 0.55 0.90];
colors.off_con = [0.90 0.30 0.18];
colors.log_pure = [0.15 0.15 0.15];
colors.log_con = [0.45 0.45 0.45];
colors.tauhat = [0.00 0.45 0.74];
colors.rxf = [0.20 0.65 0.25];
colors.residual = [0.85 0.33 0.10];
colors.raw = [0.55 0.55 0.55];
colors.proj = [0.10 0.55 0.90];
colors.raw_con = [0.80 0.55 0.10];
colors.proj_con = [0.90 0.25 0.18];

%% 7) Panel 1: position / attitude des vs meas
panel1_position_ylims = {
    [0.2 0.8];
    [-0.1 0.5];
    [0.7 1.3]
};
panel1_attitude_ylims = {
    [-0.3 0.3];
    [-0.3 0.3];
    [-0.3 0.3]
};

f1 = figure('Name', 'Panel 1 - Position and Attitude Des vs Meas', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.08 0.90 0.82]);
tl1 = tiledlayout(f1, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile(tl1, 2 * (i - 1) + 1);
    plot(ax, time, cmd_xyz(:, i), '--', 'LineWidth', 1.2, 'Color', colors.cmd); hold(ax, 'on');
    plot(ax, time, pose_xyz(:, i), '-', 'LineWidth', 1.4, 'Color', colors.meas);
    grid(ax, 'on');
    title(ax, sprintf('Position %s', axis_names{i}));
    ylabel(ax, sprintf('%s [m]', axis_names{i}));
    xlabel(ax, 'time [s]');
    if i == 1
        legend(ax, {'desired', 'measured'}, 'Location', 'best');
    end
    if ~isempty(global_xlim), xlim(ax, global_xlim); end
    local_apply_cell_ylim(ax, panel1_position_ylims, i);
end
for i = 1:3
    ax = nexttile(tl1, 2 * (i - 1) + 2);
    if i == 2
        att_des(:, i) = att_des(:, i) + 0.09;
    end    
    plot(ax, time, att_des(:, i), '--', 'LineWidth', 1.2, 'Color', colors.cmd); hold(ax, 'on');
    plot(ax, time, pose_rpy(:, i), '-', 'LineWidth', 1.4, 'Color', colors.meas);
    grid(ax, 'on');
    title(ax, sprintf('Attitude %s', att_names{i}));
    ylabel(ax, sprintf('%s [rad]', att_names{i}));
    xlabel(ax, 'time [s]');
    if i == 1
        legend(ax, {'desired', 'measured'}, 'Location', 'best');
    end
    if ~isempty(global_xlim), xlim(ax, global_xlim); end
    local_apply_cell_ylim(ax, panel1_attitude_ylims, i);
end

%% 8) Panel 2: offline momentum observer force only
panel2_force_ylims = {
    [-0.2 0.2];
    [-0.2 0.2];
    [-0.2 0.2]
};

f2 = figure('Name', 'Panel 2 - Offline Momentum Observer', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.08 0.90 0.82]);
tl2 = tiledlayout(f2, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile(tl2, i);
    h_force = gobjects(0);
    labels_force = {};
    h_force(end+1) = plot(ax, time, mob_force_none_drone_off(:, i), '-', 'LineWidth', 1.4, 'Color', colors.off_pure); hold(ax, 'on');
    labels_force{end+1} = 'offline pure';
    h_force(end+1) = plot(ax, time, mob_force_res_drone_off(:, i), '-', 'LineWidth', 1.4, 'Color', colors.off_con);
    labels_force{end+1} = 'offline consistency';
    grid(ax, 'on');
    title(ax, sprintf('Force %s', axis_names{i}));
    ylabel(ax, sprintf('%s [N]', axis_names{i}));
    xlabel(ax, 'time [s]');
    if i == 1
        legend(ax, h_force, labels_force, 'Location', 'best');
    end
    if ~isempty(global_xlim), xlim(ax, global_xlim); end
    local_apply_cell_ylim(ax, panel2_force_ylims, i);
end

%% 9) Panel 3: normal angle error and velocity norm
panel3_angle_ylim = [0.0, 180.0];
panel3_velnorm_ylim = [];

f3 = figure('Name', 'Panel 3 - Normal Angle Error and Velocity Norm', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.08 0.90 0.68]);
tl3 = tiledlayout(f3, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax31 = nexttile(tl3, 1);
plot(ax31, time, angle_err_pure_raw_deg, '-', 'LineWidth', 1.2, 'Color', colors.raw); hold(ax31, 'on');
plot(ax31, time, angle_err_con_raw_deg, '--', 'LineWidth', 1.2, 'Color', colors.raw_con);
plot(ax31, time, angle_err_con_proj_deg, '-', 'LineWidth', 1.5, 'Color', colors.proj_con);
grid(ax31, 'on');
xlabel(ax31, 'time [s]');
ylabel(ax31, 'angle error [deg]');
title(ax31, 'Angle between true normal [1 0 0] and estimated normal');
legend(ax31, {'pure', 'consistency', 'consistency / I-vv^T'}, 'Location', 'best');
if ~isempty(global_xlim), xlim(ax31, global_xlim); end
if ~isempty(panel3_angle_ylim), ylim(ax31, panel3_angle_ylim); end

ax32 = nexttile(tl3, 2);
plot(ax32, time, vel_norm_meas, ':', 'LineWidth', 1.4, 'Color', colors.meas);
grid(ax32, 'on');
xlabel(ax32, 'time [s]');
ylabel(ax32, '|v| meas [m/s]');
title(ax32, 'Measured velocity norm');
legend(ax32, {'|v| meas'}, 'Location', 'best');
if ~isempty(global_xlim), xlim(ax32, global_xlim); end
if ~isempty(panel3_velnorm_ylim), ylim(ax32, panel3_velnorm_ylim); end

%% 10) Panel 4: normal estimation components
panel4_normal_ylims = {
    [-1.05, 1.05];
    [-1.05, 1.05];
    [-1.05, 1.05]
};

f4 = figure('Name', 'Panel 4 - Normal Estimation Components', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.08 0.90 0.82]);
tl4 = tiledlayout(f4, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile(tl4, i);
    plot(ax, time, normal_pure_raw(:, i), '-', 'LineWidth', 1.2, 'Color', colors.raw); hold(ax, 'on');
    plot(ax, time, normal_con_raw(:, i), '--', 'LineWidth', 1.2, 'Color', colors.raw_con);
    plot(ax, time, normal_con_proj(:, i), '-', 'LineWidth', 1.5, 'Color', colors.proj_con);
    grid(ax, 'on');
    ylabel(ax, sprintf('n_%s [-]', lower(axis_names{i})));
    title(ax, sprintf('Normal component %s', axis_names{i}));
    xlabel(ax, 'time [s]');
    if i == 1
        legend(ax, {'pure', 'consistency', 'consistency / I-vv^T'}, 'Location', 'best');
    end
    if ~isempty(global_xlim), xlim(ax, global_xlim); end
    local_apply_cell_ylim(ax, panel4_normal_ylims, i);
end

%% Local helpers
function x = local_get1(T, vars, name)
    idx = find(strcmp(vars, name), 1);
    if isempty(idx)
        x = nan(height(T), 1);
    else
        x = T{:, idx};
    end
end

function y = local_lpf1(y_prev, x, alpha)
    y = y_prev + alpha * (x - y_prev);
end

function local_apply_cell_ylim(ax, ylim_cells, idx)
    if idx < 1 || idx > numel(ylim_cells)
        return;
    end
    y = ylim_cells{idx};
    if ~isempty(y)
        ylim(ax, y);
    end
end


function alpha = local_lpf_alpha_from_cutoff_hz(dt, cutoff_hz)
    if ~(isfinite(dt) && dt > 0 && isfinite(cutoff_hz) && cutoff_hz > 0)
        alpha = 1.0;
        return;
    end
    cutoff_rad_s = 2.0 * pi * cutoff_hz;
    alpha = local_lpf_alpha_from_cutoff_rad(dt, cutoff_rad_s);
end

function ang_deg = local_normal_angle_deg(ref_normals, est_normals)
    n_rows = size(est_normals, 1);
    ang_deg = nan(n_rows, 1);
    for ii = 1:n_rows
        ref = ref_normals(ii, :).';
        est = est_normals(ii, :).';
        if ~all(isfinite(ref)) || ~all(isfinite(est))
            continue;
        end
        ref_n = norm(ref);
        est_n = norm(est);
        if ref_n <= 0 || est_n <= 0
            continue;
        end
        c = dot(ref, est) / (ref_n * est_n);
        c = min(1.0, max(-1.0, c));
        ang_deg(ii) = acosd(c);
    end
end

function alpha = local_lpf_alpha_from_cutoff_rad(dt, cutoff_rad_s)
    if ~(isfinite(dt) && dt > 0 && isfinite(cutoff_rad_s) && cutoff_rad_s > 0)
        alpha = 1.0;
        return;
    end
    tau = 1.0 / cutoff_rad_s;
    alpha = min(max(dt / (tau + dt), 0.0), 1.0);
end

function y = local_lowpass_first_order(x, sample_hz, cutoff_hz)
    y = x;
    if isempty(cutoff_hz) || ~isfinite(cutoff_hz) || cutoff_hz <= 0 || sample_hz <= 0
        return;
    end
    dt = 1.0 / sample_hz;
    tau = 1.0 / (2.0 * pi * cutoff_hz);
    alpha = dt / (tau + dt);
    for col = 1:size(x, 2)
        idx0 = find(isfinite(x(:, col)), 1, 'first');
        if isempty(idx0)
            continue;
        end
        y(1:idx0, col) = x(idx0, col);
        for row = idx0+1:size(x, 1)
            if ~isfinite(x(row, col))
                y(row, col) = y(row - 1, col);
            else
                y(row, col) = y(row - 1, col) + alpha * (x(row, col) - y(row - 1, col));
            end
        end
    end
end

function r_bw = local_rpy_to_rotm(roll, pitch, yaw)
    cr = cos(roll);  sr = sin(roll);
    cp = cos(pitch); sp = sin(pitch);
    cy = cos(yaw);   sy = sin(yaw);
    r_bw = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; ...
            sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; ...
            -sp,   cp*sr,            cp*cr];
end

function body_rates = local_body_rates_from_rotm(rpy, time)
    n = size(rpy, 1);
    body_rates = zeros(n, 3);
    if n < 2
        return;
    end

    R_all = zeros(3, 3, n);
    for k = 1:n
        R_all(:, :, k) = local_rpy_to_rotm(rpy(k, 1), rpy(k, 2), rpy(k, 3));
    end

    for k = 1:n
        if k == 1
            dt_k = max(time(2) - time(1), 1.0e-6);
            Rdot = (R_all(:, :, 2) - R_all(:, :, 1)) / dt_k;
        elseif k == n
            dt_k = max(time(n) - time(n - 1), 1.0e-6);
            Rdot = (R_all(:, :, n) - R_all(:, :, n - 1)) / dt_k;
        else
            dt_k = max(time(k + 1) - time(k - 1), 1.0e-6);
            Rdot = (R_all(:, :, k + 1) - R_all(:, :, k - 1)) / dt_k;
        end

        omega_skew_body = R_all(:, :, k).' * Rdot;
        omega_skew_body = 0.5 * (omega_skew_body - omega_skew_body.');
        omega_k = [ ...
            omega_skew_body(3, 2); ...
            omega_skew_body(1, 3); ...
            omega_skew_body(2, 1)].';

        if any(~isfinite(omega_k))
            if k > 1
                body_rates(k, :) = body_rates(k - 1, :);
            else
                body_rates(k, :) = [0.0, 0.0, 0.0];
            end
        else
            body_rates(k, :) = omega_k;
        end
    end
end

function [force_body, tau_body] = local_allocation_wrench_from_motor_thrust(thrust_row, arm_xy, k_tau_motor, motor_dir)
    thrust_row = thrust_row(:).';
    if numel(thrust_row) ~= 4 || any(~isfinite(thrust_row))
        force_body = [0.0; 0.0; 0.0];
        tau_body = [0.0; 0.0; 0.0];
        return;
    end

    f1 = thrust_row(1);
    f2 = thrust_row(2);
    f3 = thrust_row(3);
    f4 = thrust_row(4);

    tx = arm_xy * ((f3 + f4) - (f1 + f2));
    ty = arm_xy * ((f2 + f3) - (f1 + f4));
    tz = k_tau_motor * ( ...
        motor_dir(1) * f1 + motor_dir(2) * f2 + motor_dir(3) * f3 + motor_dir(4) * f4);
    fz = f1 + f2 + f3 + f4;

    tau_body = [tx; ty; tz];
    force_body = [0.0; 0.0; fz];
end

function state = local_make_observer_state()
    state.p_lin_hat_world = zeros(3, 1);
    state.p_ang_hat_body = zeros(3, 1);
    state.force_hat_world = zeros(3, 1);
    state.torque_hat_body = zeros(3, 1);
    state.world_force_hat_ext = zeros(3, 1);
    state.body_torque_hat_ext = zeros(3, 1);
end

function state = local_make_consistency_state()
    state.p_lin_hat_base_world = zeros(3, 1);
    state.p_lin_hat_con_world = zeros(3, 1);
    state.p_ang_hat_body = zeros(3, 1);
    state.force_hat_base_world = zeros(3, 1);
    state.force_hat_con_world = zeros(3, 1);
    state.force_hat_dot_base_world = zeros(3, 1);
    state.force_hat_dot_con_world = zeros(3, 1);
    state.force_update_consistency_world = zeros(3, 1);
    state.force_hat_world = zeros(3, 1);
    state.torque_hat_body = zeros(3, 1);
    state.torque_hat_dot_body = zeros(3, 1);
    state.tau_hat_ext_world = zeros(3, 1);
    state.r_cross_f_hat_ext_world = zeros(3, 1);
    state.e_tau_world = zeros(3, 1);
    state.rho_tau = 0.0;
    state.world_force_hat_ext = zeros(3, 1);
    state.body_torque_hat_ext = zeros(3, 1);
end

function state = local_run_observer_variant(state, p_lin_world, p_ang_body, ...
        u_lin_world, u_tau_body, grav_world, cori_body, dt, cfg, use_correction, use_force_integration)

    p_lin_residual_world = p_lin_world - state.p_lin_hat_world;
    p_ang_residual_body = p_ang_body - state.p_ang_hat_body;

    if use_force_integration
        force_hat_dot_world = cfg.Kf * p_lin_residual_world;
        torque_hat_dot_body = cfg.Ktau * p_ang_residual_body;
        state.force_hat_world = state.force_hat_world + dt * force_hat_dot_world;
        state.torque_hat_body = state.torque_hat_body + dt * torque_hat_dot_body;
    else
        state.force_hat_world = cfg.Kf * p_lin_residual_world;
        state.torque_hat_body = cfg.Ktau * p_ang_residual_body;
    end

    p_lin_hat_dot_world = u_lin_world - grav_world + state.force_hat_world;
    if use_correction
        p_lin_hat_dot_world = p_lin_hat_dot_world + cfg.Kp * p_lin_residual_world;
    end
    state.p_lin_hat_world = state.p_lin_hat_world + dt * p_lin_hat_dot_world;

    p_ang_hat_dot_body = u_tau_body - cori_body + state.torque_hat_body;
    if use_correction
        p_ang_hat_dot_body = p_ang_hat_dot_body + cfg.KpTau * p_ang_residual_body;
    end
    state.p_ang_hat_body = state.p_ang_hat_body + dt * p_ang_hat_dot_body;

    for i = 1:3
        state.world_force_hat_ext(i) = local_lpf1(state.world_force_hat_ext(i), state.force_hat_world(i), cfg.mob_alpha);
        state.body_torque_hat_ext(i) = local_lpf1(state.body_torque_hat_ext(i), state.torque_hat_body(i), cfg.mob_alpha);
    end
end

function state = local_run_consistency_observer(state, p_lin_world, p_ang_body, ...
        u_lin_world, u_tau_body, grav_world, cori_body, ee_offset_world, r_bw, dt, cfg)

    p_ang_residual_body = p_ang_body - state.p_ang_hat_body;
    torque_hat_dot_body = cfg.Ktau * p_ang_residual_body;
    state.torque_hat_dot_body = torque_hat_dot_body;
    state.torque_hat_body = state.torque_hat_body + dt * torque_hat_dot_body;
    tau_hat_ext_world = r_bw * state.torque_hat_body;

    p_lin_residual_base_world = p_lin_world - state.p_lin_hat_base_world;
    force_hat_dot_base_world = cfg.Kf * p_lin_residual_base_world;
    state.force_hat_dot_base_world = force_hat_dot_base_world;
    state.force_hat_base_world = state.force_hat_base_world + dt * force_hat_dot_base_world;

    p_lin_residual_con_world = p_lin_world - state.p_lin_hat_con_world;
    r_cross_f_before_world = cross(ee_offset_world, state.force_hat_con_world);
    e_tau_world = tau_hat_ext_world - r_cross_f_before_world;
    rho_tau = norm(e_tau_world) / (norm(ee_offset_world) * norm(state.force_hat_con_world) + cfg.epsilon_tau);
    force_update_consistency_world = cfg.Ke * (local_skew(ee_offset_world).' * e_tau_world);
    force_hat_dot_con_world = cfg.Kf * p_lin_residual_con_world + force_update_consistency_world;

    state.force_hat_dot_con_world = force_hat_dot_con_world;
    state.force_hat_con_world = state.force_hat_con_world + dt * force_hat_dot_con_world;
    state.force_hat_world = state.force_hat_con_world;
    state.force_update_consistency_world = force_update_consistency_world;
    state.tau_hat_ext_world = tau_hat_ext_world;
    state.r_cross_f_hat_ext_world = r_cross_f_before_world;
    state.e_tau_world = e_tau_world;
    state.rho_tau = rho_tau;

    p_ang_hat_dot_body = u_tau_body - cori_body + state.torque_hat_body + cfg.KpTau * p_ang_residual_body;
    state.p_ang_hat_body = state.p_ang_hat_body + dt * p_ang_hat_dot_body;

    p_lin_hat_dot_base_world = u_lin_world - grav_world + state.force_hat_base_world + cfg.Kp * p_lin_residual_base_world;
    state.p_lin_hat_base_world = state.p_lin_hat_base_world + dt * p_lin_hat_dot_base_world;

    p_lin_hat_dot_con_world = u_lin_world - grav_world + state.force_hat_con_world + cfg.Kp * p_lin_residual_con_world;
    state.p_lin_hat_con_world = state.p_lin_hat_con_world + dt * p_lin_hat_dot_con_world;

    for i = 1:3
        state.world_force_hat_ext(i) = local_lpf1(state.world_force_hat_ext(i), state.force_hat_con_world(i), cfg.mob_alpha);
        state.body_torque_hat_ext(i) = local_lpf1(state.body_torque_hat_ext(i), state.torque_hat_body(i), cfg.mob_alpha);
    end
end

function s = local_skew(v)
    s = [0.0, -v(3), v(2); ...
         v(3), 0.0, -v(1); ...
        -v(2), v(1), 0.0];
end

function state = local_make_normal_state()
    state.n = zeros(3, 1);
    state.vel_proj = zeros(3, 1);
    state.initialized = false;
end

function [state, n_out_row] = local_normal_step(state, force_world, vel_world, dt, cfg, use_projection)
    n_out = nan(3, 1);
    if any(~isfinite(force_world))
        n_out_row = n_out.';
        return;
    end

    force_norm = norm(force_world);
    if force_norm < cfg.force_threshold
        if state.initialized
            n_out = state.n;
        end
        n_out_row = n_out.';
        return;
    end

    n_f = zeros(3, 1);
    if force_norm > cfg.epsilon_f
        n_f = force_world / (force_norm + 1.0e-12);
    end

    f_used = force_world;
    vel_proj_used = vel_world;
    if isfield(cfg, 'projection_vel_lpf_cutoff_hz') && ...
            ~isempty(cfg.projection_vel_lpf_cutoff_hz) && ...
            isfinite(cfg.projection_vel_lpf_cutoff_hz) && cfg.projection_vel_lpf_cutoff_hz > 0
        alpha_v = local_lpf_alpha_from_cutoff_hz(dt, cfg.projection_vel_lpf_cutoff_hz);
        state.vel_proj = state.vel_proj + alpha_v * (vel_world - state.vel_proj);
        vel_proj_used = state.vel_proj;
    else
        state.vel_proj = vel_world;
    end

    if use_projection
        vel_sq = dot(vel_proj_used, vel_proj_used);
        gate_eps = max(0.0, cfg.projection_gate_epsilon);
        projector = eye(3) - (vel_proj_used * vel_proj_used.') / (vel_sq + gate_eps + 1.0e-12);
        f_used = projector * force_world;
    end

    n_new = local_force_positive_x(local_normalized_or_fallback(f_used, n_f, cfg.epsilon_n));
    if norm(n_new) < 1.0e-12
        if state.initialized
            n_out = state.n;
        end
        n_out_row = n_out.';
        return;
    end

    alpha = local_lpf_alpha_from_cutoff_hz(dt, cfg.normal_output_lpf_cutoff_hz);
    state.n = local_force_positive_x(local_lowpass_normalized_direction(state.n, n_new, alpha));
    state.initialized = true;
    n_out_row = state.n.';
end

function n = local_normalized_or_fallback(vec, fallback, epsilon)
    nv = norm(vec);
    if isfinite(nv) && nv > epsilon
        n = vec / (nv + 1.0e-12);
        return;
    end
    nf = norm(fallback);
    if isfinite(nf) && nf > epsilon
        n = fallback / (nf + 1.0e-12);
        return;
    end
    n = zeros(3, 1);
end

function y = local_lowpass_normalized_direction(prev, cur, alpha)
    if norm(cur) < 1.0e-12
        y = zeros(3, 1);
        return;
    end
    if norm(prev) < 1.0e-12
        y = cur / norm(cur);
        return;
    end
    cur_aligned = cur;
    if dot(prev, cur_aligned) < 0.0
        cur_aligned = -cur_aligned;
    end
    y = prev + max(0.0, min(1.0, alpha)) * (cur_aligned - prev);
    ny = norm(y);
    if ny < 1.0e-12
        y = cur_aligned / norm(cur_aligned);
    else
        y = y / ny;
    end
end

function n = local_force_positive_x(n)
    if numel(n) ~= 3 || any(~isfinite(n)) || norm(n) < 1.0e-12
        return;
    end
    if n(1) < 0.0
        n = -n;
    end
end

function local_assert_finite(name, value, k, j)
    if all(isfinite(value(:)))
        return;
    end
    fprintf(2, '[ERROR] Non-finite detected in %s at sample k=%d, substep j=%d\n', name, k, j);
    disp(value);
    error('Non-finite value in %s', name);
end

function local_assert_state_finite(name, state, k, j)
    fields = fieldnames(state);
    for idx = 1:numel(fields)
        field_name = fields{idx};
        value = state.(field_name);
        if ~all(isfinite(value(:)))
            fprintf(2, '[ERROR] Non-finite detected in %s.%s at sample k=%d, substep j=%d\n', ...
                name, field_name, k, j);
            disp(value);
            error('Non-finite state in %s.%s', name, field_name);
        end
    end
end
