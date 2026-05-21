%% data_again.m
% Offline result dashboard for *_again.csv files
%
% Layout order (2 rows x 3 cols, column-major intent):
%   (1,1) Position
%   (2,1) Attitude
%   (1,2) MOB force vs MOB consistency force
%   (2,2) MOB torque
%   (1,3) r x MOB consistency force vs MOB tau
%   (2,3) r x MOB force vs MOB tau
%
% Each panel is arranged internally as 3x1 subplots for X/Y/Z (or Roll/Pitch/Yaw).

clear; close all; clc;

%% 0) Pick CSV
defaultDir = fullfile(getenv("HOME"), "hitl_ws", "src", "flying_pen", "bag", "0428_experient_again");
if ~isfolder(defaultDir), defaultDir = pwd; end

[file, path] = uigetfile(fullfile(defaultDir, "*_again.csv"), "Select offline result CSV");
if isequal(file, 0)
    disp("Canceled."); return;
end
csv_path = fullfile(path, file);
fprintf("[INFO] Reading: %s\n", csv_path);

wrench_yaml_path = fullfile(getenv("HOME"), "hitl_ws", "src", "log_player", "config", "wrench_observer.yaml");
normal_yaml_path = fullfile(getenv("HOME"), "hitl_ws", "src", "log_player", "config", "normal_vector_estimation.yaml");
mass_obs = local_read_yaml_scalar(wrench_yaml_path, "mass", 0.04);
gravity_obs = local_read_yaml_scalar(wrench_yaml_path, "g", 9.81);
normal_force_based_epsilon_g = 0.001;
gravity_force_offset = mass_obs * gravity_obs;
fprintf("[INFO] Wrench observer mass=%.6f kg, g=%.6f m/s^2, mg=%.6f N\n", ...
    mass_obs, gravity_obs, gravity_force_offset);
fprintf("[INFO] Normal force-based epsilon_g=%.6e\n", normal_force_based_epsilon_g);

%% 1) Read table robustly
opts = detectImportOptions(csv_path, 'Delimiter', ',');
for i = 1:numel(opts.VariableTypes)
    opts.VariableTypes{i} = 'double';
end
T = readtable(csv_path, opts);

if isempty(T) || height(T) < 5
    error("CSV has too few rows.");
end

vars = string(T.Properties.VariableNames);
fprintf("[INFO] Columns: %d\n", numel(vars));

time = T{:, "t_sec"};
if all(~isfinite(time))
    error("t_sec is all NaN.");
end
t0 = time(find(isfinite(time), 1, 'first'));
time = time - t0;

get1 = @(name) local_get1(T, vars, name);

%% 2) Load signals
pose_xyz = [get1("pose_x"), get1("pose_y"), get1("pose_z")];
pose_rpy = [get1("pose_roll"), get1("pose_pitch"), get1("pose_yaw")];
cmd_xyz = [get1("cmd_x"), get1("cmd_y"), get1("cmd_z")];
cmd_att = [get1("attDes_roll"), get1("attDes_pitch"), get1("attDes_yaw")];
cmd_att = cmd_att * pi / 180.0;
est_vel_world = [get1("est_vx"), get1("est_vy"), get1("est_vz")];
gyro_body = [get1("gyro_x"), get1("gyro_y"), get1("gyro_z")];
motor_force_raw = [get1("motor_f1"), get1("motor_f2"), get1("motor_f3"), get1("motor_f4")];
motor_force_scaled = [get1("motor_f1_scaled"), get1("motor_f2_scaled"), get1("motor_f3_scaled"), get1("motor_f4_scaled")];
battery_status = get1("status_battery_voltage");
battery_raw = get1("raw_battery_voltage");
battery_filt = get1("filt_battery_voltage");
rbf_world = [get1("droneWorldFx_scaled"), get1("droneWorldFy_scaled"), get1("droneWorldFz_scaled")];
rbf_world_neg = -rbf_world;

mob2_force = [get1("offline_mob2_fx"), get1("offline_mob2_fy"), get1("offline_mob2_fz")];
mob2_torque = [get1("offline_mob2_tx"), get1("offline_mob2_ty"), get1("offline_mob2_tz")];
mobc_force = [get1("offline_mobc_fx"), get1("offline_mobc_fy"), get1("offline_mobc_fz")];
mobc_torque = [get1("offline_mobc_tx"), get1("offline_mobc_ty"), get1("offline_mobc_tz")];
mobc2_force = [get1("offline_mobc2_fx"), get1("offline_mobc2_fy"), get1("offline_mobc2_fz")];
mobc2_torque = [get1("offline_mobc2_tx"), get1("offline_mobc2_ty"), get1("offline_mobc2_tz")];
normal_est_pure = [get1("offline_normal_pure_nx"), get1("offline_normal_pure_ny"), get1("offline_normal_pure_nz")];
normal_est_k1 = [get1("offline_normal_k1_nx"), get1("offline_normal_k1_ny"), get1("offline_normal_k1_nz")];
normal_est_k1_novcorr = [get1("offline_normal_k1_novcorr_nx"), get1("offline_normal_k1_novcorr_ny"), get1("offline_normal_k1_novcorr_nz")];
normal_k1_gamma_v = get1("offline_normal_k1_gamma_v");
normal_k1_ws = [get1("offline_normal_k1_ws_x"), get1("offline_normal_k1_ws_y"), get1("offline_normal_k1_ws_z")];
normal_k1_nf = [get1("offline_normal_k1_nf_x"), get1("offline_normal_k1_nf_y"), get1("offline_normal_k1_nf_z")];
normal_k1_fg = [get1("offline_normal_k1_fg_x"), get1("offline_normal_k1_fg_y"), get1("offline_normal_k1_fg_z")];
normal_k1_nalg = [get1("offline_normal_k1_nalg_x"), get1("offline_normal_k1_nalg_y"), get1("offline_normal_k1_nalg_z")];
normal_est_k2 = [get1("offline_normal_k2_nx"), get1("offline_normal_k2_ny"), get1("offline_normal_k2_nz")];
normal_est_k2_nolpf = [get1("offline_normal_k2_nolpf_nx"), get1("offline_normal_k2_nolpf_ny"), get1("offline_normal_k2_nolpf_nz")];
normal_est_k2_novcorr = [get1("offline_normal_k2_novcorr_nx"), get1("offline_normal_k2_novcorr_ny"), get1("offline_normal_k2_novcorr_nz")];
tauhat = [get1("offline_tauhat_x"), get1("offline_tauhat_y"), get1("offline_tauhat_z")];
rxf_consistency = [get1("offline_rxf_x"), get1("offline_rxf_y"), get1("offline_rxf_z")];

%% 3) Cleanup rows
validTime = isfinite(time);
time = time(validTime);
pose_xyz = pose_xyz(validTime, :);
pose_rpy = pose_rpy(validTime, :);
cmd_xyz = cmd_xyz(validTime, :);
cmd_att = cmd_att(validTime, :);
est_vel_world = est_vel_world(validTime, :);
gyro_body = gyro_body(validTime, :);
motor_force_raw = motor_force_raw(validTime, :);
motor_force_scaled = motor_force_scaled(validTime, :);
battery_status = battery_status(validTime, :);
battery_raw = battery_raw(validTime, :);
battery_filt = battery_filt(validTime, :);
rbf_world = rbf_world(validTime, :);
rbf_world_neg = rbf_world_neg(validTime, :);
mob2_force = mob2_force(validTime, :);
mob2_torque = mob2_torque(validTime, :);
mobc_force = mobc_force(validTime, :);
mobc_torque = mobc_torque(validTime, :);
mobc2_force = mobc2_force(validTime, :);
mobc2_torque = mobc2_torque(validTime, :);
normal_est_pure = normal_est_pure(validTime, :);
normal_est_k1 = normal_est_k1(validTime, :);
normal_est_k1_novcorr = normal_est_k1_novcorr(validTime, :);
normal_k1_gamma_v = normal_k1_gamma_v(validTime, :);
normal_k1_ws = normal_k1_ws(validTime, :);
normal_k1_nf = normal_k1_nf(validTime, :);
normal_k1_fg = normal_k1_fg(validTime, :);
normal_k1_nalg = normal_k1_nalg(validTime, :);
normal_est_k2 = normal_est_k2(validTime, :);
normal_est_k2_nolpf = normal_est_k2_nolpf(validTime, :);
normal_est_k2_novcorr = normal_est_k2_novcorr(validTime, :);
tauhat = tauhat(validTime, :);
rxf_consistency = rxf_consistency(validTime, :);

N = numel(time);
fprintf("[INFO] Using %d rows.\n", N);

%% 4) Derived quantities
set(groot,'defaultFigureRenderer','painters');
ee_offset_body = [0.1; 0.0; 0.04];
position_numerical_velocity_lpf_cutoff_hz = 0.5;
% Projection input velocity is generated here from position by numerical differentiation.
position_numerical_velocity_world = differentiate_triplet_by_time(time, pose_xyz);
position_numerical_velocity_world = lowpass_triplet_by_time( ...
    time, position_numerical_velocity_world, position_numerical_velocity_lpf_cutoff_hz);
rxf_mob2 = nan(N, 3);
rxf_mobc = nan(N, 3);
rxf_mobc2 = nan(N, 3);
ee_vel_proxy_world = nan(N, 3);
for i = 1:N
    r = pose_rpy(i, 1);
    p = pose_rpy(i, 2);
    y = pose_rpy(i, 3);
    if ~all(isfinite([r, p, y]))
        continue;
    end
    Rwb = rpy_to_rotm_zyx(r, p, y);
    r_world = Rwb * ee_offset_body;
    omega_body = gyro_body(i, :).';
    if all(isfinite(omega_body))
        omega_world = Rwb * omega_body;
    else
        omega_world = [nan; nan; nan];
    end
    if all(isfinite(est_vel_world(i, :))) && all(isfinite(omega_world))
        ee_vel_proxy_world(i, :) = est_vel_world(i, :) + cross(omega_world.', r_world.');
    end
    if all(isfinite(mob2_force(i, :)))
        rxf_mob2(i, :) = cross(r_world.', mob2_force(i, :));
    end
    if all(isfinite(mobc_force(i, :)))
        rxf_mobc(i, :) = cross(r_world.', mobc_force(i, :));
    end
    if all(isfinite(mobc2_force(i, :)))
        rxf_mobc2(i, :) = cross(r_world.', mobc2_force(i, :));
    end
end

velcorr_ws = normal_k1_ws;
velcorr_gamma_v = normal_k1_gamma_v;
% Use the visible no-velocity-projection normal as q_f for offline reprojection.
velcorr_force_ke1 = normal_est_k1_novcorr;
velcorr_force_g_ke1_logged = normal_k1_fg;
velcorr_force_g_ke1 = velcorr_force_g_ke1_logged;
velcorr_force_parallel_ke1_logged = velcorr_force_ke1 - velcorr_force_g_ke1_logged;
velcorr_force_parallel_ke1 = velcorr_force_parallel_ke1_logged;
velcorr_force_parallel_mag_ke1 = sqrt(sum(velcorr_force_parallel_ke1.^2, 2));
velcorr_force_g_norm_ke1 = sqrt(sum(velcorr_force_g_ke1.^2, 2));
velcorr_force_norm_ke1 = sqrt(sum(velcorr_force_ke1.^2, 2));
velcorr_force_dot_ws_ke1 = row_dot(velcorr_force_ke1, normal_k1_ws);
velcorr_force_angle_ws_deg_ke1 = angle_deg_between_rows(velcorr_force_ke1, normal_k1_ws);

att_pose = [pose_rpy(:, 1), -pose_rpy(:, 2), unwrap(pose_rpy(:, 3))];
att_cmd = [cmd_att(:, 1), cmd_att(:, 2), unwrap(cmd_att(:, 3))];
pos_names = {'X', 'Y', 'Z'};
att_names = {'Roll', 'Pitch', 'Yaw'};
axis_names = {'X', 'Y', 'Z'};

pure_color = [0.0000 0.4470 0.7410];
cons_color = [0.8500 0.3250 0.0980];
tau_color = [0.4660 0.6740 0.1880];
meas_color = [0.6350 0.0780 0.1840];
cmd_color = [0.0000 0.4470 0.7410];
rbf_color = [0.4940 0.1840 0.5560];
lw1 = 1.2;
lw2 = 1.15;

ylim_pos = [0.0 1.4; -0.7 0.7; 0.3 1.7];
ylim_att = [-0.3 0.3; -0.3 0.3; -0.3 0.3];
ylim_force = [-0.2 0.2; -0.2 0.2; -0.2 0.2];
ylim_torque_milli = 1.0e-2 * [-0.3 0.3; -0.3 0.3; -0.3 0.3];

%% 5) Figure
% Figure 5 axis configuration
% Set to [t_min t_max] to force x-limits, or [NaN NaN] to use auto/full range.
xlim_fig5 = [50 110];
% Rows correspond to X/Y/Z.
ylim_fig5_pos = ylim_pos;
ylim_fig5_att = ylim_att;
ylim_fig5_force = ylim_force;
ylim_fig5_torque_milli = ylim_torque_milli;

f = figure('Name', 'Offline Again Dashboard', 'NumberTitle', 'off', ...
           'Color', 'w', 'Units', 'normalized', 'Position', [0.04 0.06 0.92 0.84]);

left = 0.035; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.025; vgap = 0.045;
ncol = 3; nrow = 2;
w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;
getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% (1,1) Position
p11 = uipanel('Parent', f, 'Position', getPos(1,1), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_cmd_meas_triplet_panel(p11, time, cmd_xyz, pose_xyz, pos_names, ...
    'command', cmd_color, 'measured', meas_color, 'Position', '[m]', ylim_fig5_pos, xlim_fig5, 1.25, lw2);

% (2,1) Attitude
p21 = uipanel('Parent', f, 'Position', getPos(2,1), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_cmd_meas_triplet_panel(p21, time, att_cmd, att_pose, att_names, ...
    'command', cmd_color, 'measured', meas_color, 'Attitude', '[rad]', ylim_fig5_att, xlim_fig5, 1.25, lw2);

% (1,2) Pure MOB vs two consistency-MOB variants
p12 = uipanel('Parent', f, 'Position', getPos(1,2), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_three_triplet_panel(p12, time, mob2_force, mobc_force, mobc2_force, axis_names, ...
    'Pure MOB 2nd', pure_color, 'Consistency MOB Ke1', cons_color, 'Consistency MOB Ke2', rbf_color, ...
    'Pure MOB / Consistency MOB Ke1 / Consistency MOB Ke2', '[N]', lw1, lw2, lw2, ylim_fig5_force, xlim_fig5);

% (2,2) MOB torque
p22 = uipanel('Parent', f, 'Position', getPos(2,2), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p22, time, 1.0e3 * mob2_torque, 1.0e3 * mobc_torque, axis_names, ...
    'Pure MOB 2nd', pure_color, 'Consistency MOB 2nd', cons_color, ...
    'MOB Torque', '[N m] x10^{-3}', lw1, lw2, 1.0e3 * ylim_fig5_torque_milli, xlim_fig5);

% (1,3) r x MOB consistency force vs MOB tau
p13 = uipanel('Parent', f, 'Position', getPos(1,3), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p13, time, 1.0e3 * rxf_consistency, 1.0e3 * tauhat, axis_names, ...
    'r x MOB consistency force', cons_color, 'MOB tau', tau_color, ...
    'r x MOB Consistency Force vs MOB Tau', '[N m] x10^{-3}', lw1, lw2, 1.0e3 * ylim_fig5_torque_milli, xlim_fig5);

% (2,3) r x MOB force vs MOB tau
p23 = uipanel('Parent', f, 'Position', getPos(2,3), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p23, time, 1.0e3 * rxf_mob2, 1.0e3 * tauhat, axis_names, ...
    'r x MOB force', pure_color, 'MOB tau', tau_color, ...
    'r x MOB Force vs MOB Tau', '[N m] x10^{-3}', lw1, lw2, 1.0e3 * ylim_fig5_torque_milli, xlim_fig5);

%% 6) Force/Torque Comparison Figure
% Figure 6 axis configuration
% Set to [t_min t_max] to force x-limits, or [NaN NaN] to use auto/full range.
xlim_fig6 = [55 100];
% Rows correspond to X/Y/Z.
ylim_fig6_force = [-0.03 0.03; -0.03 0.03; -0.03 0.03];
ylim_fig6_torque_milli = 1.0e-2 * [-0.5 0.5; -0.8 0.2; -0.5 0.5];
rbf_world_neg_fig6 = rbf_world_neg;
rbf_world_neg_fig6(:, 3) = rbf_world_neg_fig6(:, 3) + gravity_force_offset;
true_normal_world = repmat([1, 0, 0], N, 1);
angle_true_pure_deg = angle_deg_between_rows(true_normal_world, normal_est_pure);
angle_true_k1_deg = angle_deg_between_rows(true_normal_world, normal_est_k1);
angle_true_k1_novcorr_deg = angle_deg_between_rows(true_normal_world, normal_est_k1_novcorr);
angle_true_k2_deg = angle_deg_between_rows(true_normal_world, normal_est_k2);
angle_true_k2_nolpf_deg = angle_deg_between_rows(true_normal_world, normal_est_k2_nolpf);
angle_true_k2_novcorr_deg = angle_deg_between_rows(true_normal_world, normal_est_k2_novcorr);
mob2_force_norm = sqrt(sum(mob2_force.^2, 2));
mobc_force_norm = sqrt(sum(mobc_force.^2, 2));
mobc2_force_norm = sqrt(sum(mobc2_force.^2, 2));
ylim_fig6_normal_angle_deg = [0 90];
ylim_fig6_normal_component = [-1.1 1.1; -1.1 1.1; -1.1 1.1];

f_cmp = figure('Name', 'Offline Force Torque Comparison', 'NumberTitle', 'off', ...
               'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.06 0.90 0.84]);

left2 = 0.04; right2 = 0.02; top2 = 0.05; bottom2 = 0.06;
hgap2 = 0.025; vgap2 = 0.05;
ncol2 = 3; nrow2 = 2;
w2 = (1-left2-right2-hgap2*(ncol2-1))/ncol2;
h2 = (1-top2-bottom2-vgap2*(nrow2-1))/nrow2;
getPos2 = @(row, col)[ ...
    left2 + (col-1)*(w2+hgap2), ...
    1 - top2 - row*h2 - (row-1)*vgap2, ...
    w2, h2];

% Top row: -R_B * F vs each MOB mode
p_cmp_11 = uipanel('Parent', f_cmp, 'Position', getPos2(1,1), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p_cmp_11, time, rbf_world_neg_fig6, mob2_force, axis_names, ...
    '-R_B * F', rbf_color, 'Pure MOB 2nd', pure_color, ...
    '-R_B * F vs Pure MOB', '[N]', lw1, lw2, ylim_fig6_force, xlim_fig6);

p_cmp_12 = uipanel('Parent', f_cmp, 'Position', getPos2(1,2), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p_cmp_12, time, rbf_world_neg_fig6, mobc_force, axis_names, ...
    '-R_B * F', rbf_color, 'Consistency MOB Ke', pure_color, ...
    '-R_B * F vs MOB Ke1', '[N]', lw1, lw2, ylim_fig6_force, xlim_fig6);

p_cmp_13 = uipanel('Parent', f_cmp, 'Position', getPos2(1,3), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p_cmp_13, time, rbf_world_neg_fig6, mobc2_force, axis_names, ...
    '-R_B * F', rbf_color, 'Consistency MOB Ke2', pure_color, ...
    '-R_B * F vs MOB Ke2', '[N]', lw1, lw2, ylim_fig6_force, xlim_fig6);

% Bottom row: r x MOB force vs MOB tau for each mode
p_cmp_21 = uipanel('Parent', f_cmp, 'Position', getPos2(2,1), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p_cmp_21, time, 1.0e3 * rxf_mob2, 1.0e3 * mob2_torque, axis_names, ...
    'r x Pure MOB force', cons_color, 'Pure MOB tau', tau_color, ...
    'r x Pure MOB Force vs Pure MOB Tau', '[N m] x10^{-3}', lw1, lw2, 1.0e3 * ylim_fig6_torque_milli, xlim_fig6);

p_cmp_22 = uipanel('Parent', f_cmp, 'Position', getPos2(2,2), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p_cmp_22, time, 1.0e3 * rxf_mobc, 1.0e3 * mobc_torque, axis_names, ...
    'r x MOB Ke1 force', cons_color, 'MOB Ke1 tau', tau_color, ...
    'r x MOB Ke1 Force vs MOB Ke1 Tau', '[N m] x10^{-3}', lw1, lw2, 1.0e3 * ylim_fig6_torque_milli, xlim_fig6);

p_cmp_23 = uipanel('Parent', f_cmp, 'Position', getPos2(2,3), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p_cmp_23, time, 1.0e3 * rxf_mobc2, 1.0e3 * mobc2_torque, axis_names, ...
    'r x MOB Ke2 force', cons_color, 'MOB Ke2 tau', tau_color, ...
    'r x MOB Ke2 Force vs MOB Ke2 Tau', '[N m] x10^{-3}', lw1, lw2, 1.0e3 * ylim_fig6_torque_milli, xlim_fig6);

apply_yaxis_display_scale(findall(f_cmp, 'Type', 'axes'), 0.1);

%% Normal estimation comparison figure
% Normal figure axis configuration
% Set to [t_min t_max] to force x-limits, or [NaN NaN] to use auto/full range.
xlim_fig_normal = xlim_fig6;
matlab_velproj_gain_fig_normal = 1.0;  % 0: no projection, 1: nominal, >1: stronger removal
matlab_epsilon_g_fig_normal = normal_force_based_epsilon_g;  % override here if needed
ylim_fig_normal_angle_deg = ylim_fig6_normal_angle_deg;
% Rows correspond to X/Y/Z.
ylim_fig_normal_component = ylim_fig6_normal_component;
normal_force_g_ke1_fig_normal = project_force_direction_by_velocity( ...
    normal_est_k1_novcorr, position_numerical_velocity_world, matlab_epsilon_g_fig_normal, matlab_velproj_gain_fig_normal);
normal_est_k1_matlab_proj_fig_normal = safe_normalize_rows_with_fallback(normal_force_g_ke1_fig_normal, normal_est_k1_novcorr);
angle_true_k1_matlab_proj_fig_normal_deg = angle_deg_between_rows(true_normal_world, normal_est_k1_matlab_proj_fig_normal);

f_norm = figure('Name', 'Offline Normal Comparison with Velocity Projection', 'NumberTitle', 'off', ...
                'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.06 0.90 0.84]);

p_norm_11 = uipanel('Parent', f_norm, 'Position', getPos2(1,1), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_scalar_panel(p_norm_11, time, angle_true_pure_deg, pure_color, ...
    'acos(true normal \cdot pure est)', '[deg]', ylim_fig_normal_angle_deg, xlim_fig_normal);

p_norm_12 = uipanel('Parent', f_norm, 'Position', getPos2(1,2), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_scalar_panel(p_norm_12, time, angle_true_k1_matlab_proj_fig_normal_deg, cons_color, ...
    sprintf('Offline velocity projection angle, gain=%.2f, \\epsilon_g=%.1e', ...
    matlab_velproj_gain_fig_normal, matlab_epsilon_g_fig_normal), ...
    '[deg]', ylim_fig_normal_angle_deg, xlim_fig_normal);

p_norm_13 = uipanel('Parent', f_norm, 'Position', getPos2(1,3), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_scalar_panel(p_norm_13, time, angle_true_k1_novcorr_deg, tau_color, ...
    'acos(true normal \cdot ke1 no vel proj est)', '[deg]', ylim_fig_normal_angle_deg, xlim_fig_normal);

p_norm_21 = uipanel('Parent', f_norm, 'Position', getPos2(2,1), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p_norm_21, time, normal_est_pure, normal_est_pure, axis_names, ...
    'Pure normal est', pure_color, 'Pure normal est', pure_color, ...
    'Pure vs Pure Normal Estimate', '[-]', lw1, lw2, ylim_fig_normal_component, xlim_fig_normal);

p_norm_22 = uipanel('Parent', f_norm, 'Position', getPos2(2,2), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p_norm_22, time, normal_est_pure, normal_est_k1_matlab_proj_fig_normal, axis_names, ...
    'Pure normal est', pure_color, sprintf('Ke1 offline vel projection est (g=%.2f, eps=%.1e)', ...
    matlab_velproj_gain_fig_normal, matlab_epsilon_g_fig_normal), cons_color, ...
    'Pure vs Ke1 Offline Velocity Projection Estimate', '[-]', lw1, lw2, ylim_fig_normal_component, xlim_fig_normal);

p_norm_23 = uipanel('Parent', f_norm, 'Position', getPos2(2,3), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p_norm_23, time, normal_est_pure, normal_est_k1_novcorr, axis_names, ...
    'Pure normal est', pure_color, 'Ke1 no vel proj est', tau_color, ...
    'Pure vs Ke1 No Vel Proj Normal Estimate', '[-]', lw1, lw2, ylim_fig_normal_component, xlim_fig_normal);

%% Normal sensitivity comparison figure
% Normal sensitivity figure axis configuration
% Set to [t_min t_max] to force x-limits, or [NaN NaN] to use auto/full range.
xlim_fig_normal_sens = [55 100];
matlab_velproj_gain_fig_normal_sens = 1.0;  % 0: no projection, 1: logged nominal, >1: stronger removal
matlab_epsilon_g_fig_normal_sens = 0.00001;  % override here if needed

% (1,1) Normal angle by consistency gain
ylim_fig_normal_sens_11_angle_deg = [0 90];

% (1,2) MOB force norm
ylim_fig_normal_sens_12_force_norm = [0 0.2];

% (1,3) Position
% Rows correspond to X/Y/Z.
ylim_fig_normal_sens_13_pos = [0.5 1.1; -0.3 0.3; 0.8 1.4];

% (2,1) Normal components by consistency gain
% Rows correspond to X/Y/Z.
ylim_fig_normal_sens_21_component = [0.0 2.0; -1.0 1.0; -1.0 1.0];

% (2,2) MOB force components
% Rows correspond to X/Y/Z.
ylim_fig_normal_sens_22_force = [-0.05 0.05 ; -0.05 0.05; -0.05 0.05];

% (2,3) Attitude
% Rows correspond to Roll/Pitch/Yaw.
att_cmd_lpf_cutoff_hz_fig_normal_sens_23 = 1.0;
ylim_fig_normal_sens_23_att = [-0.1 0.1; -0.1 0.1; -0.2 0.2];

att_cmd_fig_normal_sens_23 = lowpass_triplet_by_time(time, att_cmd, att_cmd_lpf_cutoff_hz_fig_normal_sens_23);
velcorr_force_g_ke1 = project_force_direction_by_velocity( ...
    velcorr_force_ke1, position_numerical_velocity_world, matlab_epsilon_g_fig_normal_sens, matlab_velproj_gain_fig_normal_sens);
velcorr_force_parallel_ke1 = velcorr_force_ke1 - velcorr_force_g_ke1;
velcorr_force_parallel_mag_ke1 = sqrt(sum(velcorr_force_parallel_ke1.^2, 2));
velcorr_force_novproj_norm_ke1 = sqrt(sum(normal_est_k1_novcorr.^2, 2));
velcorr_force_g_norm_ke1 = sqrt(sum(velcorr_force_g_ke1.^2, 2));
normal_est_k1_matlab_proj = safe_normalize_rows_with_fallback(velcorr_force_g_ke1, velcorr_force_ke1);
angle_true_k1_matlab_proj_deg = angle_deg_between_rows(true_normal_world, normal_est_k1_matlab_proj);

f_norm_sens = figure('Name', 'Offline Normal Sensitivity Comparison with Velocity Projection', 'NumberTitle', 'off', ...
                     'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.06 0.90 0.84]);

p_norm_sens_11 = uipanel('Parent', f_norm_sens, 'Position', getPos2(1,1), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_three_scalar_panel_solid(p_norm_sens_11, time, angle_true_pure_deg, angle_true_k1_novcorr_deg, angle_true_k1_matlab_proj_deg, ...
    'Pure', pure_color, 'Ke1 no vel proj', cons_color, sprintf('Offline vel projection (g=%.2f, eps=%.1e)', ...
    matlab_velproj_gain_fig_normal_sens, matlab_epsilon_g_fig_normal_sens), tau_color, ...
    'Normal Angle Comparison', '[deg]', ylim_fig_normal_sens_11_angle_deg, xlim_fig_normal_sens);

p_norm_sens_12 = uipanel('Parent', f_norm_sens, 'Position', getPos2(1,2), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_four_scalar_panel_solid(p_norm_sens_12, time, mob2_force_norm, mobc_force_norm, velcorr_force_novproj_norm_ke1, velcorr_force_g_norm_ke1, ...
    'Pure MOB', pure_color, 'Ke1 raw force', cons_color, 'Ke1 no vel proj', meas_color, ...
    'Ke1 vel-projected force', tau_color, ...
    'MOB Force Norm Comparison', '[N]', ylim_fig_normal_sens_12_force_norm, xlim_fig_normal_sens);

p_norm_sens_13 = uipanel('Parent', f_norm_sens, 'Position', getPos2(1,3), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_cmd_meas_triplet_panel(p_norm_sens_13, time, cmd_xyz, pose_xyz, pos_names, ...
    'command', cmd_color, 'measured', meas_color, 'Position', '[m]', ylim_fig_normal_sens_13_pos, xlim_fig_normal_sens, 1.25, lw2);

p_norm_sens_21 = uipanel('Parent', f_norm_sens, 'Position', getPos2(2,1), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_three_triplet_panel_solid(p_norm_sens_21, time, normal_est_pure, normal_est_k1_novcorr, normal_est_k1_matlab_proj, axis_names, ...
    'Pure normal est', pure_color, 'Ke1 no vel proj est', cons_color, sprintf('Offline vel projection est (g=%.2f, eps=%.1e)', ...
    matlab_velproj_gain_fig_normal_sens, matlab_epsilon_g_fig_normal_sens), tau_color, ...
    'Normal Components Comparison with Offline Velocity Projection', '[-]', lw1, lw2, lw2, ylim_fig_normal_sens_21_component, xlim_fig_normal_sens);

p_norm_sens_22 = uipanel('Parent', f_norm_sens, 'Position', getPos2(2,2), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_three_triplet_panel_solid(p_norm_sens_22, time, normal_est_pure, velcorr_force_ke1, velcorr_force_g_ke1, axis_names, ...
    'Pure normal est', pure_color, 'Ke1 q_f', cons_color, sprintf('Ke1 q_g offline projection (g=%.2f, eps=%.1e)', ...
    matlab_velproj_gain_fig_normal_sens, matlab_epsilon_g_fig_normal_sens), tau_color, ...
    'Normal Direction / Offline Velocity Projection', '[-]', lw1, lw2, lw2, ylim_fig_normal_sens_21_component, xlim_fig_normal_sens);

p_norm_sens_23 = uipanel('Parent', f_norm_sens, 'Position', getPos2(2,3), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_cmd_meas_triplet_panel(p_norm_sens_23, time, att_cmd_fig_normal_sens_23, att_pose, att_names, ...
    'command', cmd_color, 'measured', meas_color, 'Attitude', '[rad]', ylim_fig_normal_sens_23_att, xlim_fig_normal_sens, 1.25, lw2);

%% Velocity correction debug figure
% Debug figure for k1 velocity projection reproduction.
xlim_fig_normal_debug = xlim_fig_normal;
ylim_fig_normal_debug_force = ylim_fig6_force;
ylim_fig_normal_debug_scalar = [0 1.2];
ylim_fig_normal_debug_vel = [-5 5; -5 5; -5 5];

f_norm_dbg = figure('Name', 'Offline Normal Velocity Correction Debug', 'NumberTitle', 'off', ...
                    'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.06 0.90 0.84]);

p_dbg_11 = uipanel('Parent', f_norm_dbg, 'Position', getPos2(1,1), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p_dbg_11, time, velcorr_force_ke1, velcorr_force_g_ke1, axis_names, ...
    'Ke1 q_f', tau_color, sprintf('velocity-projected q_g (MATLAB, g=%.2f, eps=%.1e)', ...
    matlab_velproj_gain_fig_normal_sens, matlab_epsilon_g_fig_normal_sens), cons_color, ...
    'Ke1 q_f vs q_g', '[-]', lw1, lw2, ylim_fig_normal_component, xlim_fig_normal_debug);

p_dbg_12 = uipanel('Parent', f_norm_dbg, 'Position', getPos2(1,2), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p_dbg_12, time, position_numerical_velocity_world, velcorr_ws, axis_names, ...
    'Numerically differentiated position velocity (world, 1 Hz LPF)', pure_color, 'w_s', meas_color, ...
    'Projection Velocity Input vs w_s', '[-]', lw1, lw2, ylim_fig_normal_debug_vel, xlim_fig_normal_debug);

p_dbg_13 = uipanel('Parent', f_norm_dbg, 'Position', getPos2(1,3), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_three_scalar_panel(p_dbg_13, time, velcorr_gamma_v, velcorr_force_parallel_mag_ke1, velcorr_force_g_norm_ke1, ...
    '\gamma_v', rbf_color, '|F_{\parallel}|', meas_color, '|f_g|', cons_color, ...
    'Velocity Correction Scalars', '[-] / [N]', ylim_fig_normal_debug_scalar, xlim_fig_normal_debug);

p_dbg_21 = uipanel('Parent', f_norm_dbg, 'Position', getPos2(2,1), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p_dbg_21, time, velcorr_force_parallel_ke1, velcorr_force_parallel_ke1_logged, axis_names, ...
    'q_f - q_g (MATLAB)', meas_color, 'logged residual', rbf_color, ...
    'Removed Velocity-Aligned Direction', '[-]', lw1, lw2, ylim_fig_normal_component, xlim_fig_normal_debug);

p_dbg_22 = uipanel('Parent', f_norm_dbg, 'Position', getPos2(2,2), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_panel(p_dbg_22, time, normal_est_k1, normal_est_k1_novcorr, axis_names, ...
    'Ke1 + vel proj', tau_color, 'Ke1 no vel proj', meas_color, ...
    'Current vs No Vel Corr Normal', '[-]', lw1, lw2, ylim_fig_normal_component, xlim_fig_normal_debug);

p_dbg_23 = uipanel('Parent', f_norm_dbg, 'Position', getPos2(2,3), 'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_scalar_panel(p_dbg_23, time, angle_true_k1_deg, angle_true_k1_novcorr_deg, ...
    'Ke1 + vel proj angle', tau_color, 'Ke1 no vel proj angle', meas_color, ...
    'Angle: Current vs No Vel Corr', '[deg]', ylim_fig_normal_angle_deg, xlim_fig_normal_debug);

f_norm_dbg2 = figure('Name', 'Offline Normal Velocity Alignment Debug', 'NumberTitle', 'off', ...
                     'Color', 'w', 'Units', 'normalized', 'Position', [0.08 0.08 0.72 0.42]);
tl_dbg2 = tiledlayout(f_norm_dbg2, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax_dbg2_1 = nexttile(tl_dbg2, 1);
plot(ax_dbg2_1, time, velcorr_force_dot_ws_ke1, '-', 'LineWidth', 1.2, 'Color', rbf_color);
grid(ax_dbg2_1, 'on');
title(ax_dbg2_1, 'dot(F, w_s) for Ke1');
ylabel(ax_dbg2_1, '[N]');
if numel(xlim_fig_normal_debug) == 2 && all(isfinite(xlim_fig_normal_debug))
    xlim(ax_dbg2_1, xlim_fig_normal_debug);
end

ax_dbg2_2 = nexttile(tl_dbg2, 2);
plot(ax_dbg2_2, time, velcorr_force_angle_ws_deg_ke1, '-', 'LineWidth', 1.2, 'Color', cons_color); hold(ax_dbg2_2, 'on');
yline(ax_dbg2_2, 90.0, ':', 'Color', [0.25 0.25 0.25]);
grid(ax_dbg2_2, 'on');
title(ax_dbg2_2, 'angle(F, w_s) for Ke1');
ylabel(ax_dbg2_2, '[deg]');
xlabel(ax_dbg2_2, 'time [s]');
if numel(xlim_fig_normal_debug) == 2 && all(isfinite(xlim_fig_normal_debug))
    xlim(ax_dbg2_2, xlim_fig_normal_debug);
end

%% Local functions
function v = local_get1(T, vars, name)
if any(vars == string(name))
    v = T{:, char(name)};
else
    v = nan(height(T), 1);
end
end

function out = row_dot(A, B)
out = sum(A .* B, 2);
end

function vel = differentiate_triplet_by_time(time, pos)
vel = nan(size(pos));
if size(pos, 1) < 2
    return;
end

for j = 1:size(pos, 2)
    x = pos(:, j);
    for i = 1:size(pos, 1)
        if ~isfinite(x(i))
            continue;
        end
        if i == 1
            dt = time(i + 1) - time(i);
            if isfinite(dt) && dt > 0 && isfinite(x(i + 1))
                vel(i, j) = (x(i + 1) - x(i)) / dt;
            end
        elseif i == size(pos, 1)
            dt = time(i) - time(i - 1);
            if isfinite(dt) && dt > 0 && isfinite(x(i - 1))
                vel(i, j) = (x(i) - x(i - 1)) / dt;
            end
        else
            dt = time(i + 1) - time(i - 1);
            if isfinite(dt) && dt > 0 && isfinite(x(i - 1)) && isfinite(x(i + 1))
                vel(i, j) = (x(i + 1) - x(i - 1)) / dt;
            end
        end
    end
end
end

function q_g = project_force_direction_by_velocity(q_f_world, v_c_world, epsilon_g, projection_gain)
if nargin < 4 || ~isfinite(projection_gain)
    projection_gain = 1.0;
end
q_g = nan(size(q_f_world));
I3 = eye(3);
for i = 1:size(q_f_world, 1)
    qf_w_i = q_f_world(i, :);
    v_w_i = v_c_world(i, :);
    if ~all(isfinite(qf_w_i)) || ~all(isfinite(v_w_i))
        continue;
    end

    v_col = v_w_i(:);
    vel_norm_sq = dot(v_col, v_col);
    projector = I3 - projection_gain * (v_col * v_col.') / (vel_norm_sq + epsilon_g + 1.0e-12);
    q_g(i, :) = (projector * qf_w_i(:)).';
end
end

function Xn = safe_normalize_rows(X)
Xn = nan(size(X));
for i = 1:size(X, 1)
    xi = X(i, :);
    if ~all(isfinite(xi))
        continue;
    end
    nrm = norm(xi);
    if nrm > 1.0e-12
        Xn(i, :) = xi / nrm;
    end
end
end

function Xn = safe_normalize_rows_with_fallback(X, X_fallback)
Xn = nan(size(X));
for i = 1:size(X, 1)
    xi = X(i, :);
    if all(isfinite(xi))
        nrm = norm(xi);
        if nrm > 1.0e-12
            Xn(i, :) = xi / nrm;
            continue;
        end
    end

    if nargin >= 2 && i <= size(X_fallback, 1)
        xfb = X_fallback(i, :);
        if all(isfinite(xfb))
            nrm_fb = norm(xfb);
            if nrm_fb > 1.0e-12
                Xn(i, :) = xfb / nrm_fb;
            end
        end
    end
end
end

function value = local_read_yaml_scalar(yaml_path, key, default_value)
value = default_value;
if ~isfile(yaml_path)
    fprintf("[WARN] YAML not found: %s. Using default %.6f for %s\n", ...
        yaml_path, default_value, key);
    return;
end

yaml_text = fileread(yaml_path);
pattern = "(?m)^\s*" + regexptranslate('escape', key) + "\s*:\s*([-+]?[\d\.eE]+)";
tokens = regexp(yaml_text, pattern, 'tokens', 'once');
if isempty(tokens)
    fprintf("[WARN] Key '%s' not found in %s. Using default %.6f\n", ...
        key, yaml_path, default_value);
    return;
end

parsed_value = str2double(tokens{1});
if isfinite(parsed_value)
    value = parsed_value;
else
    fprintf("[WARN] Failed to parse key '%s' in %s. Using default %.6f\n", ...
        key, yaml_path, default_value);
end
end

function Rwb = rpy_to_rotm_zyx(roll, pitch, yaw)
cr = cos(roll); sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw); sy = sin(yaw);
Rwb = [ ...
    cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; ...
    sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; ...
    -sp,   cp*sr,            cp*cr];
end

function plot_cmd_meas_triplet_panel(parent_panel, time, cmd_data, meas_data, names, label_cmd, color_cmd, label_meas, color_meas, panel_title, unit_suffix, ylims, xlims, lw_cmd, lw_meas)
tl = tiledlayout(parent_panel, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_list = gobjects(0);
for i = 1:3
    ax = nexttile(tl, i); ax_list(end+1) = ax; %#ok<AGROW>
    h_cmd = plot(ax, time, cmd_data(:, i), '--', 'LineWidth', lw_cmd, 'Color', color_cmd); hold(ax, 'on');
    h_meas = plot(ax, time, meas_data(:, i), '-', 'LineWidth', lw_meas, 'Color', color_meas);
    grid(ax, 'on');
    ylabel(ax, sprintf('%s %s', names{i}, unit_suffix));
    if i == 1
        title(ax, panel_title);
        legend(ax, [h_cmd h_meas], label_cmd, label_meas, 'Location', 'best');
    end
    ylim(ax, ylims(i, :));
    if i < 3
        ax.XTickLabel = [];
    else
        xlabel(ax, 'time [s]');
    end
end
linkaxes(ax_list, 'x');
if numel(xlims) == 2 && all(isfinite(xlims))
    xlim(ax_list, xlims);
end
end

function plot_scalar_panel(parent_panel, time, data, color, panel_title, unit_suffix, ylims, xlims)
ax = axes(parent_panel);
plot(ax, time, data, '-', 'LineWidth', 1.25, 'Color', color);
grid(ax, 'on');
title(ax, panel_title);
ylabel(ax, unit_suffix);
xlabel(ax, 'time [s]');
if numel(ylims) == 2 && all(isfinite(ylims))
    ylim(ax, ylims);
end
if numel(xlims) == 2 && all(isfinite(xlims))
    xlim(ax, xlims);
end
end

function plot_compare_scalar_panel(parent_panel, time, data_a, data_b, label_a, color_a, label_b, color_b, panel_title, unit_suffix, ylims, xlims)
ax = axes(parent_panel);
h1 = plot(ax, time, data_a, '-', 'LineWidth', 1.25, 'Color', color_a); hold(ax, 'on');
h2 = plot(ax, time, data_b, '--', 'LineWidth', 1.15, 'Color', color_b);
grid(ax, 'on');
title(ax, panel_title);
ylabel(ax, unit_suffix);
xlabel(ax, 'time [s]');
legend(ax, [h1 h2], label_a, label_b, 'Location', 'best');
if numel(ylims) == 2 && all(isfinite(ylims))
    ylim(ax, ylims);
end
if numel(xlims) == 2 && all(isfinite(xlims))
    xlim(ax, xlims);
end
end

function plot_three_scalar_panel(parent_panel, time, data_a, data_b, data_c, label_a, color_a, label_b, color_b, label_c, color_c, panel_title, unit_suffix, ylims, xlims)
ax = axes(parent_panel);
h1 = plot(ax, time, data_a, '-', 'LineWidth', 1.25, 'Color', color_a); hold(ax, 'on');
h2 = plot(ax, time, data_b, '--', 'LineWidth', 1.15, 'Color', color_b);
h3 = plot(ax, time, data_c, ':', 'LineWidth', 1.15, 'Color', color_c);
grid(ax, 'on');
title(ax, panel_title);
ylabel(ax, unit_suffix);
xlabel(ax, 'time [s]');
legend(ax, [h1 h2 h3], label_a, label_b, label_c, 'Location', 'best');
if numel(ylims) == 2 && all(isfinite(ylims))
    ylim(ax, ylims);
end
if numel(xlims) == 2 && all(isfinite(xlims))
    xlim(ax, xlims);
end
end

function plot_three_scalar_panel_solid(parent_panel, time, data_a, data_b, data_c, label_a, color_a, label_b, color_b, label_c, color_c, panel_title, unit_suffix, ylims, xlims)
ax = axes(parent_panel);
h1 = plot(ax, time, data_a, '-', 'LineWidth', 1.25, 'Color', color_a); hold(ax, 'on');
h2 = plot(ax, time, data_b, '-', 'LineWidth', 1.15, 'Color', color_b);
h3 = plot(ax, time, data_c, '-', 'LineWidth', 1.15, 'Color', color_c);
grid(ax, 'on');
title(ax, panel_title);
ylabel(ax, unit_suffix);
xlabel(ax, 'time [s]');
legend(ax, [h1 h2 h3], label_a, label_b, label_c, 'Location', 'best');
if numel(ylims) == 2 && all(isfinite(ylims))
    ylim(ax, ylims);
end
if numel(xlims) == 2 && all(isfinite(xlims))
    xlim(ax, xlims);
end
end

function plot_four_scalar_panel_solid(parent_panel, time, data_a, data_b, data_c, data_d, label_a, color_a, label_b, color_b, label_c, color_c, label_d, color_d, panel_title, unit_suffix, ylims, xlims)
ax = axes(parent_panel);
h1 = plot(ax, time, data_a, '-', 'LineWidth', 1.25, 'Color', color_a); hold(ax, 'on');
h2 = plot(ax, time, data_b, '-', 'LineWidth', 1.15, 'Color', color_b);
h3 = plot(ax, time, data_c, '-', 'LineWidth', 1.15, 'Color', color_c);
h4 = plot(ax, time, data_d, '-', 'LineWidth', 1.15, 'Color', color_d);
grid(ax, 'on');
title(ax, panel_title);
ylabel(ax, unit_suffix);
xlabel(ax, 'time [s]');
legend(ax, [h1 h2 h3 h4], label_a, label_b, label_c, label_d, 'Location', 'best');
if numel(ylims) == 2 && all(isfinite(ylims))
    ylim(ax, ylims);
end
if numel(xlims) == 2 && all(isfinite(xlims))
    xlim(ax, xlims);
end
end

function plot_compare_triplet_panel(parent_panel, time, data_a, data_b, names, label_a, color_a, label_b, color_b, panel_title, unit_suffix, lw1, lw2, ylims, xlims)
tl = tiledlayout(parent_panel, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_list = gobjects(0);
for i = 1:3
    ax = nexttile(tl, i); ax_list(end+1) = ax; %#ok<AGROW>
    h1 = plot(ax, time, data_a(:, i), '-', 'LineWidth', lw1, 'Color', color_a); hold(ax, 'on');
    h2 = plot(ax, time, data_b(:, i), '--', 'LineWidth', lw2, 'Color', color_b);
    grid(ax, 'on');
    ylabel(ax, sprintf('%s %s', names{i}, unit_suffix));
    if i == 1
        title(ax, panel_title);
        legend(ax, [h1 h2], label_a, label_b, 'Location', 'best');
    end
    ylim(ax, ylims(i, :));
    if i < 3
        ax.XTickLabel = [];
    else
        xlabel(ax, 'time [s]');
    end
end
linkaxes(ax_list, 'x');
if numel(xlims) == 2 && all(isfinite(xlims))
    xlim(ax_list, xlims);
end
end

function plot_three_triplet_panel(parent_panel, time, data_a, data_b, data_c, names, label_a, color_a, label_b, color_b, label_c, color_c, panel_title, unit_suffix, lw1, lw2, lw3, ylims, xlims)
tl = tiledlayout(parent_panel, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_list = gobjects(0);
for i = 1:3
    ax = nexttile(tl, i); ax_list(end+1) = ax; %#ok<AGROW>
    h1 = plot(ax, time, data_a(:, i), '-', 'LineWidth', lw1, 'Color', color_a); hold(ax, 'on');
    h2 = plot(ax, time, data_b(:, i), '--', 'LineWidth', lw2, 'Color', color_b);
    h3 = plot(ax, time, data_c(:, i), ':', 'LineWidth', lw3, 'Color', color_c);
    grid(ax, 'on');
    ylabel(ax, sprintf('%s %s', names{i}, unit_suffix));
    if i == 1
        title(ax, panel_title);
        legend(ax, [h1 h2 h3], label_a, label_b, label_c, 'Location', 'best');
    end
    ylim(ax, ylims(i, :));
    if i < 3
        ax.XTickLabel = [];
    else
        xlabel(ax, 'time [s]');
    end
end
linkaxes(ax_list, 'x');
if numel(xlims) == 2 && all(isfinite(xlims))
    xlim(ax_list, xlims);
end
end

function plot_three_triplet_panel_solid(parent_panel, time, data_a, data_b, data_c, names, label_a, color_a, label_b, color_b, label_c, color_c, panel_title, unit_suffix, lw1, lw2, lw3, ylims, xlims)
tl = tiledlayout(parent_panel, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_list = gobjects(0);
for i = 1:3
    ax = nexttile(tl, i); ax_list(end+1) = ax; %#ok<AGROW>
    h1 = plot(ax, time, data_a(:, i), '-', 'LineWidth', lw1, 'Color', color_a); hold(ax, 'on');
    h2 = plot(ax, time, data_b(:, i), '-', 'LineWidth', lw2, 'Color', color_b);
    h3 = plot(ax, time, data_c(:, i), '-', 'LineWidth', lw3, 'Color', color_c);
    grid(ax, 'on');
    ylabel(ax, sprintf('%s %s', names{i}, unit_suffix));
    if i == 1
        title(ax, panel_title);
        legend(ax, [h1 h2 h3], label_a, label_b, label_c, 'Location', 'best');
    end
    ylim(ax, ylims(i, :));
    if i < 3
        ax.XTickLabel = [];
    else
        xlabel(ax, 'time [s]');
    end
end
linkaxes(ax_list, 'x');
if numel(xlims) == 2 && all(isfinite(xlims))
    xlim(ax_list, xlims);
end
end

function apply_yaxis_display_scale(ax_list, scale)
for i = 1:numel(ax_list)
    ax = ax_list(i);
    ticks = yticks(ax);
    scaled_labels = arrayfun(@(v) sprintf('%.3g', scale * v), ticks, 'UniformOutput', false);
    yticklabels(ax, scaled_labels);
end
end

function ang_deg = angle_deg_between_rows(a, b)
n = size(a, 1);
ang_deg = nan(n, 1);
for i = 1:n
    ai = a(i, :);
    bi = b(i, :);
    if ~all(isfinite(ai)) || ~all(isfinite(bi))
        continue;
    end
    na = norm(ai);
    nb = norm(bi);
    if na < 1e-12 || nb < 1e-12
        continue;
    end
    c = dot(ai, bi) / (na * nb);
    c = max(-1.0, min(1.0, c));
    ang_deg(i) = acos(c) * 180.0 / pi;
end
end

function data_filt = lowpass_triplet_by_time(time, data, cutoff_hz)
data_filt = data;
if ~(isfinite(cutoff_hz) && cutoff_hz > 0.0)
    return;
end

for i = 2:size(data, 1)
    dt = time(i) - time(i - 1);
    if ~(isfinite(dt) && dt > 0.0)
        data_filt(i, :) = data_filt(i - 1, :);
        continue;
    end
    tau = 1.0 / (2.0 * pi * cutoff_hz);
    alpha = max(0.0, min(1.0, dt / (tau + dt)));
    for j = 1:size(data, 2)
        x = data(i, j);
        y_prev = data_filt(i - 1, j);
        if ~isfinite(x)
            data_filt(i, j) = y_prev;
        elseif ~isfinite(y_prev)
            data_filt(i, j) = x;
        else
            data_filt(i, j) = y_prev + alpha * (x - y_prev);
        end
    end
end
end

function fit = compute_affine_force_fit(time, input_force, time_window, target_force)
fit = struct( ...
    'valid', false, ...
    'a', nan, ...
    'b', nan, ...
    'rmse', nan, ...
    'target_force', target_force, ...
    'time_window', time_window, ...
    'input_force', input_force, ...
    'scaled_force', nan(size(input_force)), ...
    'target_series', nan(size(input_force)));

if ~(numel(time_window) == 2 && all(isfinite(time_window)) && isfinite(target_force))
    return;
end

t0 = min(time_window);
t1 = max(time_window);
window_mask = time >= t0 & time <= t1 & isfinite(input_force);
if nnz(window_mask) < 2
    return;
end

x = input_force(window_mask);
y = target_force * ones(nnz(window_mask), 1);
theta = [x, ones(size(x))] \ y;
a = theta(1);
b = theta(2);

scaled_force = a * input_force + b;
target_series = target_force * ones(size(input_force));
rmse = sqrt(mean((scaled_force(window_mask) - target_series(window_mask)).^2, 'omitnan'));

fit.valid = true;
fit.a = a;
fit.b = b;
fit.rmse = rmse;
fit.time_window = [t0, t1];
fit.scaled_force = scaled_force;
fit.target_series = target_series;
end
