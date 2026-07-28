%% data_defense.m
% Defense-focused reader and plotting dashboard for suWrenchObs SI logging.
%
% Matches: data_logging_debug.cpp
% Columns:
%   pose_x,y,z, pose_roll,pitch,yaw,                  [m], [rad], world/body
%   cmd_x,cmd_y,cmd_z,cmd_yaw,                        [m], [rad], world
%   fwCmd_x,fwCmd_y,fwCmd_z,                          [m], [rad], firmware final position setpoint
%   f1,f2,f3,f4,                                     [N], per-motor thrust command
%   pwm1,pwm2,pwm3,pwm4,                             [0..65535], actuator command
%   bodyFx,bodyFy,bodyFz,                            [N], body frame
%   worldFx,worldFy,worldFz,                         [N], world frame
%   bodyTx,bodyTy,bodyTz,                            [N*m], body frame
%   stateVx,stateVy,stateVz,                         [m/s], world frame
%   posVx,posVy,posVz,                               [m/s], world frame
%   accWx,accWy,accWz,                               [m/s^2], world frame
%   velDes_vx,velDes_vy,velDes_vz,                   [m/s], desired velocity
%   attDes_roll,attDes_pitch,attDes_yaw,            [deg], desired attitude
%   status_battery_voltage,pm_vbat,                  [V], host status / firmware pm.vbat
%   zero_bias_count
%   mobForceNone_x,y,z, mobForceResidual_x,y,z,        [N], momentum-only / momentum+consistency
%   mobForceFinal_x,y,z                                 [N], final MOB force used by control after selection/bias compensation
%   mobTorque_x,y,z, mobResidual_x,y,z                 [N*m], observer torque / consistency residual
%   accRawBody_x,y,z                                   [G], body-frame accel after manual bias correction, before gravity-trim/LPF
%   gyroBody_x,y,z                                     [deg/s], body-frame gyro used by Mahony/complementary
%   forceDesired                                       [N], scalar preload force command from PositionControl
%   normalPre_x,y,z                                    [-], force-direction evidence before velocity projection
%   normalPost_x,y,z                                   [-], normal candidate after velocity projection
%   normalEst_x,y,z                                    [-], firmware normal estimator output in world frame
%   yawRef_deg                                         [deg], final referenceYawDeg written into setpoint->attitude.yaw
%
% Notes:
% - The debug CSV currently has no timestamp column, so time is reconstructed
%   from sample_hz below. Update sample_hz if your logger rate changes.
% - This script is meant to quickly validate whether the logged firmware units
%   look physically consistent in flight.
%
clear; close all; clc;
set(groot, 'defaultFigureRenderer', 'painters');

%% 0) User config
sample_hz = 50.0;      % data_logging_debug loop_hz default
mass_kg = 0.0425;      % Crazyflie 2.1 Brushless mass
gravity_ms2 = 9.81;
defaultDir = fullfile(getenv("HOME"), "hitl_ws", "src", "flying_pen", "bag", "logging");
if ~isfolder(defaultDir), defaultDir = pwd; end

summary_xlim = [10 400];         % e.g. [0 10]


%% 1) Pick CSV and read robustly
[file, path] = uigetfile(fullfile(defaultDir, "*.csv"), "Select debug logging CSV");
if isequal(file,0)
    disp("Canceled."); return;
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
fprintf("[INFO] Columns: %d\n", numel(vars));

if any(strcmp(vars, "t_sec"))
    time = T{:, "t_sec"};
    time = time - time(find(isfinite(time), 1, 'first'));
else
    time = (0:height(T)-1).' ./ sample_hz;
end

%% 2) Helpers
get1 = @(name) local_get1(T, vars, name);
mg_total = mass_kg * gravity_ms2;
mg_per_motor = mg_total / 4.0;

%% 3) Load signals
pose_xyz = [get1("pose_x"), get1("pose_y"), get1("pose_z")];
pose_rpy = [get1("pose_roll"), get1("pose_pitch"), unwrap(get1("pose_yaw"))];
cmd_xyz = [get1("cmd_x"), get1("cmd_y"), get1("cmd_z")];
cmd_yaw = get1("cmd_yaw");
fw_cmd_xyz = [get1("fwCmd_x"), get1("fwCmd_y"), get1("fwCmd_z")];

motor_thrust = [get1("f1"), get1("f2"), get1("f3"), get1("f4")];
motor_pwm = [get1("pwm1"), get1("pwm2"), get1("pwm3"), get1("pwm4")];
body_force = [get1("bodyFx"), get1("bodyFy"), get1("bodyFz")];
world_force = [get1("worldFx"), get1("worldFy"), get1("worldFz")];
body_torque = [get1("bodyTx"), get1("bodyTy"), get1("bodyTz")];
state_vel = [get1("stateVx"), get1("stateVy"), get1("stateVz")];
pos_vel = [get1("posVx"), get1("posVy"), get1("posVz")];
acc_world = [get1("accWx"), get1("accWy"), get1("accWz")];
vel_des = [get1("velDes_vx"), get1("velDes_vy"), get1("velDes_vz")];
att_des = deg2rad([get1("attDes_roll"), get1("attDes_pitch"), get1("attDes_yaw")]);
% Crazyflie legacy convention uses the opposite sign for desired pitch.
att_des(:,2) = -att_des(:,2);
yaw_ref = deg2rad(get1("yawRef_deg"));
batt_status = get1("status_battery_voltage");
batt_pm = get1("pm_vbat");
zero_bias_count = get1("zero_bias_count");
mob_force_none = [get1("mobForceNone_x"), get1("mobForceNone_y"), get1("mobForceNone_z")];
mob_force_residual = [get1("mobForceResidual_x"), get1("mobForceResidual_y"), get1("mobForceResidual_z")];
mob_force_final = [get1("mobForceFinal_x"), get1("mobForceFinal_y"), get1("mobForceFinal_z")];
mob_torque = [get1("mobTorque_x"), get1("mobTorque_y"), get1("mobTorque_z")];
mob_residual = [get1("mobResidual_x"), get1("mobResidual_y"), get1("mobResidual_z")];
acc_raw_body = [get1("accRawBody_x"), get1("accRawBody_y"), get1("accRawBody_z")];
gyro_body = [get1("gyroBody_x"), get1("gyroBody_y"), get1("gyroBody_z")];
normal_pre_logged = [get1("normalPre_x"), get1("normalPre_y"), get1("normalPre_z")];
normal_post_logged = [get1("normalPost_x"), get1("normalPost_y"), get1("normalPost_z")];
normal_est_logged = [get1("normalEst_x"), get1("normalEst_y"), get1("normalEst_z")];
ee_vel_used_world = [get1("eeVelUsed_x"), get1("eeVelUsed_y"), get1("eeVelUsed_z")];
omega_n_logged = get1("omega_n");
normal_velocity_leakage_logged = get1("normalVelocityLeakage");
stabilizer_loop_dt_us = get1("loopDtUs");
stabilizer_loop_dt_us_max = get1("loopDtUsMax");
alpha_frame_logged = get1("alphaFrame");
t1_cmd_des_logged = get1("t1CmdDes");
t2_cmd_des_logged = get1("t2CmdDes");
force_desired = get1("forceDesired");

valid = isfinite(time);
time = time(valid);
pose_xyz = pose_xyz(valid,:);
pose_rpy = pose_rpy(valid,:);
cmd_xyz = cmd_xyz(valid,:);
cmd_yaw = cmd_yaw(valid);
fw_cmd_xyz = fw_cmd_xyz(valid,:);
motor_thrust = motor_thrust(valid,:);
motor_pwm = motor_pwm(valid,:);
body_force = body_force(valid,:);
world_force = world_force(valid,:);
body_torque = body_torque(valid,:);
state_vel = state_vel(valid,:);
pos_vel = pos_vel(valid,:);
acc_world = acc_world(valid,:);
vel_des = vel_des(valid,:);
att_des = att_des(valid,:);
yaw_ref = yaw_ref(valid);
batt_status = batt_status(valid);
batt_pm = batt_pm(valid);
zero_bias_count = zero_bias_count(valid);
mob_force_none = mob_force_none(valid,:);
mob_force_residual = mob_force_residual(valid,:);
mob_force_final = mob_force_final(valid,:);
mob_torque = mob_torque(valid,:);
mob_residual = mob_residual(valid,:);
acc_raw_body = acc_raw_body(valid,:);
gyro_body = gyro_body(valid,:);
force_desired = force_desired(valid);
ee_vel_used_world = ee_vel_used_world(valid,:);
omega_n_logged = omega_n_logged(valid);
normal_velocity_leakage_logged = normal_velocity_leakage_logged(valid);
stabilizer_loop_dt_us = stabilizer_loop_dt_us(valid);
stabilizer_loop_dt_us_max = stabilizer_loop_dt_us_max(valid);
alpha_frame_logged = alpha_frame_logged(valid);
t1_cmd_des_logged = t1_cmd_des_logged(valid);
t2_cmd_des_logged = t2_cmd_des_logged(valid);

N = numel(time);
fprintf("[INFO] Using %d rows.\n", N);

%% 4) Derived signals
sum_thrust = sum(motor_thrust, 2, 'omitnan');
thrust_error_to_mg = sum_thrust - mg_total;
state_vel_norm = vecnorm(state_vel, 2, 2);
pos_vel_norm = vecnorm(pos_vel, 2, 2);
vel_des_norm = vecnorm(vel_des, 2, 2);
vel_diff = state_vel - pos_vel;
vel_diff_norm = vecnorm(vel_diff, 2, 2);
pos_error = pose_xyz - cmd_xyz;
fw_pos_error = pose_xyz - fw_cmd_xyz;
position_des_plot = fw_cmd_xyz;
for i = 1:3
    if ~any(isfinite(position_des_plot(:,i)))
        position_des_plot(:,i) = cmd_xyz(:,i);
    end
end
pos_error_norm = vecnorm(pos_error, 2, 2);
fw_pos_error_norm = vecnorm(fw_pos_error, 2, 2);
acc_norm = vecnorm(acc_world, 2, 2);
world_force_norm = vecnorm(world_force, 2, 2);
body_torque_norm = vecnorm(body_torque, 2, 2);
mob_force_none_norm = vecnorm(mob_force_none, 2, 2);
mob_force_residual_norm = vecnorm(mob_force_residual, 2, 2);
mob_force_final_norm = vecnorm(mob_force_final, 2, 2);
mob_torque_norm = vecnorm(mob_torque, 2, 2);
mob_residual_norm = vecnorm(mob_residual, 2, 2);

panelm1_acc_lpf_hz = [0.1];          % e.g. 8.0, [] or <=0 disables LPF
acc_raw_body_for_recon = local_lowpass_first_order(acc_raw_body, sample_hz, panelm1_acc_lpf_hz);

acc_from_raw_rpy = nan(size(acc_raw_body_for_recon));
acc_from_raw_rpy(:,1) = atan2(acc_raw_body_for_recon(:,2), acc_raw_body_for_recon(:,3));
acc_from_raw_rpy(:,2) = atan2(-acc_raw_body_for_recon(:,1), sqrt(acc_raw_body_for_recon(:,2).^2 + acc_raw_body_for_recon(:,3).^2));
acc_from_raw_rpy(:,3) = nan(size(time));

panelm1_gyro_lpf_hz = [0.1];         % e.g. 8.0, [] or <=0 disables LPF
gyro_body_for_recon = local_lowpass_first_order(gyro_body, sample_hz, panelm1_gyro_lpf_hz);
gyro_integrated_rpy = local_integrate_body_rates_to_rpy_deg(gyro_body_for_recon, time, pose_rpy(1,:));
gyro_integrated_rpy(:,3) = unwrap(gyro_integrated_rpy(:,3));

acc_normal_world = local_rpy_to_world_normal(acc_from_raw_rpy);
gyro_normal_world = local_rpy_to_world_normal(gyro_integrated_rpy);
pose_normal_world = local_rpy_to_world_normal(pose_rpy);
acc_normal_tilt_err = local_angle_between_unit_vectors(acc_normal_world, pose_normal_world);
gyro_normal_tilt_err = local_angle_between_unit_vectors(gyro_normal_world, pose_normal_world);
force_measured_x_for_control = -mob_force_final(:,1);  % normal = [-1, 0, 0] => f_n = n^T f = -fHatW_x
force_desired_plot = force_desired;
force_desired_plot_mn = 1.0e3 * force_desired_plot;
force_measured_x_for_control_mn = 1.0e3 * force_measured_x_for_control;
mob_force_none_mn = 1.0e3 * mob_force_none;
mob_force_residual_mn = 1.0e3 * mob_force_residual;
ee_offset_body = [0.1, 0.0, 0.04];
ee_pos = local_compute_ee_position_world(pose_xyz, pose_rpy, ee_offset_body);
normal_est_xy = normal_est_logged;
normal_est_xy(:,3) = 0.0;
normal_est_xy = local_normalize_rows(normal_est_xy);
normal_pre_xy = normal_pre_logged;
normal_pre_xy(:,3) = 0.0;
normal_pre_xy = local_normalize_rows(normal_pre_xy);
[normal_frame_t1, normal_frame_t2] = local_compute_normal_frame_tangents(normal_est_logged);
% Prefer firmware-logged gated tangential commands. Fall back to raw
% cmd_position tangential channels for older CSV files that do not yet log
% alpha_frame * t1/t2 desired commands.
contact_t1_cmd = t1_cmd_des_logged;
if ~any(isfinite(contact_t1_cmd))
    contact_t1_cmd = cmd_xyz(:,2);
end
contact_t1_meas = local_project_rows(ee_vel_used_world, normal_frame_t1);
contact_t2_cmd = t2_cmd_des_logged;
if ~any(isfinite(contact_t2_cmd))
    contact_t2_cmd = cmd_xyz(:,3);
end
contact_t2_meas = local_project_rows(ee_vel_used_world, normal_frame_t2);

hover_mask = isfinite(sum_thrust) & sum_thrust > 0.6 * mg_total & sum_thrust < 1.4 * mg_total;
if nnz(hover_mask) < 10
    hover_mask = true(size(sum_thrust));
end

fprintf("[INFO] Mean total thrust = %.4f N\n", mean(sum_thrust, 'omitnan'));
fprintf("[INFO] Hover reference (m*g) = %.4f N\n", mg_total);
fprintf("[INFO] Mean thrust error to m*g = %.4f N\n", mean(thrust_error_to_mg(hover_mask), 'omitnan'));
fprintf("[INFO] Mean |state_vel - pos_vel| = %.4f m/s\n", mean(vel_diff_norm, 'omitnan'));
fprintf("[INFO] Mean |pose - cmd| = %.4f m\n", mean(pos_error_norm, 'omitnan'));
fprintf("[INFO] Mean |pose - fw_cmd| = %.4f m\n", mean(fw_pos_error_norm, 'omitnan'));
fprintf("[INFO] Mean acc norm = %.4f m/s^2\n", mean(acc_norm, 'omitnan'));

%% 5) Plot styling
axis_names = {'X', 'Y', 'Z'};
motor_names = {'M1', 'M2', 'M3', 'M4'};
motor_colors = [
    0.8500 0.3250 0.0980
    0.4660 0.6740 0.1880
    0.0000 0.4470 0.7410
    0.4940 0.1840 0.5560
];
cmd_color = [0.0000 0.4470 0.7410];
meas_color = [0.8500 0.3250 0.0980];
vel_color = [0.0000 0.4470 0.7410];
acc_color = [0.2 0.6 0.2];
pos_color = [0.4940 0.1840 0.5560];
run_figure1_horizontal = true;
run_figure2_vertical = true;

%% 5.8) Figure 1: for horizontal
if run_figure1_horizontal
panelm045_xlim = [66 91];
panelm045_relative_time_origin = panelm045_xlim(1);
panelm045_split_time = 78.5;         % [] disables split. Absolute time in log.
panelm045_pos_ylim = {[-.0 1.6], [-0 1.5], [0.25 0.4]};
panelm045_force_ylim = {[-0.00 50.0], [-0.2 0.2], [-0.1 0.1]};
panelm045_mob_ylim = {[-80.0 80.0], [-80.0 80.0], [-80.0 80.0]};
panelm045_normal_ylim = {[-1.7 -0.3], [-1. 1.], [-0.8 0.6]};
panelm045_att_ylim = {[-0.1 0.1], [-0.1 0.1], [-1.5 0.5]};
panelm045_metric_ylim = {[], [], [0.10 1.02], []};
panelm045_alpha_min = 1.0;
panelm045_leak_bar = 0.01;
panelm045_normal_arrow_count = 15;   % larger -> denser arrows
panelm045_normal_arrow_scale = 0.6; % larger -> longer arrows

alpha_leakage = panelm045_alpha_min + (1.0 - panelm045_alpha_min) ./ ...
    (1.0 + (normal_velocity_leakage_logged ./ panelm045_leak_bar).^2);
att_des_plot_m045 = att_des;
att_des_plot_m045(:,3) = unwrap(att_des_plot_m045(:,3));
normal_est_norm = vecnorm(normal_est_logged, 2, 2);
panelm045_segments = local_make_time_segments(panelm045_xlim, panelm045_split_time);
panelm045_proxy_normal = nan(size(normal_est_logged));
for seg_idx = 1:size(panelm045_segments, 1)
    seg_mask = local_segment_mask(time, panelm045_segments(seg_idx,:));
    seg_proxy = local_estimate_proxy_wall_normal(ee_pos(seg_mask,:), normal_est_logged(seg_mask,:));
    if any(seg_mask) && all(isfinite(seg_proxy))
        panelm045_proxy_normal(seg_mask,:) = repmat(seg_proxy, nnz(seg_mask), 1);
    end
end
panelm045_proxy_err_xyz = normal_est_logged - panelm045_proxy_normal;
panelm045_proxy_rmse_xyz = nan(size(panelm045_segments, 1), 3);
for seg_idx = 1:size(panelm045_segments, 1)
    seg_mask = local_segment_mask(time, panelm045_segments(seg_idx,:));
    panelm045_proxy_rmse_xyz(seg_idx,:) = local_compute_component_rmse(panelm045_proxy_err_xyz(seg_mask,:));
end
fm045 = figure('Name', 'Defense 3x2 Contact / MOB / Normal Dashboard', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.05 0.9 0.9]);
time_axes_m045 = gobjects(0);

left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;
ncol = 3; nrow = 2;
w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;
getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% (1,1) Position meas vs desired
pm045_11 = uipanel('Parent', fm045, 'Position', getPos(1,1), 'BackgroundColor', 'w');
tlm045_11 = tiledlayout(pm045_11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile(tlm045_11, i);
    plot(ax, time, position_des_plot(:,i), '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(ax, 'on');
    plot(ax, time, pose_xyz(:,i), '-', 'LineWidth', 1.7, 'Color', meas_color);
    grid(ax, 'on');
    xlim(ax, panelm045_xlim);
    time_axes_m045(end+1) = ax;
    if ~isempty(panelm045_pos_ylim{i})
        ylim(ax, panelm045_pos_ylim{i});
    end
    ylabel(ax, sprintf('$%s$ [m]', lower(axis_names{i})), 'Interpreter', 'latex', 'FontSize', 12);
    if i == 1
        title(ax, 'Position');
        legend(ax, {'desired', 'measured'}, 'Location', 'best');
    end
    if i == 3
        xlabel(ax, 'Relative time [s]', 'Interpreter', 'latex', 'FontSize', 12);
    else
        set(ax, 'XTickLabel', []);
    end
end

% (2,1) Force / t1 / t2 cmd vs measured
pm045_21 = uipanel('Parent', fm045, 'Position', getPos(2,1), 'BackgroundColor', 'w');
tlm045_21 = tiledlayout(pm045_21, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

axm045_force = nexttile(tlm045_21, 1);
plot(axm045_force, time, force_desired_plot_mn, '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(axm045_force, 'on');
plot(axm045_force, time, force_measured_x_for_control_mn, '-', 'LineWidth', 1.7, 'Color', meas_color);
grid(axm045_force, 'on');
xlim(axm045_force, panelm045_xlim);
time_axes_m045(end+1) = axm045_force;
if ~isempty(panelm045_force_ylim{1})
    ylim(axm045_force, panelm045_force_ylim{1});
end
ylabel(axm045_force, '${}^{w}\hat{f}_x$ [mN]', 'Interpreter', 'latex', 'FontSize', 12);
title(axm045_force, 'Force / Tangential Command');
legend(axm045_force, {'cmd', 'measured'}, 'Location', 'best');
set(axm045_force, 'XTickLabel', []);

axm045_t1 = nexttile(tlm045_21, 2);
plot(axm045_t1, time, contact_t1_cmd, '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(axm045_t1, 'on');
plot(axm045_t1, time, contact_t1_meas, '-', 'LineWidth', 1.7, 'Color', meas_color);
grid(axm045_t1, 'on');
xlim(axm045_t1, panelm045_xlim);
time_axes_m045(end+1) = axm045_t1;
if ~isempty(panelm045_force_ylim{2})
    ylim(axm045_t1, panelm045_force_ylim{2});
end
ylabel(axm045_t1, '$t_1$ [m/s]', 'Interpreter', 'latex', 'FontSize', 12);
set(axm045_t1, 'XTickLabel', []);

axm045_t2 = nexttile(tlm045_21, 3);
plot(axm045_t2, time, contact_t2_cmd, '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(axm045_t2, 'on');
plot(axm045_t2, time, contact_t2_meas, '-', 'LineWidth', 1.7, 'Color', meas_color);
grid(axm045_t2, 'on');
xlim(axm045_t2, panelm045_xlim);
time_axes_m045(end+1) = axm045_t2;
if ~isempty(panelm045_force_ylim{3})
    ylim(axm045_t2, panelm045_force_ylim{3});
end
xlabel(axm045_t2, 'Relative time [s]', 'Interpreter', 'latex', 'FontSize', 12);
ylabel(axm045_t2, '$t_2$ [m/s]', 'Interpreter', 'latex', 'FontSize', 12);

% (1,2) Momentum observer pure vs consistency
pm045_12 = uipanel('Parent', fm045, 'Position', getPos(1,2), 'BackgroundColor', 'w');
tlm045_12 = tiledlayout(pm045_12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile(tlm045_12, i);
    plot(ax, time, mob_force_none_mn(:,i), '-', 'LineWidth', 2.0, 'Color', meas_color); hold(ax, 'on');
    plot(ax, time, mob_force_residual_mn(:,i), '-', 'LineWidth', 1.7, 'Color', cmd_color);
    grid(ax, 'on');
    xlim(ax, panelm045_xlim);
    time_axes_m045(end+1) = ax;
    if ~isempty(panelm045_mob_ylim{i})
        ylim(ax, panelm045_mob_ylim{i});
    end
    ylabel(ax, sprintf('${}^{w}\\hat{f}_{%s}$ [mN]', lower(axis_names{i})), 'Interpreter', 'latex', 'FontSize', 12);
    if i == 1
        title(ax, 'Momentum Observer');
        legend(ax, {'pure', 'consistency'}, 'Location', 'best');
    end
    if i == 3
        xlabel(ax, 'Relative time [s]', 'Interpreter', 'latex', 'FontSize', 12);
    else
        set(ax, 'XTickLabel', []);
    end
end

% (2,2) Normal estimation: final only
pm045_22 = uipanel('Parent', fm045, 'Position', getPos(2,2), 'BackgroundColor', 'w');
tlm045_22 = tiledlayout(pm045_22, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile(tlm045_22, i);
    plot(ax, time, normal_est_logged(:,i), '-', 'LineWidth', 1.7, 'Color', meas_color);
    grid(ax, 'on');
    xlim(ax, panelm045_xlim);
    time_axes_m045(end+1) = ax;
    if ~isempty(panelm045_normal_ylim{i})
        ylim(ax, panelm045_normal_ylim{i});
    end
    ylabel(ax, sprintf('$n_%s$ [-]', lower(axis_names{i})), 'Interpreter', 'latex', 'FontSize', 12);
    if i == 1
        title(ax, 'Normal Estimation');
    end
    if i == 3
        xlabel(ax, 'Relative time [s]', 'Interpreter', 'latex', 'FontSize', 12);
    else
        set(ax, 'XTickLabel', []);
    end
end

% (1,3) EE trajectory segment notes
pm045_13 = uipanel('Parent', fm045, 'Position', getPos(1,3), 'BackgroundColor', 'w');
axm045_info = axes('Parent', pm045_13, 'Position', [0 0 1 1], 'Visible', 'off');
text(axm045_info, 0.05, 0.86, 'EE trajectory + normal plots are opened in separate windows.', ...
    'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');
text(axm045_info, 0.05, 0.64, sprintf('segment 1: [%.1f, %.1f] s', ...
    panelm045_segments(1,1) - panelm045_relative_time_origin, ...
    panelm045_segments(1,2) - panelm045_relative_time_origin), ...
    'Units', 'normalized', 'FontSize', 11);
if size(panelm045_segments, 1) >= 2
    text(axm045_info, 0.05, 0.48, sprintf('segment 2: [%.1f, %.1f] s', ...
        panelm045_segments(2,1) - panelm045_relative_time_origin, ...
        panelm045_segments(2,2) - panelm045_relative_time_origin), ...
        'Units', 'normalized', 'FontSize', 11);
else
    text(axm045_info, 0.05, 0.48, 'segment 2: disabled', ...
        'Units', 'normalized', 'FontSize', 11);
end
text(axm045_info, 0.05, 0.24, 'Use panelm045_split_time to choose the break point.', ...
    'Units', 'normalized', 'FontSize', 11, 'Color', [0.3 0.3 0.3]);

% (2,3) Attitude desired vs measured
pm045_23 = uipanel('Parent', fm045, 'Position', getPos(2,3), 'BackgroundColor', 'w');
tlm045_23 = tiledlayout(pm045_23, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

axm045_att_roll = nexttile(tlm045_23, 1);
plot(axm045_att_roll, time, att_des_plot_m045(:,1), '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(axm045_att_roll, 'on');
plot(axm045_att_roll, time, pose_rpy(:,1), '-', 'LineWidth', 1.7, 'Color', meas_color);
grid(axm045_att_roll, 'on');
xlim(axm045_att_roll, panelm045_xlim);
time_axes_m045(end+1) = axm045_att_roll;
if ~isempty(panelm045_att_ylim{1})
    ylim(axm045_att_roll, panelm045_att_ylim{1});
end
ylabel(axm045_att_roll, '$\phi$ [rad]', 'Interpreter', 'latex', 'FontSize', 12);
title(axm045_att_roll, 'Attitude');
legend(axm045_att_roll, {'desired', 'measured'}, 'Location', 'best');
set(axm045_att_roll, 'XTickLabel', []);

axm045_att_pitch = nexttile(tlm045_23, 2);
plot(axm045_att_pitch, time, att_des_plot_m045(:,2), '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(axm045_att_pitch, 'on');
plot(axm045_att_pitch, time, pose_rpy(:,2), '-', 'LineWidth', 1.7, 'Color', meas_color);
grid(axm045_att_pitch, 'on');
xlim(axm045_att_pitch, panelm045_xlim);
time_axes_m045(end+1) = axm045_att_pitch;
if ~isempty(panelm045_att_ylim{2})
    ylim(axm045_att_pitch, panelm045_att_ylim{2});
end
ylabel(axm045_att_pitch, '$\theta$ [rad]', 'Interpreter', 'latex', 'FontSize', 12);
set(axm045_att_pitch, 'XTickLabel', []);

axm045_att_yaw = nexttile(tlm045_23, 3);
plot(axm045_att_yaw, time, att_des_plot_m045(:,3), '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(axm045_att_yaw, 'on');
plot(axm045_att_yaw, time, pose_rpy(:,3), '-', 'LineWidth', 1.7, 'Color', meas_color);
grid(axm045_att_yaw, 'on');
xlim(axm045_att_yaw, panelm045_xlim);
time_axes_m045(end+1) = axm045_att_yaw;
if ~isempty(panelm045_att_ylim{3})
    ylim(axm045_att_yaw, panelm045_att_ylim{3});
end
xlabel(axm045_att_yaw, 'Relative time [s]', 'Interpreter', 'latex', 'FontSize', 12);
ylabel(axm045_att_yaw, '$\psi$ [rad]', 'Interpreter', 'latex', 'FontSize', 12);

% Separate normal metrics window
fm045_metrics = figure('Name', 'Defense Normal Metrics', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.08 0.08 0.32 0.70]);
tlm045_metrics = tiledlayout(fm045_metrics, 1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

axm045_leak = nexttile(tlm045_metrics, 1);
plot(axm045_leak, time, normal_velocity_leakage_logged, '-', 'LineWidth', 1.8, 'Color', pos_color);
grid(axm045_leak, 'on');
xlim(axm045_leak, panelm045_xlim);
time_axes_m045(end+1) = axm045_leak;
if ~isempty(panelm045_metric_ylim{2})
    ylim(axm045_leak, panelm045_metric_ylim{2});
end
ylabel(axm045_leak, '$v_{\mathrm{leak}}$ [-]', 'Interpreter', 'latex', 'FontSize', 12);
title(axm045_leak, 'Normal Velocity Leakage');
xlabel(axm045_leak, 'Relative time [s]', 'Interpreter', 'latex', 'FontSize', 12);

linkaxes(time_axes_m045, 'x');
local_apply_relative_time_ticks(time_axes_m045, panelm045_relative_time_origin);
set(findall(fm045, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');
set(findall(fm045_metrics, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

for seg_idx = 1:size(panelm045_segments, 1)
    fm045_xz = figure('Name', sprintf('Defense EE / Normal Horizontal Segment %d', seg_idx), ...
        'NumberTitle', 'off', 'Color', 'w', 'Units', 'normalized', ...
        'Position', [0.10 + 0.04 * (seg_idx - 1) 0.12 + 0.04 * (seg_idx - 1) 0.255 0.39]);
    axm045_xz = axes('Parent', fm045_xz);
    local_plot_ee_normal_xz_defense(axm045_xz, time, ee_pos, normal_est_logged, ...
        panelm045_segments(seg_idx,:), panelm045_relative_time_origin, ...
        normal_est_norm, panelm045_proxy_normal, panelm045_proxy_rmse_xyz(seg_idx,:), ...
        panelm045_normal_arrow_count, panelm045_normal_arrow_scale);
    local_add_view_preset_buttons(fm045_xz, axm045_xz);
    set(axm045_xz, 'FontSize', 10, 'Color', 'w');
end

if size(panelm045_segments, 1) >= 1
    fm045_xz_first = figure('Name', 'Defense EE / Normal Horizontal Start-to-Split XZ', ...
        'NumberTitle', 'off', 'Color', 'w', 'Units', 'normalized', ...
        'Position', [0.22 0.18 0.255 0.39]);
    axm045_xz_first = axes('Parent', fm045_xz_first);
    local_plot_ee_normal_xz_plane_defense(axm045_xz_first, time, ee_pos, normal_est_logged, ...
        panelm045_segments(1,:), panelm045_relative_time_origin, ...
        normal_est_norm, panelm045_proxy_normal, panelm045_proxy_rmse_xyz(1,:), ...
        panelm045_normal_arrow_count, panelm045_normal_arrow_scale);
    local_add_view_preset_buttons(fm045_xz_first, axm045_xz_first);
    set(axm045_xz_first, 'FontSize', 10, 'Color', 'w');
end
end


%% 5.8) Figure 2: For vertical
if run_figure2_vertical
panelm045_xlim = [65 95];
panelm045_relative_time_origin = panelm045_xlim(1);
panelm045_split_time = 103;         % [] disables split. Absolute time in log.
panelm045_pos_ylim = {[-.0 1.6], [-1 0.5], [0.7 2.2]};
panelm045_force_ylim = {[-0.00 50.0], [-0.2 0.2], [-0.1 0.1]};
panelm045_mob_ylim = {[-80.0 80.0], [-80.0 80.0], [-80.0 80.0]};
panelm045_normal_ylim = {[-1.7 -0.3], [-1. 1.], [-0.8 0.6]};
panelm045_att_ylim = {[-0.1 0.1], [-0.1 0.1], [-0.5 1]};
panelm045_metric_ylim = {[], [], [0.10 1.02], []};
panelm045_alpha_min = 1.0;
panelm045_leak_bar = 0.01;
panelm045_normal_arrow_count = 15;   % larger -> denser arrows
panelm045_normal_arrow_scale = 0.35; % larger -> longer arrows

alpha_leakage = panelm045_alpha_min + (1.0 - panelm045_alpha_min) ./ ...
    (1.0 + (normal_velocity_leakage_logged ./ panelm045_leak_bar).^2);
att_des_plot_m045 = att_des;
att_des_plot_m045(:,3) = unwrap(att_des_plot_m045(:,3));
normal_est_norm = vecnorm(normal_est_logged, 2, 2);
panelm045_segments = local_make_time_segments(panelm045_xlim, panelm045_split_time);
panelm045_proxy_normal = nan(size(normal_est_logged));
for seg_idx = 1:size(panelm045_segments, 1)
    seg_mask = local_segment_mask(time, panelm045_segments(seg_idx,:));
    seg_proxy = local_estimate_proxy_wall_normal(ee_pos(seg_mask,:), normal_est_logged(seg_mask,:));
    if any(seg_mask) && all(isfinite(seg_proxy))
        panelm045_proxy_normal(seg_mask,:) = repmat(seg_proxy, nnz(seg_mask), 1);
    end
end
panelm045_proxy_err_xyz = normal_est_logged - panelm045_proxy_normal;
panelm045_proxy_rmse_xyz = nan(size(panelm045_segments, 1), 3);
for seg_idx = 1:size(panelm045_segments, 1)
    seg_mask = local_segment_mask(time, panelm045_segments(seg_idx,:));
    panelm045_proxy_rmse_xyz(seg_idx,:) = local_compute_component_rmse(panelm045_proxy_err_xyz(seg_mask,:));
end

fm045 = figure('Name', 'Defense 3x2 Contact / MOB / Normal Dashboard', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.05 0.9 0.9]);
time_axes_m045 = gobjects(0);

left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;
ncol = 3; nrow = 2;
w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;
getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% (1,1) Position meas vs desired
pm045_11 = uipanel('Parent', fm045, 'Position', getPos(1,1), 'BackgroundColor', 'w');
tlm045_11 = tiledlayout(pm045_11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile(tlm045_11, i);
    plot(ax, time, position_des_plot(:,i), '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(ax, 'on');
    plot(ax, time, pose_xyz(:,i), '-', 'LineWidth', 1.7, 'Color', meas_color);
    grid(ax, 'on');
    xlim(ax, panelm045_xlim);
    time_axes_m045(end+1) = ax;
    if ~isempty(panelm045_pos_ylim{i})
        ylim(ax, panelm045_pos_ylim{i});
    end
    ylabel(ax, sprintf('$%s$ [m]', lower(axis_names{i})), 'Interpreter', 'latex', 'FontSize', 12);
    if i == 1
        title(ax, 'Position');
        legend(ax, {'desired', 'measured'}, 'Location', 'best');
    end
    if i == 3
        xlabel(ax, 'Relative time [s]', 'Interpreter', 'latex', 'FontSize', 12);
    else
        set(ax, 'XTickLabel', []);
    end
end

% (2,1) Force / t1 / t2 cmd vs measured
pm045_21 = uipanel('Parent', fm045, 'Position', getPos(2,1), 'BackgroundColor', 'w');
tlm045_21 = tiledlayout(pm045_21, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

axm045_force = nexttile(tlm045_21, 1);
plot(axm045_force, time, force_desired_plot_mn, '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(axm045_force, 'on');
plot(axm045_force, time, force_measured_x_for_control_mn, '-', 'LineWidth', 1.7, 'Color', meas_color);
grid(axm045_force, 'on');
xlim(axm045_force, panelm045_xlim);
time_axes_m045(end+1) = axm045_force;
if ~isempty(panelm045_force_ylim{1})
    ylim(axm045_force, panelm045_force_ylim{1});
end
ylabel(axm045_force, '${}^{w}\hat{f}_x$ [mN]', 'Interpreter', 'latex', 'FontSize', 12);
title(axm045_force, 'Force / Tangential Command');
legend(axm045_force, {'cmd', 'measured'}, 'Location', 'best');
set(axm045_force, 'XTickLabel', []);

axm045_t1 = nexttile(tlm045_21, 2);
plot(axm045_t1, time, contact_t1_cmd, '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(axm045_t1, 'on');
plot(axm045_t1, time, contact_t1_meas, '-', 'LineWidth', 1.7, 'Color', meas_color);
grid(axm045_t1, 'on');
xlim(axm045_t1, panelm045_xlim);
time_axes_m045(end+1) = axm045_t1;
if ~isempty(panelm045_force_ylim{2})
    ylim(axm045_t1, panelm045_force_ylim{2});
end
ylabel(axm045_t1, '$t_1$ [m/s]', 'Interpreter', 'latex', 'FontSize', 12);
set(axm045_t1, 'XTickLabel', []);

axm045_t2 = nexttile(tlm045_21, 3);
plot(axm045_t2, time, contact_t2_cmd, '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(axm045_t2, 'on');
plot(axm045_t2, time, contact_t2_meas, '-', 'LineWidth', 1.7, 'Color', meas_color);
grid(axm045_t2, 'on');
xlim(axm045_t2, panelm045_xlim);
time_axes_m045(end+1) = axm045_t2;
if ~isempty(panelm045_force_ylim{3})
    ylim(axm045_t2, panelm045_force_ylim{3});
end
xlabel(axm045_t2, 'Relative time [s]', 'Interpreter', 'latex', 'FontSize', 12);
ylabel(axm045_t2, '$t_2$ [m/s]', 'Interpreter', 'latex', 'FontSize', 12);

% (1,2) Momentum observer pure vs consistency
pm045_12 = uipanel('Parent', fm045, 'Position', getPos(1,2), 'BackgroundColor', 'w');
tlm045_12 = tiledlayout(pm045_12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile(tlm045_12, i);
    plot(ax, time, mob_force_none_mn(:,i), '-', 'LineWidth', 2.0, 'Color', meas_color); hold(ax, 'on');
    plot(ax, time, mob_force_residual_mn(:,i), '-', 'LineWidth', 1.7, 'Color', cmd_color);
    grid(ax, 'on');
    xlim(ax, panelm045_xlim);
    time_axes_m045(end+1) = ax;
    if ~isempty(panelm045_mob_ylim{i})
        ylim(ax, panelm045_mob_ylim{i});
    end
    ylabel(ax, sprintf('${}^{w}\\hat{f}_{%s}$ [mN]', lower(axis_names{i})), 'Interpreter', 'latex', 'FontSize', 12);
    if i == 1
        title(ax, 'Momentum Observer');
        legend(ax, {'pure', 'consistency'}, 'Location', 'best');
    end
    if i == 3
        xlabel(ax, 'Relative time [s]', 'Interpreter', 'latex', 'FontSize', 12);
    else
        set(ax, 'XTickLabel', []);
    end
end

% (2,2) Normal estimation: final only
pm045_22 = uipanel('Parent', fm045, 'Position', getPos(2,2), 'BackgroundColor', 'w');
tlm045_22 = tiledlayout(pm045_22, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile(tlm045_22, i);
    plot(ax, time, normal_est_logged(:,i), '-', 'LineWidth', 1.7, 'Color', meas_color);
    grid(ax, 'on');
    xlim(ax, panelm045_xlim);
    time_axes_m045(end+1) = ax;
    if ~isempty(panelm045_normal_ylim{i})
        ylim(ax, panelm045_normal_ylim{i});
    end
    ylabel(ax, sprintf('$n_%s$ [-]', lower(axis_names{i})), 'Interpreter', 'latex', 'FontSize', 12);
    if i == 1
        title(ax, 'Normal Estimation');
    end
    if i == 3
        xlabel(ax, 'Relative time [s]', 'Interpreter', 'latex', 'FontSize', 12);
    else
        set(ax, 'XTickLabel', []);
    end
end

% (1,3) EE trajectory segment notes
pm045_13 = uipanel('Parent', fm045, 'Position', getPos(1,3), 'BackgroundColor', 'w');
axm045_info = axes('Parent', pm045_13, 'Position', [0 0 1 1], 'Visible', 'off');
text(axm045_info, 0.05, 0.86, 'EE trajectory + normal plots are opened in separate windows.', ...
    'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');
text(axm045_info, 0.05, 0.64, sprintf('segment 1: [%.1f, %.1f] s', ...
    panelm045_segments(1,1) - panelm045_relative_time_origin, ...
    panelm045_segments(1,2) - panelm045_relative_time_origin), ...
    'Units', 'normalized', 'FontSize', 11);
if size(panelm045_segments, 1) >= 2
    text(axm045_info, 0.05, 0.48, sprintf('segment 2: [%.1f, %.1f] s', ...
        panelm045_segments(2,1) - panelm045_relative_time_origin, ...
        panelm045_segments(2,2) - panelm045_relative_time_origin), ...
        'Units', 'normalized', 'FontSize', 11);
else
    text(axm045_info, 0.05, 0.48, 'segment 2: disabled', ...
        'Units', 'normalized', 'FontSize', 11);
end
text(axm045_info, 0.05, 0.24, 'Use panelm045_split_time to choose the break point.', ...
    'Units', 'normalized', 'FontSize', 11, 'Color', [0.3 0.3 0.3]);

% (2,3) Attitude desired vs measured
pm045_23 = uipanel('Parent', fm045, 'Position', getPos(2,3), 'BackgroundColor', 'w');
tlm045_23 = tiledlayout(pm045_23, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

axm045_att_roll = nexttile(tlm045_23, 1);
plot(axm045_att_roll, time, att_des_plot_m045(:,1), '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(axm045_att_roll, 'on');
plot(axm045_att_roll, time, pose_rpy(:,1), '-', 'LineWidth', 1.7, 'Color', meas_color);
grid(axm045_att_roll, 'on');
xlim(axm045_att_roll, panelm045_xlim);
time_axes_m045(end+1) = axm045_att_roll;
if ~isempty(panelm045_att_ylim{1})
    ylim(axm045_att_roll, panelm045_att_ylim{1});
end
ylabel(axm045_att_roll, '$\phi$ [rad]', 'Interpreter', 'latex', 'FontSize', 12);
title(axm045_att_roll, 'Attitude');
legend(axm045_att_roll, {'desired', 'measured'}, 'Location', 'best');
set(axm045_att_roll, 'XTickLabel', []);

axm045_att_pitch = nexttile(tlm045_23, 2);
plot(axm045_att_pitch, time, att_des_plot_m045(:,2), '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(axm045_att_pitch, 'on');
plot(axm045_att_pitch, time, pose_rpy(:,2), '-', 'LineWidth', 1.7, 'Color', meas_color);
grid(axm045_att_pitch, 'on');
xlim(axm045_att_pitch, panelm045_xlim);
time_axes_m045(end+1) = axm045_att_pitch;
if ~isempty(panelm045_att_ylim{2})
    ylim(axm045_att_pitch, panelm045_att_ylim{2});
end
ylabel(axm045_att_pitch, '$\theta$ [rad]', 'Interpreter', 'latex', 'FontSize', 12);
set(axm045_att_pitch, 'XTickLabel', []);

axm045_att_yaw = nexttile(tlm045_23, 3);
plot(axm045_att_yaw, time, att_des_plot_m045(:,3), '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(axm045_att_yaw, 'on');
plot(axm045_att_yaw, time, pose_rpy(:,3), '-', 'LineWidth', 1.7, 'Color', meas_color);
grid(axm045_att_yaw, 'on');
xlim(axm045_att_yaw, panelm045_xlim);
time_axes_m045(end+1) = axm045_att_yaw;
if ~isempty(panelm045_att_ylim{3})
    ylim(axm045_att_yaw, panelm045_att_ylim{3});
end
xlabel(axm045_att_yaw, 'Relative time [s]', 'Interpreter', 'latex', 'FontSize', 12);
ylabel(axm045_att_yaw, '$\psi$ [rad]', 'Interpreter', 'latex', 'FontSize', 12);

% Separate normal metrics window
fm045_metrics = figure('Name', 'Defense Normal Metrics', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.08 0.08 0.32 0.70]);
tlm045_metrics = tiledlayout(fm045_metrics, 1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

axm045_leak = nexttile(tlm045_metrics, 1);
plot(axm045_leak, time, normal_velocity_leakage_logged, '-', 'LineWidth', 1.8, 'Color', pos_color);
grid(axm045_leak, 'on');
xlim(axm045_leak, panelm045_xlim);
time_axes_m045(end+1) = axm045_leak;
if ~isempty(panelm045_metric_ylim{2})
    ylim(axm045_leak, panelm045_metric_ylim{2});
end
ylabel(axm045_leak, '$v_{\mathrm{leak}}$ [-]', 'Interpreter', 'latex', 'FontSize', 12);
title(axm045_leak, 'Normal Velocity Leakage');
xlabel(axm045_leak, 'Relative time [s]', 'Interpreter', 'latex', 'FontSize', 12);

linkaxes(time_axes_m045, 'x');
local_apply_relative_time_ticks(time_axes_m045, panelm045_relative_time_origin);
set(findall(fm045, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');
set(findall(fm045_metrics, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

for seg_idx = 1:size(panelm045_segments, 1)
    fm045_xz = figure('Name', sprintf('Defense EE / Normal Vertical Segment %d', seg_idx), ...
        'NumberTitle', 'off', 'Color', 'w', 'Units', 'normalized', ...
        'Position', [0.46 + 0.04 * (seg_idx - 1) 0.12 + 0.04 * (seg_idx - 1) 0.255 0.39]);
    axm045_xz = axes('Parent', fm045_xz);
    local_plot_ee_normal_xz_plane_defense(axm045_xz, time, ee_pos, normal_est_logged, ...
        panelm045_segments(seg_idx,:), panelm045_relative_time_origin, ...
        normal_est_norm, panelm045_proxy_normal, panelm045_proxy_rmse_xyz(seg_idx,:), ...
        panelm045_normal_arrow_count, panelm045_normal_arrow_scale);
    set(axm045_xz, 'FontSize', 10, 'Color', 'w');
end
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


function rpy = local_integrate_body_rates_to_rpy_deg(gyro_deg_s, time, rpy0)
    n = size(gyro_deg_s, 1);
    rpy = nan(n, 3);
    if n == 0
        return;
    end

    if nargin < 3 || any(~isfinite(rpy0))
        rpy0 = [0 0 0];
    end

    q = local_rpy_to_quat(rpy0);
    rpy(1, :) = rpy0;

    for k = 2:n
        dt = time(k) - time(k - 1);
        if ~isfinite(dt) || dt <= 0
            rpy(k, :) = rpy(k - 1, :);
            continue;
        end

        omega_deg = gyro_deg_s(k - 1, :);
        if any(~isfinite(omega_deg))
            rpy(k, :) = rpy(k - 1, :);
            continue;
        end

        omega = deg2rad(omega_deg);
        omega_quat = [0, omega(1), omega(2), omega(3)];
        qdot = 0.5 * local_quat_mul(q, omega_quat);
        q = q + qdot * dt;
        q = q / norm(q);
        rpy(k, :) = local_quat_to_rpy(q);
    end
end

function q = local_rpy_to_quat(rpy)
    cr = cos(rpy(1) / 2); sr = sin(rpy(1) / 2);
    cp = cos(rpy(2) / 2); sp = sin(rpy(2) / 2);
    cy = cos(rpy(3) / 2); sy = sin(rpy(3) / 2);

    q = [ ...
        cr * cp * cy + sr * sp * sy, ...
        sr * cp * cy - cr * sp * sy, ...
        cr * sp * cy + sr * cp * sy, ...
        cr * cp * sy - sr * sp * cy];
end

function rpy = local_quat_to_rpy(q)
    w = q(1); x = q(2); y = q(3); z = q(4);

    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    sinp = 2 * (w * y - z * x);
    sinp = max(min(sinp, 1), -1);
    pitch = asin(sinp);

    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);

    rpy = [roll, pitch, yaw];
end

function out = local_quat_mul(a, b)
    out = [ ...
        a(1) * b(1) - a(2) * b(2) - a(3) * b(3) - a(4) * b(4), ...
        a(1) * b(2) + a(2) * b(1) + a(3) * b(4) - a(4) * b(3), ...
        a(1) * b(3) - a(2) * b(4) + a(3) * b(1) + a(4) * b(2), ...
        a(1) * b(4) + a(2) * b(3) - a(3) * b(2) + a(4) * b(1)];
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
        finite_idx = find(isfinite(x(:, col)), 1, 'first');
        if isempty(finite_idx)
            continue;
        end

        y(1:finite_idx, col) = x(finite_idx, col);
        for row = finite_idx + 1:size(x, 1)
            if ~isfinite(x(row, col))
                y(row, col) = y(row - 1, col);
            else
                y(row, col) = y(row - 1, col) + alpha * (x(row, col) - y(row - 1, col));
            end
        end
    end
end

function normal_world = local_rpy_to_world_normal(rpy)
    n = size(rpy, 1);
    normal_world = nan(n, 3);
    for k = 1:n
        if any(~isfinite(rpy(k, 1:2)))
            continue;
        end

        roll = rpy(k, 1);
        pitch = rpy(k, 2);
        yaw = 0.0;
        if size(rpy, 2) >= 3 && isfinite(rpy(k, 3))
            yaw = rpy(k, 3);
        end

        R = local_rpy_to_rotmat([roll, pitch, yaw]);
        normal_world(k, :) = R(:, 3).';
    end
end

function R = local_rpy_to_rotmat(rpy)
    roll = rpy(1);
    pitch = rpy(2);
    yaw = rpy(3);

    cr = cos(roll); sr = sin(roll);
    cp = cos(pitch); sp = sin(pitch);
    cy = cos(yaw); sy = sin(yaw);

    R = [ ...
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr; ...
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr; ...
        -sp,     cp * sr,                cp * cr];
end

function ang_deg = local_angle_between_unit_vectors(a, b)
    dot_ab = sum(a .* b, 2, 'omitnan');
    norm_a = vecnorm(a, 2, 2);
    norm_b = vecnorm(b, 2, 2);
    denom = norm_a .* norm_b;
    cos_theta = nan(size(dot_ab));
    valid = denom > 0 & isfinite(denom);
    cos_theta(valid) = dot_ab(valid) ./ denom(valid);
    cos_theta = max(min(cos_theta, 1), -1);
    ang_deg = rad2deg(acos(cos_theta));
end

function ee_pos = local_compute_ee_position_world(pose_xyz, pose_rpy, ee_offset_body)
    n = size(pose_xyz, 1);
    ee_pos = nan(n, 3);
    for k = 1:n
        if any(~isfinite(pose_xyz(k,:))) || any(~isfinite(pose_rpy(k,:)))
            continue;
        end
        R = local_rpy_to_rotmat(pose_rpy(k,:));
        ee_pos(k,:) = pose_xyz(k,:) + (R * ee_offset_body(:)).';
    end
end

function unit_rows = local_normalize_rows(x)
    unit_rows = x;
    row_norm = vecnorm(x, 2, 2);
    valid = isfinite(row_norm) & row_norm > 1.0e-9;
    unit_rows(valid,:) = x(valid,:) ./ row_norm(valid);
    unit_rows(~valid,:) = nan;
end

function [color_data, color_min, color_max] = local_prepare_time_spectrum_data(time_valid, n_points, time_origin)
    if nargin < 3 || isempty(time_origin) || ~isfinite(time_origin)
        first_valid_idx = find(isfinite(time_valid), 1, 'first');
        if isempty(first_valid_idx)
            time_origin = 0.0;
        else
            time_origin = time_valid(first_valid_idx);
        end
    end

    color_data = time_valid(:) - time_origin;
    color_min = min(color_data, [], 'omitnan');
    color_max = max(color_data, [], 'omitnan');
    if ~isfinite(color_min) || ~isfinite(color_max) || abs(color_max - color_min) < 1.0e-9
        color_min = 0.0;
        color_max = 1.0;
        color_data = linspace(0.0, 1.0, n_points).';
    end
end

function cmap = local_make_ee_spectrum_colormap(n)
    if nargin < 1 || isempty(n)
        n = 256;
    end

    base = parula(n);
    % Tone down saturation/contrast so the EE spectrum stays readable
    % without overpowering the normal arrows and annotations.
    cmap = 0.68 * base + 0.32 * ones(size(base));
end

function local_plot_spectrum_trajectory3(ax, xyz, color_data, line_width, click_callback)
    if nargin < 4 || isempty(line_width)
        line_width = 2.0;
    end
    if nargin < 5
        click_callback = [];
    end

    if size(xyz, 1) >= 2
        surface(ax, ...
            [xyz(:,1), xyz(:,1)], ...
            [xyz(:,2), xyz(:,2)], ...
            [xyz(:,3), xyz(:,3)], ...
            [color_data(:), color_data(:)], ...
            'FaceColor', 'none', 'EdgeColor', 'interp', 'LineWidth', line_width, ...
            'HandleVisibility', 'off');
    end

    scatter_args = {'filled', 'MarkerFaceAlpha', 0.9, 'MarkerEdgeAlpha', 0.0};
    if ~isempty(click_callback)
        scatter_args = [scatter_args, {'PickableParts', 'all', 'ButtonDownFcn', click_callback}];
    end
    scatter3(ax, xyz(:,1), xyz(:,2), xyz(:,3), 16, color_data, scatter_args{:});
end

function local_plot_spectrum_trajectory2(ax, xy, color_data, line_width)
    if nargin < 4 || isempty(line_width)
        line_width = 2.0;
    end

    if size(xy, 1) >= 2
        surface(ax, ...
            [xy(:,1), xy(:,1)], ...
            [xy(:,2), xy(:,2)], ...
            zeros(size(xy,1), 2), ...
            [color_data(:), color_data(:)], ...
            'FaceColor', 'none', 'EdgeColor', 'interp', 'LineWidth', line_width, ...
            'HandleVisibility', 'off');
    end
end

function local_plot_ee_normal_xy(ax, time, ee_pos, normal_pre_xy, normal_est_xy, color_metric, xlim_time)
    valid_time = true(size(time));
    if numel(xlim_time) == 2 && all(isfinite(xlim_time))
        valid_time = time >= xlim_time(1) & time <= xlim_time(2);
    end

    ee_masked = ee_pos;
    normal_pre_masked = normal_pre_xy;
    normal_est_masked = normal_est_xy;
    color_masked = color_metric;
    ee_masked(~valid_time, :) = nan;
    normal_pre_masked(~valid_time, :) = nan;
    normal_est_masked(~valid_time, :) = nan;
    color_masked(~valid_time) = nan;
    valid = all(isfinite(ee_masked), 2);

    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'X [m]');
    ylabel(ax, 'Y [m]');
    title(ax, 'XY path with logged normal');

    if ~any(valid)
        text(ax, 0.5, 0.5, 'EE position is not available', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
        return;
    end

    time_valid = time(valid);
    ee_valid = ee_masked(valid,:);
    color_valid = color_masked(valid);
    if ~any(isfinite(color_valid))
        color_valid = linspace(0.0, 1.0, size(ee_valid,1)).';
    end
    color_min = min(color_valid, [], 'omitnan');
    color_max = max(color_valid, [], 'omitnan');
    if ~isfinite(color_min) || ~isfinite(color_max) || abs(color_max - color_min) < 1.0e-9
        color_min = 0.0;
        color_max = 1.0;
        color_valid = linspace(0.0, 1.0, size(ee_valid,1)).';
    end

    local_plot_spectrum_trajectory2(ax, ee_valid(:,1:2), color_valid, 3.0);
    hTip = scatter(ax, ee_valid(:,1), ee_valid(:,2), 16, color_valid, ...
        'filled', 'MarkerFaceAlpha', 0.9, 'MarkerEdgeAlpha', 0.0, ...
        'PickableParts', 'all', 'ButtonDownFcn', @local_handle_ee_xy_click);
    scatter(ax, ee_valid(1,1), ee_valid(1,2), 36, [0.30 0.30 0.30], 'filled', 'MarkerEdgeColor', 'k');
    scatter(ax, ee_valid(end,1), ee_valid(end,2), 36, [0.15 0.15 0.15], 'filled', 'MarkerEdgeColor', 'k');

    cmap = local_make_ee_spectrum_colormap(256);
    colormap(ax, cmap);
    cb = colorbar(ax);
    cb.Label.String = '\alpha_{frame}';
    caxis(ax, [color_min color_max]);

    ax.UserData.ee_xy_time = time_valid;
    ax.UserData.ee_xy_pos = ee_valid;
    ax.UserData.ee_xy_cdata = color_valid(:);
    ax.UserData.ee_xy_cmap = cmap;
    ax.UserData.ee_xy_title = 'XY path with logged normal';

    quiver_valid = valid & all(isfinite(normal_est_masked), 2);
    quiver_idx = find(quiver_valid);
    quiver_step = max(1, floor(numel(quiver_idx) / 22));
    quiver_idx = quiver_idx(1:quiver_step:end);
    if ~isempty(quiver_idx)
        hEst = quiver(ax, ee_masked(quiver_idx,1), ee_masked(quiver_idx,2), ...
            normal_est_masked(quiver_idx,1), normal_est_masked(quiver_idx,2), 0.84, ...
            'Color', [0.1500 0.7000 0.2500], 'LineWidth', 1.2, 'MaxHeadSize', 1.35);
    else
        hEst = gobjects(1);
    end

    arrow_xy = ee_valid(:, 1:2);
    if ~isempty(quiver_idx)
        arrow_xy = [arrow_xy; ...
            ee_masked(quiver_idx,1:2); ...
            ee_masked(quiver_idx,1:2) + 0.84 * normal_est_masked(quiver_idx,1:2)];
    end
    local_set_planar_axis_limits(ax, arrow_xy, 0.10, 0.005);
    legend(ax, [hTip, hEst], ...
        {'tip position', 'normal est'}, ...
        'Location', 'eastoutside');
end

function local_plot_ee_normal_xy_3d(ax, time, ee_pos, normal_world, xlim_time)
    valid_time = true(size(time));
    if numel(xlim_time) == 2 && all(isfinite(xlim_time))
        valid_time = time >= xlim_time(1) & time <= xlim_time(2);
    end

    ee_masked = ee_pos;
    normal_masked = normal_world;
    ee_masked(~valid_time, :) = nan;
    normal_masked(~valid_time, :) = nan;
    valid = all(isfinite(ee_masked), 2);

    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'X [m]');
    ylabel(ax, 'Y [m]');
    zlabel(ax, 'Z [m]');
    title(ax, '3D path with logged normal');

    if ~any(valid)
        text(ax, 0.5, 0.5, 'EE position is not available', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
        return;
    end

    time_valid = time(valid);
    ee_valid = ee_masked(valid,:);
    time_origin = [];
    if numel(xlim_time) == 2 && all(isfinite(xlim_time))
        time_origin = xlim_time(1);
    end
    [color_valid, color_min, color_max] = local_prepare_time_spectrum_data(time_valid, size(ee_valid,1), time_origin);
    cmap = local_make_ee_spectrum_colormap(256);
    local_plot_spectrum_trajectory3(ax, ee_valid, color_valid, 3.0, @local_handle_ee_xy_click);
    hTip = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 6, ...
        'MarkerFaceColor', cmap(128,:), 'MarkerEdgeColor', 'none', ...
        'LineStyle', 'none');
    scatter3(ax, ee_valid(1,1), ee_valid(1,2), ee_valid(1,3), 36, [0.30 0.30 0.30], 'filled', 'MarkerEdgeColor', 'k');
    scatter3(ax, ee_valid(end,1), ee_valid(end,2), ee_valid(end,3), 36, [0.15 0.15 0.15], 'filled', 'MarkerEdgeColor', 'k');

    colormap(ax, cmap);
    cb = colorbar(ax);
    cb.Label.String = 'relative time [s]';
    caxis(ax, [color_min color_max]);
    ax.UserData.ee_xy_time = time_valid;
    ax.UserData.ee_xy_time_display = color_valid(:);
    ax.UserData.ee_xy_pos = ee_valid;
    ax.UserData.ee_xy_cdata = color_valid(:);
    ax.UserData.ee_xy_cmap = cmap;
    ax.UserData.ee_xy_title = '3D path with logged normal';

    quiver_idx = find(valid & all(isfinite(normal_masked), 2));
    quiver_step = max(1, floor(numel(quiver_idx) / 25));
    quiver_idx = quiver_idx(1:quiver_step:end);
    if ~isempty(quiver_idx)
        hEst = quiver3(ax, ee_masked(quiver_idx,1), ee_masked(quiver_idx,2), ee_masked(quiver_idx,3), ...
            normal_masked(quiver_idx,1), normal_masked(quiver_idx,2), normal_masked(quiver_idx,3), 0.50, ...
            'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.0, 'MaxHeadSize', 1.5);
    else
        hEst = gobjects(1);
    end

    spatial_points = ee_valid;
    if ~isempty(quiver_idx)
        spatial_points = [spatial_points; ...
            ee_masked(quiver_idx,:); ...
            ee_masked(quiver_idx,:) + 0.50 * normal_masked(quiver_idx,:)];
    end
    local_set_spatial_axis_limits(ax, spatial_points, 0.10, 0.005);
    legend(ax, [hTip, hEst], {'tip position', 'normal est'}, 'Location', 'eastoutside');
end

function local_plot_ee_normal_xy_plane_3d(ax, time, ee_pos, normal_final, normal_pre, xlim_time)
    final_arrow_scale = 0.50;

    valid_time = true(size(time));
    if numel(xlim_time) == 2 && all(isfinite(xlim_time))
        valid_time = time >= xlim_time(1) & time <= xlim_time(2);
    end

    ee_masked = ee_pos;
    final_masked = normal_final;
    ee_masked(~valid_time, :) = nan;
    final_masked(~valid_time, :) = nan;

    ee_proj = ee_masked;
    ee_proj(:,3) = 0.0;
    final_proj = final_masked;
    final_proj(:,3) = 0.0;
    final_proj = local_normalize_rows(final_proj);

    valid = all(isfinite(ee_proj), 2);

    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 2);
    xlabel(ax, 'X [m]');
    ylabel(ax, 'Y [m]');
    zlabel(ax, 'plane Z');
    title(ax, 'EE trajectory + normal arrow (XY plane / rotatable 3D)');

    if ~any(valid)
        text(ax, 0.5, 0.5, 'EE position is not available', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
        return;
    end

    time_valid = time(valid);
    ee_valid = ee_proj(valid,:);
    time_origin = [];
    if numel(xlim_time) == 2 && all(isfinite(xlim_time))
        time_origin = xlim_time(1);
    end
    [color_valid, color_min, color_max] = local_prepare_time_spectrum_data(time_valid, size(ee_valid,1), time_origin);
    cmap = local_make_ee_spectrum_colormap(256);
    local_plot_spectrum_trajectory3(ax, ee_valid, color_valid, 3.0, @local_handle_ee_xy_click);
    hTip = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 6, ...
        'MarkerFaceColor', cmap(128,:), 'MarkerEdgeColor', 'none', ...
        'LineStyle', 'none');
    scatter3(ax, ee_valid(1,1), ee_valid(1,2), ee_valid(1,3), 36, [0.30 0.30 0.30], 'filled', 'MarkerEdgeColor', 'k');
    scatter3(ax, ee_valid(end,1), ee_valid(end,2), ee_valid(end,3), 36, [0.15 0.15 0.15], 'filled', 'MarkerEdgeColor', 'k');

    colormap(ax, cmap);
    cb = colorbar(ax);
    cb.Label.String = 'relative time [s]';
    caxis(ax, [color_min color_max]);
    ax.UserData.ee_xy_time = time_valid;
    ax.UserData.ee_xy_time_display = color_valid(:);
    ax.UserData.ee_xy_pos = ee_valid;
    ax.UserData.ee_xy_cdata = color_valid(:);
    ax.UserData.ee_xy_cmap = cmap;
    ax.UserData.ee_xy_title = 'EE trajectory + normal arrow (XY plane / rotatable 3D)';

    final_idx = find(valid & all(isfinite(final_proj), 2));
    final_step = max(1, floor(numel(final_idx) / 25));
    final_idx = final_idx(1:final_step:end);
    if ~isempty(final_idx)
        hFinal = quiver3(ax, ee_proj(final_idx,1), ee_proj(final_idx,2), ee_proj(final_idx,3), ...
            final_proj(final_idx,1), final_proj(final_idx,2), final_proj(final_idx,3), final_arrow_scale, ...
            'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.2, 'MaxHeadSize', 2.0);
    else
        hFinal = gobjects(1);
    end

    spatial_points = ee_valid;
    if ~isempty(final_idx)
        spatial_points = [spatial_points; ...
            ee_proj(final_idx,:); ...
            ee_proj(final_idx,:) + final_arrow_scale * final_proj(final_idx,:)];
    end
    local_set_spatial_axis_limits(ax, spatial_points, 0.10, 0.005);
    legend(ax, [hTip, hFinal], {'tip position', 'final normal'}, ...
        'Location', 'southwest');
end

function local_sync_panelm045_xz(src_ax, fig_handle)
    if ~isgraphics(src_ax, 'axes') || ~isgraphics(fig_handle, 'figure')
        return;
    end

    ax_xz = getappdata(fig_handle, 'panelm045_xz_ax');
    time = getappdata(fig_handle, 'panelm045_time');
    ee_pos = getappdata(fig_handle, 'panelm045_ee_pos');
    normal_est = getappdata(fig_handle, 'panelm045_normal_est');
    if ~isgraphics(ax_xz, 'axes') || isempty(time) || isempty(ee_pos) || isempty(normal_est)
        return;
    end

    local_plot_ee_normal_xz(ax_xz, time, ee_pos, normal_est, src_ax.XLim);
end

function local_apply_relative_time_ticks(ax_list, time_origin)
    if isempty(ax_list)
        return;
    end
    for k = 1:numel(ax_list)
        ax = ax_list(k);
        if ~isgraphics(ax, 'axes')
            continue;
        end
        current_labels = get(ax, 'XTickLabel');
        if isempty(current_labels)
            continue;
        end
        xt = get(ax, 'XTick');
        if isempty(xt)
            continue;
        end
        xtlbl = compose('%.1f', xt - time_origin);
        set(ax, 'XTickLabel', xtlbl);
    end
end

function segments = local_make_time_segments(xlim_time, split_time)
    segments = xlim_time;
    if numel(xlim_time) ~= 2 || ~all(isfinite(xlim_time))
        return;
    end

    split_time_used = split_time;
    if isempty(split_time_used) || ~isfinite(split_time_used) || ...
            split_time_used <= xlim_time(1) || split_time_used >= xlim_time(2)
        split_time_used = 0.5 * (xlim_time(1) + xlim_time(2));
    end

    segments = [
        xlim_time(1), split_time_used
        split_time_used, xlim_time(2)
    ];
end

function mask = local_segment_mask(time, segment_xlim)
    mask = false(size(time));
    if numel(segment_xlim) ~= 2 || ~all(isfinite(segment_xlim))
        return;
    end
    mask = isfinite(time) & time >= segment_xlim(1) & time <= segment_xlim(2);
end

function rmse_xyz = local_compute_component_rmse(err_xyz)
    rmse_xyz = [nan, nan, nan];
    if isempty(err_xyz)
        return;
    end
    for axis_idx = 1:3
        col = err_xyz(:, axis_idx);
        valid = isfinite(col);
        if any(valid)
            rmse_xyz(axis_idx) = sqrt(mean(col(valid).^2));
        end
    end
end

function proxy_normal = local_estimate_proxy_wall_normal(ee_segment, normal_segment)
    proxy_normal = [nan, nan, nan];
    valid = all(isfinite(ee_segment), 2);
    if nnz(valid) < 3
        return;
    end

    pts = ee_segment(valid,:);
    pts_centered = pts - mean(pts, 1, 'omitnan');
    if rank(pts_centered) < 2
        return;
    end

    [~, ~, V] = svd(pts_centered, 'econ');
    proxy_normal = V(:,end).';
    proxy_normal = proxy_normal ./ max(norm(proxy_normal), eps);

    normal_valid = normal_segment(valid,:);
    if any(all(isfinite(normal_valid), 2))
        mean_logged = mean(normal_valid(all(isfinite(normal_valid), 2), :), 1, 'omitnan');
        if all(isfinite(mean_logged)) && dot(proxy_normal, mean_logged) < 0
            proxy_normal = -proxy_normal;
        end
    end
end

function yaw_series = local_normal_to_yaw(normal_rows)
    yaw_series = nan(size(normal_rows, 1), 1);
    valid = all(isfinite(normal_rows), 2);
    if ~any(valid)
        return;
    end

    yaw_valid = atan2(normal_rows(valid, 2), normal_rows(valid, 1));
    yaw_series(valid) = unwrap(yaw_valid);
end

function yaw_series = local_compute_horizontal_curvature_normal_yaw(ee_pos, time, normal_ref, yaw_ref_series)
    yaw_series = nan(size(time));
    if size(ee_pos, 1) ~= numel(time)
        return;
    end

    valid = all(isfinite(ee_pos(:,1:2)), 2) & isfinite(time);
    if nnz(valid) < 5
        return;
    end

    xy = ee_pos(valid, 1:2);
    t = time(valid);

    xy(:,1) = local_lowpass_first_order(xy(:,1), 1.0 / median(diff(t)), 1.0);
    xy(:,2) = local_lowpass_first_order(xy(:,2), 1.0 / median(diff(t)), 1.0);

    vx = gradient(xy(:,1), t);
    vy = gradient(xy(:,2), t);
    speed = hypot(vx, vy);

    tx = nan(size(vx));
    ty = nan(size(vy));
    moving = isfinite(speed) & speed > 1.0e-4;
    tx(moving) = vx(moving) ./ speed(moving);
    ty(moving) = vy(moving) ./ speed(moving);

    dtx = gradient(tx, t);
    dty = gradient(ty, t);
    normal_xy = [dtx, dty];
    normal_xy = local_normalize_rows([normal_xy, zeros(size(normal_xy, 1), 1)]);
    normal_xy = normal_xy(:,1:2);

    if nargin >= 3 && size(normal_ref, 1) == numel(time)
        ref_xy = normal_ref(valid, 1:2);
        ref_xy = local_normalize_rows([ref_xy, zeros(size(ref_xy, 1), 1)]);
        ref_xy = ref_xy(:,1:2);
        ref_valid = all(isfinite(ref_xy), 2) & all(isfinite(normal_xy), 2);
        align_flip = false;
        if any(ref_valid)
            dot_mean = mean(sum(normal_xy(ref_valid,:) .* ref_xy(ref_valid,:), 2), 'omitnan');
            if isfinite(dot_mean) && dot_mean < 0
                align_flip = true;
            end
        end
        if align_flip
            normal_xy = -normal_xy;
        end
    end

    normal_valid = all(isfinite(normal_xy), 2);
    if ~any(normal_valid)
        return;
    end

    yaw_valid = atan2(normal_xy(normal_valid, 2), normal_xy(normal_valid, 1));
    if nargin >= 4 && numel(yaw_ref_series) == numel(time)
        yaw_ref_valid = yaw_ref_series(valid);
        yaw_ref_valid = yaw_ref_valid(normal_valid);
        yaw_valid = local_wrap_angles_near_reference(yaw_valid, yaw_ref_valid);
    end
    yaw_valid = unwrap(yaw_valid);

    yaw_tmp = nan(size(t));
    yaw_tmp(normal_valid) = yaw_valid;
    yaw_series(valid) = yaw_tmp;
end

function angles_out = local_wrap_angles_near_reference(angles_in, ref_angles)
    angles_out = angles_in;
    valid = isfinite(angles_in) & isfinite(ref_angles);
    if ~any(valid)
        return;
    end

    delta = atan2(sin(angles_in(valid) - ref_angles(valid)), cos(angles_in(valid) - ref_angles(valid)));
    angles_out(valid) = ref_angles(valid) + delta;
end

function local_plot_ee_normal_xz_defense(ax, time, ee_pos, normal_world, segment_xlim, time_origin, ...
    normal_est_norm, proxy_normal_series, proxy_rmse_xyz, arrow_count, arrow_scale)
    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'X [m]');
    ylabel(ax, 'Y [m]');
    zlabel(ax, 'Z [m]');

    if numel(segment_xlim) ~= 2 || ~all(isfinite(segment_xlim))
        title(ax, 'EE + normal (split disabled)');
        text(ax, 0.5, 0.5, 'Set panelm045_split_time inside panelm045_xlim to create the second segment.', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center', 'Color', [0.3 0.3 0.3]);
        return;
    end

    segment_mask = local_segment_mask(time, segment_xlim);
    ee_masked = ee_pos;
    normal_masked = normal_world;
    ee_masked(~segment_mask, :) = nan;
    normal_masked(~segment_mask, :) = nan;
    valid = all(isfinite(ee_masked), 2);

    segment_title = sprintf('EE + normal (t = %.1f to %.1f s)', ...
        segment_xlim(1) - time_origin, segment_xlim(2) - time_origin);
    title(ax, segment_title);

    if ~any(valid)
        text(ax, 0.5, 0.5, 'EE position is not available in this segment.', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
        return;
    end

    time_valid = time(valid);
    ee_valid = ee_masked(valid,:);
    [color_valid, color_min, color_max] = local_prepare_time_spectrum_data(time_valid, size(ee_valid,1), time_origin);
    local_plot_spectrum_trajectory3(ax, ee_valid, color_valid, 2.2);
    scatter3(ax, ee_valid(1,1), ee_valid(1,2), ee_valid(1,3), 52, [0.30 0.30 0.30], 'filled', 'MarkerEdgeColor', 'k');
    scatter3(ax, ee_valid(end,1), ee_valid(end,2), ee_valid(end,3), 52, [0.85 0.33 0.10], 'filled', 'MarkerEdgeColor', 'k');
    colormap(ax, local_make_ee_spectrum_colormap(256));
    cb = colorbar(ax);
    cb.Label.String = 'relative time [s]';
    caxis(ax, [color_min color_max]);

    quiver_idx = find(valid & all(isfinite(normal_masked), 2));
    arrow_count = max(1, round(arrow_count));
    quiver_step = max(1, floor(numel(quiver_idx) / arrow_count));
    quiver_idx = quiver_idx(1:quiver_step:end);
    if ~isempty(quiver_idx)
        quiver3(ax, ee_masked(quiver_idx,1), ee_masked(quiver_idx,2), ee_masked(quiver_idx,3), ...
            normal_masked(quiver_idx,1), normal_masked(quiver_idx,2), normal_masked(quiver_idx,3), arrow_scale, ...
            'Color', [0.10 0.65 0.20], 'LineWidth', 1.4, 'MaxHeadSize', 2.2);
    end

    spatial_points = ee_valid;
    if ~isempty(quiver_idx)
        spatial_points = [spatial_points; ...
            ee_masked(quiver_idx,:); ...
            ee_masked(quiver_idx,:) + arrow_scale * normal_masked(quiver_idx,:)];
    end
    local_set_spatial_axis_limits(ax, spatial_points, 0.10, 0.005);
    view(ax, 2);
    legend(ax, {'EE trajectory', 'start', 'end', 'normal est'}, 'Location', 'best');

end

function local_plot_ee_normal_iso_defense(ax, time, ee_pos, normal_world, segment_xlim, time_origin, ...
    arrow_count, arrow_scale)
    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'X [m]');
    ylabel(ax, 'Y [m]');
    zlabel(ax, 'Z [m]');

    if numel(segment_xlim) ~= 2 || ~all(isfinite(segment_xlim))
        title(ax, 'EE + normal iso (split disabled)');
        text(ax, 0.5, 0.5, 'Set panelm045_split_time inside panelm045_xlim to create the second segment.', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center', 'Color', [0.3 0.3 0.3]);
        return;
    end

    segment_mask = local_segment_mask(time, segment_xlim);
    ee_masked = ee_pos;
    normal_masked = normal_world;
    ee_masked(~segment_mask, :) = nan;
    normal_masked(~segment_mask, :) = nan;
    valid = all(isfinite(ee_masked), 2);

    segment_title = sprintf('EE + normal iso (t = %.1f to %.1f s)', ...
        segment_xlim(1) - time_origin, segment_xlim(2) - time_origin);
    title(ax, segment_title);

    if ~any(valid)
        text(ax, 0.5, 0.5, 'EE position is not available in this segment.', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
        return;
    end

    time_valid = time(valid);
    ee_valid = ee_masked(valid,:);
    [color_valid, color_min, color_max] = local_prepare_time_spectrum_data(time_valid, size(ee_valid,1), time_origin);
    local_plot_spectrum_trajectory3(ax, ee_valid, color_valid, 2.2);
    scatter3(ax, ee_valid(1,1), ee_valid(1,2), ee_valid(1,3), 52, ...
        [0.30 0.30 0.30], 'filled', 'MarkerEdgeColor', 'k');
    scatter3(ax, ee_valid(end,1), ee_valid(end,2), ee_valid(end,3), 52, ...
        [0.85 0.33 0.10], 'filled', 'MarkerEdgeColor', 'k');
    colormap(ax, local_make_ee_spectrum_colormap(256));
    cb = colorbar(ax);
    cb.Label.String = 'relative time [s]';
    caxis(ax, [color_min color_max]);

    quiver_idx = find(valid & all(isfinite(normal_masked), 2));
    arrow_count = max(1, round(arrow_count));
    quiver_step = max(1, floor(numel(quiver_idx) / arrow_count));
    quiver_idx = quiver_idx(1:quiver_step:end);
    if ~isempty(quiver_idx)
        quiver3(ax, ee_masked(quiver_idx,1), ee_masked(quiver_idx,2), ee_masked(quiver_idx,3), ...
            normal_masked(quiver_idx,1), normal_masked(quiver_idx,2), normal_masked(quiver_idx,3), ...
            arrow_scale, 'Color', [0.10 0.65 0.20], 'LineWidth', 1.4, 'MaxHeadSize', 2.2);
    end

    local_set_spatial_axis_limits(ax, ee_valid, 0.10, 0.005);
    view(ax, 3);
    legend(ax, {'EE trajectory', 'start', 'end', 'normal est'}, 'Location', 'best');
end

function local_plot_ee_normal_yz_defense(ax, time, ee_pos, normal_world, segment_xlim, time_origin, ...
    normal_est_norm, proxy_normal_series, proxy_rmse_xyz, arrow_count, arrow_scale)
    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'Y [m]');
    ylabel(ax, 'Z [m]');

    if numel(segment_xlim) ~= 2 || ~all(isfinite(segment_xlim))
        title(ax, 'EE + normal (split disabled)');
        text(ax, 0.5, 0.5, 'Set panelm045_split_time inside panelm045_xlim to create the second segment.', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center', 'Color', [0.3 0.3 0.3]);
        return;
    end

    segment_mask = local_segment_mask(time, segment_xlim);
    ee_masked = ee_pos;
    normal_masked = normal_world;
    ee_masked(~segment_mask, :) = nan;
    normal_masked(~segment_mask, :) = nan;
    valid = all(isfinite(ee_masked), 2);

    segment_title = sprintf('EE + normal (t = %.1f to %.1f s)', ...
        segment_xlim(1) - time_origin, segment_xlim(2) - time_origin);
    title(ax, segment_title);

    if ~any(valid)
        text(ax, 0.5, 0.5, 'EE position is not available in this segment.', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
        return;
    end

    time_valid = time(valid);
    ee_valid = ee_masked(valid,:);
    [color_valid, color_min, color_max] = local_prepare_time_spectrum_data(time_valid, size(ee_valid,1), time_origin);
    local_plot_spectrum_trajectory2(ax, ee_valid(:, [2 3]), color_valid, 2.2);
    scatter(ax, ee_valid(1,2), ee_valid(1,3), 52, [0.30 0.30 0.30], 'filled', 'MarkerEdgeColor', 'k');
    scatter(ax, ee_valid(end,2), ee_valid(end,3), 52, [0.85 0.33 0.10], 'filled', 'MarkerEdgeColor', 'k');
    colormap(ax, local_make_ee_spectrum_colormap(256));
    cb = colorbar(ax);
    cb.Label.String = 'relative time [s]';
    caxis(ax, [color_min color_max]);

    quiver_idx = find(valid & all(isfinite(normal_masked), 2));
    arrow_count = max(1, round(arrow_count));
    quiver_step = max(1, floor(numel(quiver_idx) / arrow_count));
    quiver_idx = quiver_idx(1:quiver_step:end);
    if ~isempty(quiver_idx)
        quiver(ax, ee_masked(quiver_idx,2), ee_masked(quiver_idx,3), ...
            normal_masked(quiver_idx,2), normal_masked(quiver_idx,3), arrow_scale, ...
            'Color', [0.10 0.65 0.20], 'LineWidth', 1.4, 'MaxHeadSize', 2.2);
    end

    local_set_planar_axis_limits(ax, ee_valid(:, [2 3]));
    legend(ax, {'EE trajectory', 'start', 'end', 'normal est (YZ proj)'}, 'Location', 'best');
end

function local_plot_ee_normal_xz_plane_defense(ax, time, ee_pos, normal_world, segment_xlim, time_origin, ...
    normal_est_norm, proxy_normal_series, proxy_rmse_xyz, arrow_count, arrow_scale)
    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'X [m]');
    ylabel(ax, 'Z [m]');

    if numel(segment_xlim) ~= 2 || ~all(isfinite(segment_xlim))
        title(ax, 'EE + normal (split disabled)');
        text(ax, 0.5, 0.5, 'Set panelm045_split_time inside panelm045_xlim to create the second segment.', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center', 'Color', [0.3 0.3 0.3]);
        return;
    end

    segment_mask = local_segment_mask(time, segment_xlim);
    ee_masked = ee_pos;
    normal_masked = normal_world;
    ee_masked(~segment_mask, :) = nan;
    normal_masked(~segment_mask, :) = nan;
    valid = all(isfinite(ee_masked), 2);

    segment_title = sprintf('EE + normal (t = %.1f to %.1f s)', ...
        segment_xlim(1) - time_origin, segment_xlim(2) - time_origin);
    title(ax, segment_title);

    if ~any(valid)
        text(ax, 0.5, 0.5, 'EE position is not available in this segment.', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
        return;
    end

    time_valid = time(valid);
    ee_valid = ee_masked(valid,:);
    [color_valid, color_min, color_max] = local_prepare_time_spectrum_data(time_valid, size(ee_valid,1), time_origin);
    local_plot_spectrum_trajectory2(ax, ee_valid(:, [1 3]), color_valid, 2.2);
    scatter(ax, ee_valid(1,1), ee_valid(1,3), 52, [0.30 0.30 0.30], 'filled', 'MarkerEdgeColor', 'k');
    scatter(ax, ee_valid(end,1), ee_valid(end,3), 52, [0.85 0.33 0.10], 'filled', 'MarkerEdgeColor', 'k');
    colormap(ax, local_make_ee_spectrum_colormap(256));
    cb = colorbar(ax);
    cb.Label.String = 'relative time [s]';
    caxis(ax, [color_min color_max]);

    quiver_idx = find(valid & all(isfinite(normal_masked), 2));
    arrow_count = max(1, round(arrow_count));
    quiver_step = max(1, floor(numel(quiver_idx) / arrow_count));
    quiver_idx = quiver_idx(1:quiver_step:end);
    if ~isempty(quiver_idx)
        quiver(ax, ee_masked(quiver_idx,1), ee_masked(quiver_idx,3), ...
            normal_masked(quiver_idx,1), normal_masked(quiver_idx,3), arrow_scale, ...
            'Color', [0.10 0.65 0.20], 'LineWidth', 1.4, 'MaxHeadSize', 2.2);
    end

    local_set_xz_normal_focus_limits(ax, ee_valid(:, [1 3]), normal_masked(valid, [1 3]), arrow_scale);
    legend(ax, {'EE trajectory', 'start', 'end', 'normal est (XZ proj)'}, 'Location', 'best');
end

function local_plot_ee_normal_xz(ax, time, ee_pos, normal_world, xlim_time)
    valid_time = true(size(time));
    if numel(xlim_time) == 2 && all(isfinite(xlim_time))
        valid_time = time >= xlim_time(1) & time <= xlim_time(2);
    end

    ee_masked = ee_pos;
    normal_masked = normal_world;
    ee_masked(~valid_time, :) = nan;
    normal_masked(~valid_time, :) = nan;
    valid = all(isfinite(ee_masked), 2);

    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'X [m]');
    ylabel(ax, 'Z [m]');
    title(ax, 'Normal estimation and EE trajectory (XZ plane)');

    if ~any(valid)
        text(ax, 0.5, 0.5, 'EE position is not available', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
        return;
    end

    time_valid = time(valid);
    ee_valid = ee_masked(valid,:);
    if numel(time_valid) >= 2
        time_origin = [];
        if numel(xlim_time) == 2 && all(isfinite(xlim_time))
            time_origin = xlim_time(1);
        end
        [cdata, color_min, color_max] = local_prepare_time_spectrum_data(time_valid, size(ee_valid,1), time_origin);
        cmap = local_make_ee_spectrum_colormap(256);
        surface(ax, ...
            [ee_valid(:,1), ee_valid(:,1)], ...
            [ee_valid(:,3), ee_valid(:,3)], ...
            zeros(size(ee_valid,1), 2), ...
            [cdata(:), cdata(:)], ...
            'FaceColor', 'none', 'EdgeColor', 'interp', 'LineWidth', 2.0);
        scatter(ax, ee_valid(1,1), ee_valid(1,3), 42, cdata(1), 'filled', 'MarkerEdgeColor', 'k');
        scatter(ax, ee_valid(end,1), ee_valid(end,3), 42, cdata(end), 'filled', 'MarkerEdgeColor', 'k');
        colormap(ax, cmap);
        cb = colorbar(ax);
        cb.Label.String = 'relative time [s]';
        caxis(ax, [color_min color_max]);
    else
        plot(ax, ee_valid(:,1), ee_valid(:,3), 'LineWidth', 1.6, 'Color', [0.0000 0.4470 0.7410]);
    end

    quiver_idx = find(valid & all(isfinite(normal_masked), 2));
    quiver_step = max(1, floor(numel(quiver_idx) / 25));
    quiver_idx = quiver_idx(1:quiver_step:end);
    if ~isempty(quiver_idx)
        quiver(ax, ee_masked(quiver_idx,1), ee_masked(quiver_idx,3), ...
            normal_masked(quiver_idx,1), normal_masked(quiver_idx,3), 0.15, ...
            'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.0, 'MaxHeadSize', 1.5);
    end

    local_set_planar_axis_limits(ax, ee_valid(:, [1 3]));
    legend(ax, {'EE trajectory', 'start', 'end', 'normal est (XZ proj)'}, 'Location', 'best');
end

function local_set_planar_axis_limits(ax, planar_points, pad_ratio, min_pad)
    if nargin < 3
        [x_limits, y_limits, ok] = local_compute_planar_axis_limits(planar_points);
    elseif nargin < 4
        [x_limits, y_limits, ok] = local_compute_planar_axis_limits(planar_points, pad_ratio);
    else
        [x_limits, y_limits, ok] = local_compute_planar_axis_limits(planar_points, pad_ratio, min_pad);
    end
    if ~ok
        return;
    end

    xlim(ax, x_limits);
    ylim(ax, y_limits);
end

function [x_limits, y_limits, ok] = local_compute_planar_axis_limits(planar_points, pad_ratio, min_pad)
    x_limits = [nan, nan];
    y_limits = [nan, nan];
    ok = false;

    if isempty(planar_points) || size(planar_points, 2) ~= 2
        return;
    end
    if nargin < 3 || ~isfinite(pad_ratio)
        pad_ratio = 0.05;
    end
    if nargin < 4 || ~isfinite(min_pad)
        min_pad = 0.01;
    end

    valid = all(isfinite(planar_points), 2);
    planar_points = planar_points(valid, :);
    if isempty(planar_points)
        return;
    end

    mins = min(planar_points, [], 1);
    maxs = max(planar_points, [], 1);
    spans = maxs - mins;
    center = 0.5 * (mins + maxs);
    max_span = max(spans);
    if ~isfinite(max_span) || max_span < 1.0e-6
        max_span = 0.05;
    end
    pads = max(pad_ratio * max_span, min_pad);
    half_range = 0.5 * max_span + pads;

    x_limits = center(1) + [-half_range, half_range];
    y_limits = center(2) + [-half_range, half_range];
    ok = true;
end

function local_set_spatial_z_limits(ax, z_points, origin_z, normal_z, normal_scale)
    if nargin < 5 || ~isfinite(normal_scale)
        normal_scale = 0.25;
    end

    z_all = z_points(:);
    if ~isempty(origin_z) && ~isempty(normal_z)
        z_all = [z_all; origin_z(:) + normal_scale * normal_z(:)];
    end
    z_all = z_all(isfinite(z_all));
    if isempty(z_all)
        return;
    end

    z_min = min(z_all);
    z_max = max(z_all);
    z_span = z_max - z_min;
    if ~isfinite(z_span) || z_span < 1.0e-6
        z_span = 0.05;
    end
    z_pad = max(0.02, 0.15 * z_span);
    zlim(ax, [z_min - z_pad, z_max + z_pad]);
end

function local_set_xz_normal_focus_limits(ax, origin_xz, normal_xz, normal_scale)
    if nargin < 4 || ~isfinite(normal_scale)
        normal_scale = 0.25;
    end

    if isempty(origin_xz) || size(origin_xz, 2) ~= 2
        return;
    end

    valid_origin = all(isfinite(origin_xz), 2);
    origin_xz = origin_xz(valid_origin, :);
    if isempty(origin_xz)
        return;
    end

    center_xz = mean(origin_xz, 1, 'omitnan');
    all_points = origin_xz;

    if ~isempty(normal_xz) && size(normal_xz, 2) == 2
        n_pair = min(size(origin_xz, 1), size(normal_xz, 1));
        pair_origin = origin_xz(1:n_pair, :);
        pair_normal = normal_xz(1:n_pair, :);
        joint_valid = all(isfinite(pair_origin), 2) & all(isfinite(pair_normal), 2);
        pair_origin = pair_origin(joint_valid, :);
        pair_normal = pair_normal(joint_valid, :);
        if ~isempty(pair_origin)
            tip_xz = pair_origin + normal_scale * pair_normal;
            all_points = [all_points; tip_xz];
        end
    end

    valid_points = all(isfinite(all_points), 2);
    all_points = all_points(valid_points, :);
    if isempty(all_points) || any(~isfinite(center_xz))
        return;
    end

    spans = max(all_points, [], 1) - min(all_points, [], 1);
    max_span = max(spans);
    if ~isfinite(max_span) || max_span < 1.0e-6
        max_span = 0.05;
    end
    half_range = 0.5 * max_span + max(0.12 * max_span, 0.01);

    xlim(ax, center_xz(1) + [-half_range, half_range]);
    ylim(ax, center_xz(2) + [-half_range, half_range]);
end

function local_add_view_preset_buttons(fig_handle, ax_handle)
    if ~isgraphics(fig_handle, 'figure') || ~isgraphics(ax_handle, 'axes')
        return;
    end

    set(fig_handle, 'KeyPressFcn', @(src, evt) local_handle_view_preset_key(src, evt, ax_handle));

    button_specs = {
        'XY',  [0, 90]
        'YZ',  [90, 0]
        'XZ',  [0, 0]
        'ISO', [-37.5, 30]
    };
    x0 = 0.69;
    y0 = 0.94;
    w = 0.07;
    h = 0.045;
    gap = 0.008;

    for k = 1:size(button_specs, 1)
        label = button_specs{k, 1};
        azel = button_specs{k, 2};
        uicontrol(fig_handle, 'Style', 'pushbutton', ...
            'String', label, ...
            'Units', 'normalized', ...
            'Position', [x0 + (k - 1) * (w + gap), y0, w, h], ...
            'Callback', @(~,~) view(ax_handle, azel), ...
            'BackgroundColor', [0.96 0.96 0.96], ...
            'FontSize', 9);
    end
end

function local_handle_view_preset_key(~, evt, ax_handle)
    if ~isgraphics(ax_handle, 'axes') || isempty(evt) || ~isfield(evt, 'Key')
        return;
    end

    switch evt.Key
        case '1'
            view(ax_handle, [0, 90]);
        case '2'
            view(ax_handle, [90, 0]);
        case '3'
            view(ax_handle, [0, 0]);
        case '4'
            view(ax_handle, [-37.5, 30]);
        otherwise
            return;
    end
end

function local_set_spatial_axis_limits(ax, points_xyz, pad_ratio, min_pad)
    if isempty(points_xyz) || size(points_xyz, 2) ~= 3
        return;
    end
    if nargin < 3 || ~isfinite(pad_ratio)
        pad_ratio = 0.10;
    end
    if nargin < 4 || ~isfinite(min_pad)
        min_pad = 0.01;
    end

    valid = all(isfinite(points_xyz), 2);
    points_xyz = points_xyz(valid, :);
    if isempty(points_xyz)
        return;
    end

    mins = min(points_xyz, [], 1);
    maxs = max(points_xyz, [], 1);
    spans = maxs - mins;
    center = 0.5 * (mins + maxs);
    max_span = max(spans);
    if ~isfinite(max_span) || max_span < 1.0e-6
        max_span = 0.05;
    end
    pads = max(pad_ratio * max_span, min_pad);
    half_range = 0.5 * max_span + pads;

    xlim(ax, center(1) + [-half_range, half_range]);
    ylim(ax, center(2) + [-half_range, half_range]);
    zlim(ax, center(3) + [-half_range, half_range]);
end

function local_handle_ee_xy_click(src, ~)
    ax = ancestor(src, 'axes');
    if isempty(ax) || ~isfield(ax.UserData, 'ee_xy_pos') || ~isfield(ax.UserData, 'ee_xy_time')
        return;
    end

    cp = ax.CurrentPoint;
    click_xy = cp(1, 1:2);
    ee_pos = ax.UserData.ee_xy_pos;
    ee_time = ax.UserData.ee_xy_time;
    ee_time_display = ee_time;
    if isfield(ax.UserData, 'ee_xy_time_display')
        ee_time_display = ax.UserData.ee_xy_time_display;
    end
    cdata = ax.UserData.ee_xy_cdata;
    cmap = ax.UserData.ee_xy_cmap;

    diff_xy = ee_pos(:,1:2) - click_xy;
    [~, idx] = min(sum(diff_xy.^2, 2, 'omitnan'));
    if isempty(idx) || ~isfinite(idx)
        return;
    end

    delete(findobj(ax, 'Tag', 'ee_xy_selected_point'));
    delete(findobj(ax, 'Tag', 'ee_xy_selected_text'));

    cmin = min(cdata, [], 'omitnan');
    cmax = max(cdata, [], 'omitnan');
    if ~isfinite(cmin) || ~isfinite(cmax) || abs(cmax - cmin) < 1.0e-9
        color_frac = 0.5;
    else
        color_frac = (cdata(idx) - cmin) / (cmax - cmin);
    end
    color_idx = max(1, min(size(cmap, 1), 1 + round(color_frac * (size(cmap,1) - 1))));
    selected_color = cmap(color_idx, :);
    scatter3(ax, ee_pos(idx,1), ee_pos(idx,2), ee_pos(idx,3), 80, selected_color, ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.2, 'Tag', 'ee_xy_selected_point');
    text(ax, ee_pos(idx,1), ee_pos(idx,2), ee_pos(idx,3), sprintf('  t=%.2fs', ee_time_display(idx)), ...
        'Color', [0.1 0.1 0.1], 'FontWeight', 'bold', 'VerticalAlignment', 'bottom', ...
        'Tag', 'ee_xy_selected_text');

    if isfield(ax.UserData, 'ee_xy_title')
        title(ax, sprintf('%s | selected t = %.2f s', ax.UserData.ee_xy_title, ee_time_display(idx)));
    end
end

function [t1_rows, t2_rows] = local_compute_normal_frame_tangents(normal_rows)
    n = size(normal_rows, 1);
    t1_rows = nan(n, 3);
    t2_rows = nan(n, 3);
    z_axis = [0.0, 0.0, 1.0];
    y_axis = [0.0, 1.0, 0.0];

    for k = 1:n
        x_axis = normal_rows(k,:);
        if any(~isfinite(x_axis))
            continue;
        end
        x_norm = norm(x_axis);
        if x_norm < 1.0e-6
            continue;
        end
        x_axis = x_axis / x_norm;

        t1 = cross(z_axis, x_axis);
        if norm(t1) < 1.0e-6
            t1 = cross(y_axis, x_axis);
        end
        if norm(t1) < 1.0e-6
            continue;
        end
        t1 = t1 / norm(t1);

        t2 = cross(x_axis, t1);
        if norm(t2) < 1.0e-6
            continue;
        end
        t2 = t2 / norm(t2);

        t1_rows(k,:) = t1;
        t2_rows(k,:) = t2;
    end
end

function proj = local_project_rows(v_rows, axis_rows)
    proj = nan(size(v_rows, 1), 1);
    valid = all(isfinite(v_rows), 2) & all(isfinite(axis_rows), 2);
    proj(valid) = sum(v_rows(valid,:) .* axis_rows(valid,:), 2);
end
