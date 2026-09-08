%% data_debug.m
% Debug-only reader and plotting dashboard for suWrenchObs SI logging.
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
mob_force_final_yaw_body = local_rotate_world_to_yaw_body(mob_force_final, pose_rpy(:,3));
force_measured_x_for_control_yaw_body = -mob_force_final_yaw_body(:,1);
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

%% 5.1) Figure: MOB Force vs Desired
mob_force_desired_xlim = [38 82];
mob_force_desired_yaw_lpf_hz = 0.05;  % [] or <=0 disables phase-delay-free LPF
mob_force_desired_x_ylim = [0 0.06];
mob_force_desired_y_ylim = [-0.03 0.03];
mob_force_desired_z_ylim = [-0.03 0.03];
mob_force_desired_yaw_ylim = [];       % e.g. [-0.5 0.5], [] disables fixed ylim
mob_force_desired_pos_x_ylim = [0 1.8];     % e.g. [0.0 1.5], [] disables fixed ylim
mob_force_desired_pos_y_ylim = [-0.8 1.0];     % e.g. [-0.5 0.5], [] disables fixed ylim
mob_force_desired_axis_ylims = {
    mob_force_desired_x_ylim
    mob_force_desired_y_ylim
    mob_force_desired_z_ylim
};

f_mob_desired = figure('Name', 'MOB Force vs Desired', 'NumberTitle', 'off', 'Color', 'w');
tiledlayout(f_mob_desired, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
mob_force_desired_axes = gobjects(0);
mob_force_desired_yaw_raw = pose_rpy(:,3);
mob_force_desired_yaw_lpf = local_lowpass_zero_phase(mob_force_desired_yaw_raw, sample_hz, mob_force_desired_yaw_lpf_hz);
mob_force_final_desired_yaw_body = local_rotate_world_to_yaw_body(mob_force_final, mob_force_desired_yaw_lpf);
mob_force_desired_yaw_body_plot = [-mob_force_final_desired_yaw_body(:,1), mob_force_final_desired_yaw_body(:,2), mob_force_final_desired_yaw_body(:,3)];
mob_force_desired_legends = {
    {'force desired', 'body -fHat_x'}
    {'body fHat_y'}
    {'body fHat_z'}
};
for i = 1:3
    ax = nexttile(2 * i - 1);
    mob_force_desired_axes(end+1) = ax;
    h_force = gobjects(0);
    if i == 1
        h_force(end+1) = plot(ax, time, force_desired_plot, '--', 'LineWidth', 2.2, 'Color', cmd_color); hold(ax, 'on');
    else
        hold(ax, 'on');
    end
    h_force(end+1) = plot(ax, time, mob_force_desired_yaw_body_plot(:,i), '-', 'LineWidth', 2.0, 'Color', meas_color);
    ylabel(ax, sprintf('%s [N]', axis_names{i}));
    if ~isempty(mob_force_desired_axis_ylims{i})
        ylim(ax, mob_force_desired_axis_ylims{i});
    end
    grid(ax, 'on');
    xlabel(ax, 'time [s]');
    title(ax, sprintf('MOB Force vs Desired %s', axis_names{i}));
    legend(ax, h_force, mob_force_desired_legends{i}, 'Location', 'best');
    if ~isempty(mob_force_desired_xlim)
        xlim(ax, mob_force_desired_xlim);
    end
end

ax_mob_desired_yaw = nexttile(2);
mob_force_desired_axes(end+1) = ax_mob_desired_yaw;
plot(ax_mob_desired_yaw, time, mob_force_desired_yaw_raw, ':', 'LineWidth', 1.2, 'Color', [0.45 0.45 0.45]); hold(ax_mob_desired_yaw, 'on');
plot(ax_mob_desired_yaw, time, mob_force_desired_yaw_lpf, 'LineWidth', 2.0, 'Color', pos_color);
grid(ax_mob_desired_yaw, 'on');
xlabel(ax_mob_desired_yaw, 'time [s]');
ylabel(ax_mob_desired_yaw, '\psi [rad]');
title(ax_mob_desired_yaw, 'Yaw Angle');
legend(ax_mob_desired_yaw, {'raw', sprintf('zero-phase LPF %.2f Hz', mob_force_desired_yaw_lpf_hz)}, 'Location', 'best');
if ~isempty(mob_force_desired_xlim)
    xlim(ax_mob_desired_yaw, mob_force_desired_xlim);
end
if ~isempty(mob_force_desired_yaw_ylim)
    ylim(ax_mob_desired_yaw, mob_force_desired_yaw_ylim);
end

ax_mob_desired_pos_x = nexttile(4);
mob_force_desired_axes(end+1) = ax_mob_desired_pos_x;
h_pos_x_meas = plot(ax_mob_desired_pos_x, time, pose_xyz(:,1), '-', 'LineWidth', 2.0, 'Color', meas_color); hold(ax_mob_desired_pos_x, 'on');
h_pos_x_des = plot(ax_mob_desired_pos_x, time, fw_cmd_xyz(:,1), '--', 'LineWidth', 2.6, 'Color', cmd_color);
grid(ax_mob_desired_pos_x, 'on');
xlabel(ax_mob_desired_pos_x, 'time [s]');
ylabel(ax_mob_desired_pos_x, 'x [m]');
title(ax_mob_desired_pos_x, 'Position X');
legend(ax_mob_desired_pos_x, [h_pos_x_des, h_pos_x_meas], {'desired', 'measured'}, 'Location', 'best');
if ~isempty(mob_force_desired_xlim)
    xlim(ax_mob_desired_pos_x, mob_force_desired_xlim);
end
if ~isempty(mob_force_desired_pos_x_ylim)
    ylim(ax_mob_desired_pos_x, mob_force_desired_pos_x_ylim);
end

ax_mob_desired_pos_y = nexttile(6);
mob_force_desired_axes(end+1) = ax_mob_desired_pos_y;
h_pos_y_meas = plot(ax_mob_desired_pos_y, time, pose_xyz(:,2), '-', 'LineWidth', 2.0, 'Color', meas_color); hold(ax_mob_desired_pos_y, 'on');
h_pos_y_des = plot(ax_mob_desired_pos_y, time, fw_cmd_xyz(:,2), '--', 'LineWidth', 2.6, 'Color', cmd_color);
grid(ax_mob_desired_pos_y, 'on');
xlabel(ax_mob_desired_pos_y, 'time [s]');
ylabel(ax_mob_desired_pos_y, 'y [m]');
title(ax_mob_desired_pos_y, 'Position Y');
legend(ax_mob_desired_pos_y, [h_pos_y_des, h_pos_y_meas], {'desired', 'measured'}, 'Location', 'best');
if ~isempty(mob_force_desired_xlim)
    xlim(ax_mob_desired_pos_y, mob_force_desired_xlim);
end
if ~isempty(mob_force_desired_pos_y_ylim)
    ylim(ax_mob_desired_pos_y, mob_force_desired_pos_y_ylim);
end
linkaxes(mob_force_desired_axes, 'x');

%% 5.8) Figure -0.45: requested 3x2 contact / MOB / normal dashboard
panelm045_xlim = [55 152];
panelm045_pos_ylim = {[-.0 1.6], [-0.4 0.4], [0.6 2.2]};
panelm045_force_ylim = {[-0.02 0.08], [-0.2 0.2], [-0.1 0.1]};
panelm045_mob_ylim = {[-0.08 0.08], [-0.08 0.08], [-0.12 0.04]};
panelm045_normal_ylim = {[-1.7 -0.3], [-1. 1.], [-0.8 0.6]};
panelm045_att_ylim = {[-0.1 0.1], [-0.1 0.1], [-1 1]};
panelm045_metric_ylim = {[], [], [0.10 1.02]};
panelm045_alpha_min = 1.0;
panelm045_leak_bar = 0.01;
panelm045_mob_force_none_yaw_body = local_rotate_world_to_yaw_body(mob_force_none, pose_rpy(:,3));
panelm045_mob_force_residual_yaw_body = local_rotate_world_to_yaw_body(mob_force_residual, pose_rpy(:,3));
panelm045_mob_force_final_yaw_body = mob_force_final_yaw_body;
panelm045_force_measured_x_for_control = force_measured_x_for_control_yaw_body;

alpha_leakage = panelm045_alpha_min + (1.0 - panelm045_alpha_min) ./ ...
    (1.0 + (normal_velocity_leakage_logged ./ panelm045_leak_bar).^2);
att_des_plot_m045 = att_des;
att_des_plot_m045(:,3) = unwrap(att_des_plot_m045(:,3));

fm045 = figure('Name', '3x2 Contact / MOB / Normal Dashboard', 'NumberTitle', 'off', ...
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
    plot(ax, time, fw_cmd_xyz(:,i), '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(ax, 'on');
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
        xlabel(ax, 'Time [s]', 'Interpreter', 'latex', 'FontSize', 12);
    else
        set(ax, 'XTickLabel', []);
    end
end

% (2,1) Force / t1 / t2 cmd vs measured
pm045_21 = uipanel('Parent', fm045, 'Position', getPos(2,1), 'BackgroundColor', 'w');
tlm045_21 = tiledlayout(pm045_21, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

axm045_force = nexttile(tlm045_21, 1);
plot(axm045_force, time, force_desired_plot, '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(axm045_force, 'on');
plot(axm045_force, time, force_measured_x_for_control, ':', 'LineWidth', 1.3, 'Color', [0.45 0.45 0.45]);
plot(axm045_force, time, panelm045_force_measured_x_for_control, '-', 'LineWidth', 1.7, 'Color', meas_color);
grid(axm045_force, 'on');
xlim(axm045_force, panelm045_xlim);
time_axes_m045(end+1) = axm045_force;
if ~isempty(panelm045_force_ylim{1})
    ylim(axm045_force, panelm045_force_ylim{1});
end
ylabel(axm045_force, '$f_n$ [N]', 'Interpreter', 'latex', 'FontSize', 12);
title(axm045_force, 'Force / Tangential Command');
legend(axm045_force, {'cmd', 'measured world', 'measured yaw-body'}, 'Location', 'best');
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
xlabel(axm045_t2, 'Time [s]', 'Interpreter', 'latex', 'FontSize', 12);
ylabel(axm045_t2, '$t_2$ [m/s]', 'Interpreter', 'latex', 'FontSize', 12);

% (1,2) Momentum observer pure vs consistency
pm045_12 = uipanel('Parent', fm045, 'Position', getPos(1,2), 'BackgroundColor', 'w');
tlm045_12 = tiledlayout(pm045_12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile(tlm045_12, i);
    plot(ax, time, mob_force_none(:,i), ':', 'LineWidth', 1.1, 'Color', [0.55 0.35 0.25]); hold(ax, 'on');
    plot(ax, time, panelm045_mob_force_none_yaw_body(:,i), '-', 'LineWidth', 1.8, 'Color', meas_color);
    plot(ax, time, mob_force_residual(:,i), ':', 'LineWidth', 1.1, 'Color', [0.35 0.45 0.60]);
    plot(ax, time, panelm045_mob_force_residual_yaw_body(:,i), '-', 'LineWidth', 1.7, 'Color', cmd_color);
    grid(ax, 'on');
    xlim(ax, panelm045_xlim);
    time_axes_m045(end+1) = ax;
    if ~isempty(panelm045_mob_ylim{i})
        ylim(ax, panelm045_mob_ylim{i});
    end
    ylabel(ax, sprintf('$f_{%s}$ [N]', lower(axis_names{i})), 'Interpreter', 'latex', 'FontSize', 12);
    if i == 1
        title(ax, 'Momentum Observer (world vs yaw body)');
        legend(ax, {'pure world', 'pure yaw-body', 'consistency world', 'consistency yaw-body'}, 'Location', 'best');
    end
    if i == 3
        xlabel(ax, 'Time [s]', 'Interpreter', 'latex', 'FontSize', 12);
    else
        set(ax, 'XTickLabel', []);
    end
end

% (2,2) Normal estimation: final vs pre-projection
pm045_22 = uipanel('Parent', fm045, 'Position', getPos(2,2), 'BackgroundColor', 'w');
tlm045_22 = tiledlayout(pm045_22, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile(tlm045_22, i);
    plot(ax, time, normal_pre_logged(:,i), '-', 'LineWidth', 2.0, 'Color', cmd_color); hold(ax, 'on');
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
        legend(ax, {'pre-projection', 'final'}, 'Location', 'best');
    end
    if i == 3
        xlabel(ax, 'Time [s]', 'Interpreter', 'latex', 'FontSize', 12);
    else
        set(ax, 'XTickLabel', []);
    end
end

% (1,3) EE trajectory only
pm045_13 = uipanel('Parent', fm045, 'Position', getPos(1,3), 'BackgroundColor', 'w');
tlm045_13 = tiledlayout(pm045_13, 1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
axm045_xz = nexttile(tlm045_13, 1);
local_plot_ee_normal_xz(axm045_xz, time, ee_pos, normal_est_logged, panelm045_xlim);

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
xlabel(axm045_att_yaw, 'Time [s]', 'Interpreter', 'latex', 'FontSize', 12);
ylabel(axm045_att_yaw, '$\psi$ [rad]', 'Interpreter', 'latex', 'FontSize', 12);

% Separate normal metrics window
fm045_metrics = figure('Name', 'Normal Metrics', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.08 0.08 0.32 0.62]);
tlm045_metrics = tiledlayout(fm045_metrics, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

axm045_omega = nexttile(tlm045_metrics, 1);
plot(axm045_omega, time, omega_n_logged, '-', 'LineWidth', 1.8, 'Color', acc_color);
grid(axm045_omega, 'on');
xlim(axm045_omega, panelm045_xlim);
time_axes_m045(end+1) = axm045_omega;
if ~isempty(panelm045_metric_ylim{1})
    ylim(axm045_omega, panelm045_metric_ylim{1});
end
ylabel(axm045_omega, '$\omega_n$ [1/s]', 'Interpreter', 'latex', 'FontSize', 12);
title(axm045_omega, 'Normal Metrics');
set(axm045_omega, 'XTickLabel', []);

axm045_leak = nexttile(tlm045_metrics, 2);
plot(axm045_leak, time, normal_velocity_leakage_logged, '-', 'LineWidth', 1.8, 'Color', pos_color);
grid(axm045_leak, 'on');
xlim(axm045_leak, panelm045_xlim);
time_axes_m045(end+1) = axm045_leak;
if ~isempty(panelm045_metric_ylim{2})
    ylim(axm045_leak, panelm045_metric_ylim{2});
end
ylabel(axm045_leak, '$v_{\mathrm{leak}}$ [-]', 'Interpreter', 'latex', 'FontSize', 12);
set(axm045_leak, 'XTickLabel', []);

axm045_alpha = nexttile(tlm045_metrics, 3);
plot(axm045_alpha, time, alpha_leakage, '-', 'LineWidth', 1.8, 'Color', meas_color);
grid(axm045_alpha, 'on');
xlim(axm045_alpha, panelm045_xlim);
time_axes_m045(end+1) = axm045_alpha;
if ~isempty(panelm045_metric_ylim{3})
    ylim(axm045_alpha, panelm045_metric_ylim{3});
end
xlabel(axm045_alpha, 'Time [s]', 'Interpreter', 'latex', 'FontSize', 12);
ylabel(axm045_alpha, '$\alpha_{\mathrm{leak}}$ [-]', 'Interpreter', 'latex', 'FontSize', 12);

linkaxes(time_axes_m045, 'x');
setappdata(fm045, 'panelm045_time', time);
setappdata(fm045, 'panelm045_ee_pos', ee_pos);
setappdata(fm045, 'panelm045_normal_est', normal_est_logged);
setappdata(fm045, 'panelm045_normal_pre', normal_pre_logged);
setappdata(fm045, 'panelm045_xz_ax', axm045_xz);
setappdata(fm045, 'panelm045_sync_listener', addlistener(time_axes_m045(1), 'XLim', 'PostSet', ...
    @(~, evt)local_sync_panelm045_xz(evt.AffectedObject, fm045)));
set(findall(fm045, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

%% 5.7) Figure 0: contact-frame overview / normal + EE top view / MOB pure vs consistency
panel0_xlim = [40 103];
panel0_force_ylim = [-0.02 0.12];
panel0_t1_ylim = [-0.35 0.35];
panel0_t2_ylim = [-0.35 0.35];
panel0_mob_ylim = [-0.08 0.08];
panel0_timer_ylim = [];

f0 = figure('Name', 'Contact / Normal / MOB Overview', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.08 0.92 0.78]);

p0_left = uipanel('Parent', f0, 'Units', 'normalized', 'Position', [0.02 0.08 0.43 0.86], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
tl0_left = tiledlayout(p0_left, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax0_force = nexttile(tl0_left);
plot(ax0_force, time, force_desired_plot, '--', 'LineWidth', 1.25, 'Color', cmd_color); hold(ax0_force, 'on');
plot(ax0_force, time, force_measured_x_for_control, 'LineWidth', 1.2, 'Color', meas_color);
grid(ax0_force, 'on');
xlabel(ax0_force, 'time [s]');
ylabel(ax0_force, '[N]');
title(ax0_force, 'Force desired vs measured');
legend(ax0_force, {'force desired', 'measured = -fHatW_x'}, 'Location', 'best');
xlim(ax0_force, panel0_xlim);
ylim(ax0_force, panel0_force_ylim);

ax0_t1 = nexttile(tl0_left);
plot(ax0_t1, time, contact_t1_cmd, '--', 'LineWidth', 1.25, 'Color', cmd_color); hold(ax0_t1, 'on');
plot(ax0_t1, time, contact_t1_meas, 'LineWidth', 1.2, 'Color', meas_color);
grid(ax0_t1, 'on');
xlabel(ax0_t1, 'time [s]');
ylabel(ax0_t1, '[m/s]');
title(ax0_t1, 't1 frame velocity cmd vs measured');
legend(ax0_t1, {'cmd', 'measured'}, 'Location', 'best');
xlim(ax0_t1, panel0_xlim);
ylim(ax0_t1, panel0_t1_ylim);

ax0_t2 = nexttile(tl0_left);
plot(ax0_t2, time, contact_t2_cmd, '--', 'LineWidth', 1.25, 'Color', cmd_color); hold(ax0_t2, 'on');
plot(ax0_t2, time, contact_t2_meas, 'LineWidth', 1.2, 'Color', meas_color);
grid(ax0_t2, 'on');
xlabel(ax0_t2, 'time [s]');
ylabel(ax0_t2, '[m/s]');
title(ax0_t2, 't2 frame velocity cmd vs measured');
legend(ax0_t2, {'cmd', 'measured'}, 'Location', 'best');
xlim(ax0_t2, panel0_xlim);
ylim(ax0_t2, panel0_t2_ylim);

p0_right_top = uipanel('Parent', f0, 'Units', 'normalized', 'Position', [0.49 0.52 0.49 0.42], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
tl0_top = tiledlayout(p0_right_top, 1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax0_top_xy = nexttile(tl0_top);
local_plot_ee_normal_xy_3d(ax0_top_xy, time, ee_pos, normal_est_logged, panel0_xlim);

p0_right_bottom = uipanel('Parent', f0, 'Units', 'normalized', 'Position', [0.49 0.08 0.49 0.36], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
p0_mob = uipanel('Parent', p0_right_bottom, 'Units', 'normalized', 'Position', [0.00 0.00 0.49 1.00], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
tl0_mob = tiledlayout(p0_mob, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile(tl0_mob);
    plot(ax, time, mob_force_none(:,i), 'LineWidth', 1.2, 'Color', meas_color); hold(ax, 'on');
    plot(ax, time, mob_force_residual(:,i), '--', 'LineWidth', 1.2, 'Color', cmd_color);
    grid(ax, 'on');
    xlabel(ax, 'time [s]');
    ylabel(ax, sprintf('%s [N]', axis_names{i}));
    title(ax, sprintf('Momentum observer %s: pure vs consistency', axis_names{i}));
    legend(ax, {'pure', 'consistency'}, 'Location', 'best');
    xlim(ax, panel0_xlim);
    ylim(ax, panel0_mob_ylim);
end

p0_timer = uipanel('Parent', p0_right_bottom, 'Units', 'normalized', 'Position', [0.51 0.00 0.49 1.00], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
tl0_timer = tiledlayout(p0_timer, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax0_timer_last = nexttile(tl0_timer);
plot(ax0_timer_last, time, stabilizer_loop_dt_us, 'LineWidth', 1.2, 'Color', acc_color); hold(ax0_timer_last, 'on');
plot(ax0_timer_last, time, stabilizer_loop_dt_us_max, '--', 'LineWidth', 1.1, 'Color', pos_color);
grid(ax0_timer_last, 'on');
xlabel(ax0_timer_last, 'time [s]');
ylabel(ax0_timer_last, '[us]');
title(ax0_timer_last, 'Stabilizer loop elapsed time');
legend(ax0_timer_last, {'loopDtUs', 'loopDtUsMax'}, 'Location', 'best');
xlim(ax0_timer_last, panel0_xlim);
if ~isempty(panel0_timer_ylim)
    ylim(ax0_timer_last, panel0_timer_ylim);
end

ax0_timer_hist = nexttile(tl0_timer);
if any(isfinite(stabilizer_loop_dt_us))
    histogram(ax0_timer_hist, stabilizer_loop_dt_us(isfinite(stabilizer_loop_dt_us)), 50, ...
        'FaceColor', acc_color, 'EdgeColor', 'none');
else
    plot(ax0_timer_hist, nan, nan);
    text(ax0_timer_hist, 0.5, 0.5, 'loopDtUs unavailable in CSV', 'Units', 'normalized', ...
        'HorizontalAlignment', 'center', 'Color', [0.3 0.3 0.3]);
end
grid(ax0_timer_hist, 'on');
xlabel(ax0_timer_hist, 'loopDtUs [us]');
ylabel(ax0_timer_hist, 'count');
title(ax0_timer_hist, 'Stabilizer loop elapsed-time histogram');


%% 5.6.1) Figure -0.4: normal metrics + position
panelm04_xlim = [46 72];
panelm04_alpha_min = 0.10;
panelm04_omega_bar = 0.05;
panelm04_leak_bar = 0.007;
panelm04_pos_ylim = [];
panelm04_force_ylim = [];
panelm04_omega_ylim = [];
panelm04_leak_ylim = [];
panelm04_alpha_ylim = [panelm04_alpha_min 1.02];

omega_alpha = panelm04_alpha_min + (1.0 - panelm04_alpha_min) ./ ...
    (1.0 + (omega_n_logged ./ panelm04_omega_bar).^2);
leak_alpha = panelm04_alpha_min + (1.0 - panelm04_alpha_min) ./ ...
    (1.0 + (normal_velocity_leakage_logged ./ panelm04_leak_bar).^2);

fm04 = figure('Name', 'Debug Normal Metrics and Position', 'NumberTitle', 'off', 'Color', 'w', ...
    'Units', 'normalized', 'Position', [0.08 0.10 0.84 0.72]);

p04_left = uipanel('Parent', fm04, 'Units', 'normalized', 'Position', [0.04 0.08 0.52 0.86], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
tl04_left = tiledlayout(p04_left, 6, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile(tl04_left);
    plot(ax, time, pose_xyz(:,i), 'LineWidth', 1.2, 'Color', meas_color);
    grid(ax, 'on');
    xlabel(ax, 'time [s]');
    ylabel(ax, sprintf('%s [m]', axis_names{i}));
    title(ax, sprintf('Position %s', axis_names{i}));
    if ~isempty(panelm04_xlim)
        xlim(ax, panelm04_xlim);
    elseif ~isempty(summary_xlim)
        xlim(ax, summary_xlim);
    end
    if ~isempty(panelm04_pos_ylim)
        ylim(ax, panelm04_pos_ylim);
    end
end

for i = 1:3
    ax = nexttile(tl04_left);
    plot(ax, time, mob_force_final(:,i), 'LineWidth', 1.2, 'Color', cmd_color);
    grid(ax, 'on');
    xlabel(ax, 'time [s]');
    ylabel(ax, sprintf('%s [N]', axis_names{i}));
    title(ax, sprintf('Momentum observer force %s', axis_names{i}));
    if ~isempty(panelm04_xlim)
        xlim(ax, panelm04_xlim);
    elseif ~isempty(summary_xlim)
        xlim(ax, summary_xlim);
    end
    if ~isempty(panelm04_force_ylim)
        ylim(ax, panelm04_force_ylim);
    end
end

p04_right = uipanel('Parent', fm04, 'Units', 'normalized', 'Position', [0.60 0.08 0.36 0.86], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
p04_right_top = uipanel('Parent', p04_right, 'Units', 'normalized', 'Position', [0.00 0.52 1.00 0.48], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
tl04_top = tiledlayout(p04_right_top, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax04_top_xy = nexttile(tl04_top);
local_plot_ee_normal_xy_3d(ax04_top_xy, time, ee_pos, normal_est_logged, panelm04_xlim);
ax04_top_xz = nexttile(tl04_top);
local_plot_ee_normal_xz(ax04_top_xz, time, ee_pos, normal_est_logged, panelm04_xlim);

p04_right_bottom = uipanel('Parent', p04_right, 'Units', 'normalized', 'Position', [0.00 0.00 1.00 0.48], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
tl04_right = tiledlayout(p04_right_bottom, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax04_omega = nexttile(tl04_right);
plot(ax04_omega, time, omega_n_logged, 'LineWidth', 1.2, 'Color', acc_color);
grid(ax04_omega, 'on');
xlabel(ax04_omega, 'time [s]');
ylabel(ax04_omega, '[1/s]');
title(ax04_omega, '\omega_n');
if ~isempty(panelm04_xlim)
    xlim(ax04_omega, panelm04_xlim);
elseif ~isempty(summary_xlim)
    xlim(ax04_omega, summary_xlim);
end
if ~isempty(panelm04_omega_ylim)
    ylim(ax04_omega, panelm04_omega_ylim);
end

ax04_omega_alpha = nexttile(tl04_right);
plot(ax04_omega_alpha, time, omega_alpha, 'LineWidth', 1.2, 'Color', cmd_color);
grid(ax04_omega_alpha, 'on');
xlabel(ax04_omega_alpha, 'time [s]');
ylabel(ax04_omega_alpha, '[-]');
title(ax04_omega_alpha, sprintf('\\alpha_\\omega (\\alpha_{min}=%.2f, \\bar{\\omega}=%.2f)', ...
    panelm04_alpha_min, panelm04_omega_bar));
if ~isempty(panelm04_xlim)
    xlim(ax04_omega_alpha, panelm04_xlim);
elseif ~isempty(summary_xlim)
    xlim(ax04_omega_alpha, summary_xlim);
end
if ~isempty(panelm04_alpha_ylim)
    ylim(ax04_omega_alpha, panelm04_alpha_ylim);
end

ax04_leak = nexttile(tl04_right);
plot(ax04_leak, time, normal_velocity_leakage_logged, 'LineWidth', 1.2, 'Color', pos_color);
grid(ax04_leak, 'on');
xlabel(ax04_leak, 'time [s]');
ylabel(ax04_leak, '[-]');
title(ax04_leak, 'normal velocity leakage');
if ~isempty(panelm04_xlim)
    xlim(ax04_leak, panelm04_xlim);
elseif ~isempty(summary_xlim)
    xlim(ax04_leak, summary_xlim);
end
if ~isempty(panelm04_leak_ylim)
    ylim(ax04_leak, panelm04_leak_ylim);
end

ax04_leak_alpha = nexttile(tl04_right);
plot(ax04_leak_alpha, time, leak_alpha, 'LineWidth', 1.2, 'Color', meas_color);
grid(ax04_leak_alpha, 'on');
xlabel(ax04_leak_alpha, 'time [s]');
ylabel(ax04_leak_alpha, '[-]');
title(ax04_leak_alpha, sprintf('\\alpha_{leak} (\\alpha_{min}=%.2f, \\bar{v}_{leak}=%.2f)', ...
    panelm04_alpha_min, panelm04_leak_bar));
if ~isempty(panelm04_xlim)
    xlim(ax04_leak_alpha, panelm04_xlim);
elseif ~isempty(summary_xlim)
    xlim(ax04_leak_alpha, summary_xlim);
end
if ~isempty(panelm04_alpha_ylim)
    ylim(ax04_leak_alpha, panelm04_alpha_ylim);
end


%% 5.6) Figure -0.5: normal estimation compare
panelm05_xlim = [40  76];         % reuse accel/gyro time window
panelm05_normal_ylim = [];            % fallback for all world-normal subplots
panelm05_normal_x_ylim = [-1.7 -0.3];          % e.g. [-0.5 0.5]
panelm05_normal_y_ylim = [-0.7 0.7];          % e.g. [-0.5 0.5]
panelm05_normal_z_ylim = [-0.8 0.6];          % e.g. [0.5 1.1]
panelm05_normal_tilt_ylim = [];       % e.g. [0 20]
panelm05_normal_axis_ylims = {panelm05_normal_x_ylim, panelm05_normal_y_ylim, panelm05_normal_z_ylim};
normal_log_color = [0.8500 0.3250 0.0980];
normal_pre_color = [0.0000 0.4470 0.7410];
normal_post_color = [0.4660 0.6740 0.1880];
normal_log_valid = all(isfinite(normal_est_logged), 2);
normal_pre_valid = all(isfinite(normal_pre_logged), 2);
normal_post_valid = all(isfinite(normal_post_logged), 2);
true_normal_logged = repmat([-1.0, 0.0, 0.0], size(normal_est_logged, 1), 1);
logged_normal_pre_err = local_angle_between_unit_vectors(normal_pre_logged, true_normal_logged);
logged_normal_post_err = local_angle_between_unit_vectors(normal_post_logged, true_normal_logged);
logged_normal_tilt_err = local_angle_between_unit_vectors(normal_est_logged, true_normal_logged);
gt_normal_color = [0.2000 0.2000 0.2000];

fm05 = figure('Name', 'Debug Normal Estimation', 'NumberTitle', 'off', 'Color', 'w');
tiledlayout(fm05, 6, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    nexttile(4 * (i - 1) + 1, [2 1]);
    plot(time, true_normal_logged(:,i), '--', 'LineWidth', 1.1, 'Color', gt_normal_color); hold on;
    if any(normal_pre_valid)
        plot(time, normal_pre_logged(:,i), ':', 'LineWidth', 1.1, 'Color', normal_pre_color);
    end
    if any(normal_post_valid)
        plot(time, normal_post_logged(:,i), '-.', 'LineWidth', 1.1, 'Color', normal_post_color);
    end
    plot(time, normal_est_logged(:,i), 'LineWidth', 1.2, 'Color', normal_log_color);
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('n_%s [-]', lower(axis_names{i})));
    title(sprintf('Logged normal %s', axis_names{i}));
    legend({'ground truth', 'pre projection', 'post projection', 'final normal est'}, 'Location', 'best');
    if ~isempty(panelm05_xlim)
        xlim(panelm05_xlim);
    elseif ~isempty(summary_xlim)
        xlim(summary_xlim);
    end
    if ~isempty(panelm05_normal_axis_ylims{i})
        ylim(panelm05_normal_axis_ylims{i});
    elseif ~isempty(panelm05_normal_ylim)
        ylim(panelm05_normal_ylim);
    end
end

nexttile(2, [3 1]);
if any(normal_pre_valid)
    plot(time, logged_normal_pre_err, ':', 'LineWidth', 1.1, 'Color', normal_pre_color); hold on;
else
    hold on;
end
if any(normal_post_valid)
    plot(time, logged_normal_post_err, '-.', 'LineWidth', 1.1, 'Color', normal_post_color);
end
plot(time, logged_normal_tilt_err, 'LineWidth', 1.2, 'Color', normal_log_color);
grid on;
xlabel('time [s]');
ylabel('[deg]');
title('Logged normal error vs ground truth [-1, 0, 0]');
legend({'pre projection', 'post projection', 'final normal est'}, 'Location', 'best');
if ~isempty(panelm05_xlim)
    xlim(panelm05_xlim);
elseif ~isempty(summary_xlim)
    xlim(summary_xlim);
end
if ~isempty(panelm05_normal_tilt_ylim)
    ylim(panelm05_normal_tilt_ylim);
end

nexttile(8, [3 1]);
plot(time, mob_force_final(:,1), 'LineWidth', 1.2, 'Color', cmd_color); hold on;
plot(time, mob_force_final(:,2), '--', 'LineWidth', 1.2, 'Color', meas_color);
plot(time, mob_force_final(:,3), ':', 'LineWidth', 1.4, 'Color', [0.2 0.6 0.2]);
grid on;
xlabel('time [s]');
ylabel('[N]');
title('Momentum observer force used by control (final fHatW)');
legend({'fHatW_x', 'fHatW_y', 'fHatW_z'}, 'Location', 'best');
if ~isempty(panelm05_xlim)
    xlim(panelm05_xlim);
elseif ~isempty(summary_xlim)
    xlim(summary_xlim);
end

%% 5.5) Figure -1: accel/gyro inputs and offline attitude reconstruction
panelm1_xlim = [25 190];                % e.g. [0 10]
panelm1_acc_ylim = [];            % fallback for all raw-acc subplots
panelm1_gyro_ylim = [];           % fallback for all raw-gyro subplots
panelm1_rpy_ylim = [];            % fallback for all attitude subplots
panelm1_acc_x_ylim = [];          % e.g. [-1.5 1.5]
panelm1_acc_y_ylim = [];          % e.g. [-1.5 1.5]
panelm1_acc_z_ylim = [];          % e.g. [-1.5 1.5]
panelm1_gyro_x_ylim = [];         % e.g. [-100 100]
panelm1_gyro_y_ylim = [];         % e.g. [-100 100]
panelm1_gyro_z_ylim = [];         % e.g. [-100 100]
panelm1_rpy_x_ylim = [-0.1 0.1];          % e.g. [-0.5 0.5]
panelm1_rpy_y_ylim = [-0.1 0.1];          % e.g. [-0.5 0.5]
panelm1_rpy_z_ylim = [-0.1 0.1];          % e.g. [-3.14 3.14]

panelm1_acc_axis_ylims = {panelm1_acc_x_ylim, panelm1_acc_y_ylim, panelm1_acc_z_ylim};
panelm1_gyro_axis_ylims = {panelm1_gyro_x_ylim, panelm1_gyro_y_ylim, panelm1_gyro_z_ylim};
panelm1_rpy_axis_ylims = {panelm1_rpy_x_ylim, panelm1_rpy_y_ylim, panelm1_rpy_z_ylim};
fminus1 = figure('Name', 'Debug Acc/Gyro Inputs and Offline Attitude', 'NumberTitle', 'off', 'Color', 'w');
tiledlayout(fminus1, 3, 5, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    nexttile(5 * (i - 1) + 1);
    plot(time, acc_raw_body(:,i), '--', 'LineWidth', 0.9, 'Color', [0.6 0.6 0.6]); hold on;
    plot(time, acc_raw_body_for_recon(:,i), 'LineWidth', 1.2, 'Color', cmd_color);
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('%s [G]', axis_names{i}));
    title(sprintf('Accel %s (raw / LPF)', axis_names{i}));
    if ~isempty(panelm1_acc_lpf_hz) && isfinite(panelm1_acc_lpf_hz) && panelm1_acc_lpf_hz > 0
        legend({'raw', sprintf('LPF %.2f Hz', panelm1_acc_lpf_hz)}, 'Location', 'best');
    else
        legend({'raw', 'LPF off'}, 'Location', 'best');
    end
    if ~isempty(panelm1_xlim)
        xlim(panelm1_xlim);
    elseif ~isempty(summary_xlim)
        xlim(summary_xlim);
    end
    if ~isempty(panelm1_acc_axis_ylims{i})
        ylim(panelm1_acc_axis_ylims{i});
    elseif ~isempty(panelm1_acc_ylim)
        ylim(panelm1_acc_ylim);
    end

    nexttile(5 * (i - 1) + 2);
    plot(time, gyro_body(:,i), '--', 'LineWidth', 0.9, 'Color', [0.6 0.6 0.6]); hold on;
    plot(time, gyro_body_for_recon(:,i), 'LineWidth', 1.2, 'Color', [0.1 0.6 0.1]);
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('%s [deg/s]', axis_names{i}));
    title(sprintf('Gyro %s (raw / LPF)', axis_names{i}));
    if ~isempty(panelm1_gyro_lpf_hz) && isfinite(panelm1_gyro_lpf_hz) && panelm1_gyro_lpf_hz > 0
        legend({'raw', sprintf('LPF %.2f Hz', panelm1_gyro_lpf_hz)}, 'Location', 'best');
    else
        legend({'raw', 'LPF off'}, 'Location', 'best');
    end
    if ~isempty(panelm1_xlim)
        xlim(panelm1_xlim);
    elseif ~isempty(summary_xlim)
        xlim(summary_xlim);
    end
    if ~isempty(panelm1_gyro_axis_ylims{i})
        ylim(panelm1_gyro_axis_ylims{i});
    elseif ~isempty(panelm1_gyro_ylim)
        ylim(panelm1_gyro_ylim);
    end

    nexttile(5 * (i - 1) + 3);
    if i < 3
        plot(time, acc_from_raw_rpy(:,i), 'LineWidth', 1.2, 'Color', meas_color);
        grid on;
        xlabel('time [s]');
        ylabel(sprintf('%s [rad]', axis_names{i}));
        title(sprintf('Offline accel %s', axis_names{i}));
        if ~isempty(panelm1_xlim)
            xlim(panelm1_xlim);
        elseif ~isempty(summary_xlim)
            xlim(summary_xlim);
        end
        if ~isempty(panelm1_rpy_axis_ylims{i})
            ylim(panelm1_rpy_axis_ylims{i});
        elseif ~isempty(panelm1_rpy_ylim)
            ylim(panelm1_rpy_ylim);
        end
    else
        plot(time, nan(size(time)), 'LineWidth', 1.0, 'Color', meas_color);
        grid on;
        xlabel('time [s]');
        ylabel(sprintf('%s [rad]', axis_names{i}));
        title('Offline accel yaw (N/A)');
        text(0.5, 0.5, 'Yaw not observable from accel only', 'Units', 'normalized', ...
            'HorizontalAlignment', 'center', 'Color', [0.3 0.3 0.3]);
        if ~isempty(panelm1_xlim)
            xlim(panelm1_xlim);
        elseif ~isempty(summary_xlim)
            xlim(summary_xlim);
        end
    end

    nexttile(5 * (i - 1) + 4);
    plot(time, gyro_integrated_rpy(:,i), 'LineWidth', 1.2, 'Color', [0.4940 0.1840 0.5560]);
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('%s [rad]', axis_names{i}));
    title(sprintf('Offline gyro int %s', axis_names{i}));
    if ~isempty(panelm1_xlim)
        xlim(panelm1_xlim);
    elseif ~isempty(summary_xlim)
        xlim(summary_xlim);
    end
    if ~isempty(panelm1_rpy_axis_ylims{i})
        ylim(panelm1_rpy_axis_ylims{i});
    elseif ~isempty(panelm1_rpy_ylim)
        ylim(panelm1_rpy_ylim);
    end

    nexttile(5 * (i - 1) + 5);
    plot(time, pose_rpy(:,i), 'LineWidth', 1.2, 'Color', pos_color);
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('%s [rad]', axis_names{i}));
    title(sprintf('Computed attitude %s', axis_names{i}));
    if ~isempty(panelm1_xlim)
        xlim(panelm1_xlim);
    elseif ~isempty(summary_xlim)
        xlim(summary_xlim);
    end
    if ~isempty(panelm1_rpy_axis_ylims{i})
        ylim(panelm1_rpy_axis_ylims{i});
    elseif ~isempty(panelm1_rpy_ylim)
        ylim(panelm1_rpy_ylim);
    end
end

%% 6) Figure 0.5: top pose / command position / attitude compare panel
panel05_xlim = [40 70];               % e.g. [0 10]
panel05_pos_ylim = [];           % fallback for all position subplots
panel05_vel_ylim = [];           % fallback for all velocity subplots
panel05_att_ylim = [];           % fallback for all attitude subplots
panel05_pos_x_ylim = [-0.3 0.3];         % e.g. [-1 1]
panel05_pos_y_ylim = [-0.3 0.3];         % e.g. [-1 1]
panel05_pos_z_ylim = [0.4 1.0];         % e.g. [0 1]
panel05_vel_x_ylim = [-0.3 0.3];         % e.g. [-1 1]
panel05_vel_y_ylim = [-0.3 0.3];         % e.g. [-1 1]
panel05_vel_z_ylim = [-0.3 0.3];         % e.g. [-1 1]
panel05_att_x_ylim = [-0.1 0.1];         % e.g. [-0.5 0.5]
panel05_att_y_ylim = [-0.1 0.1];         % e.g. [-0.5 0.5]
panel05_att_z_ylim = [-0.5 0.5];         % e.g. [-3.14 3.14]

att_des_plot = att_des;
att_des_plot(:,3) = unwrap(att_des_plot(:,3));
yaw_ref_plot = unwrap(yaw_ref);

panel05_pos_axis_ylims = {panel05_pos_x_ylim, panel05_pos_y_ylim, panel05_pos_z_ylim};
panel05_vel_axis_ylims = {panel05_vel_x_ylim, panel05_vel_y_ylim, panel05_vel_z_ylim};
panel05_att_axis_ylims = {panel05_att_x_ylim, panel05_att_y_ylim, panel05_att_z_ylim};

f05 = figure('Name', 'Debug Pose / Command Position / Attitude Compare', 'NumberTitle', 'off', 'Color', 'w');
tiledlayout(f05, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    nexttile(3 * (i - 1) + 1);
    plot(time, pose_xyz(:,i), 'LineWidth', 1.8, 'Color', meas_color); hold on;
    plot(time, fw_cmd_xyz(:,i), '--', 'LineWidth', 1.8, 'Color', cmd_color);
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('%s [m]', axis_names{i}));
    title(sprintf('Position tracking %s', axis_names{i}));
    legend({'measured', 'fw final position'}, 'Location', 'best');
    if ~isempty(panel05_xlim)
        xlim(panel05_xlim);
    elseif ~isempty(summary_xlim)
        xlim(summary_xlim);
    end
    if ~isempty(panel05_pos_axis_ylims{i})
        ylim(panel05_pos_axis_ylims{i});
    elseif ~isempty(panel05_pos_ylim)
        ylim(panel05_pos_ylim);
    end

    nexttile(3 * (i - 1) + 2);
    plot(time, state_vel(:,i), 'LineWidth', 1.8, 'Color', meas_color); hold on;
    plot(time, vel_des(:,i), '--', 'LineWidth', 1.8, 'Color', cmd_color);
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('%s [m/s]', axis_names{i}));
    title(sprintf('Velocity tracking %s', axis_names{i}));
    legend({'measured', 'command'}, 'Location', 'best');
    if ~isempty(panel05_xlim)
        xlim(panel05_xlim);
    elseif ~isempty(summary_xlim)
        xlim(summary_xlim);
    end
    if ~isempty(panel05_vel_axis_ylims{i})
        ylim(panel05_vel_axis_ylims{i});
    elseif ~isempty(panel05_vel_ylim)
        ylim(panel05_vel_ylim);
    end

    nexttile(3 * (i - 1) + 3);
    plot(time, pose_rpy(:,i), 'LineWidth', 1.8, 'Color', meas_color); hold on;
    plot(time, att_des_plot(:,i), '--', 'LineWidth', 1.8, 'Color', cmd_color);
    if i == 3 && any(isfinite(yaw_ref_plot))
        plot(time, yaw_ref_plot, '-', 'LineWidth', 1.8, 'Color', pos_color);
        legend({'measured', 'att\_des', 'yawRef'}, 'Location', 'best');
    else
        legend({'measured', 'command'}, 'Location', 'best');
    end
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('%s [rad]', axis_names{i}));
    title(sprintf('Attitude tracking %s', axis_names{i}));
    if ~isempty(panel05_xlim)
        xlim(panel05_xlim);
    elseif ~isempty(summary_xlim)
        xlim(summary_xlim);
    end
    if ~isempty(panel05_att_axis_ylims{i})
        ylim(panel05_att_axis_ylims{i});
    elseif ~isempty(panel05_att_ylim)
        ylim(panel05_att_ylim);
    end
end

%% 7) Figure 1: MOB force compare / torque panel
panel1_xlim = [38 82];                % e.g. [0 10]
panel1_force_ylim = [-0.05 0.05];          % fallback for all MOB force subplots
panel1_torque_ylim = [-0.05 0.05];         % fallback for all MOB torque subplots
panel1_force_x_ylim = [-0.05 0.05];
panel1_force_y_ylim = [-0.05 0.05];
panel1_force_z_ylim = [-0.08 0.02];
panel1_torque_x_ylim = [-0.003 0.003];
panel1_torque_y_ylim = [-0.003 0.003];
panel1_torque_z_ylim = [-0.003 0.003];

panel1_force_axis_ylims = {panel1_force_x_ylim, panel1_force_y_ylim, panel1_force_z_ylim};
panel1_torque_axis_ylims = {panel1_torque_x_ylim, panel1_torque_y_ylim, panel1_torque_z_ylim};

f1 = figure('Name', 'Debug MOB Force / Torque Compare', 'NumberTitle', 'off', 'Color', 'w');
tiledlayout(f1, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    nexttile(2 * i - 1);
    plot(time, mob_force_none(:,i), 'LineWidth', 1.8, 'Color', meas_color); hold on;
    plot(time, mob_force_residual(:,i), '--', 'LineWidth', 1.8, 'Color', cmd_color);
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('%s [N]', axis_names{i}));
    title(sprintf('MOB force %s', axis_names{i}));
    legend({'pure momentum', 'consistency corrected'}, 'Location', 'best');
    if ~isempty(panel1_xlim)
        xlim(panel1_xlim);
    elseif ~isempty(summary_xlim)
        xlim(summary_xlim);
    end
    if ~isempty(panel1_force_axis_ylims{i})
        ylim(panel1_force_axis_ylims{i});
    elseif ~isempty(panel1_force_ylim)
        ylim(panel1_force_ylim);
    end

    nexttile(2 * i);
    plot(time, mob_torque(:,i), 'LineWidth', 1.8, 'Color', [0.2 0.6 0.2]);
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('%s [N*m]', axis_names{i}));
    title(sprintf('MOB torque %s', axis_names{i}));
    legend({'torque observer'}, 'Location', 'best');
    if ~isempty(panel1_xlim)
        xlim(panel1_xlim);
    elseif ~isempty(summary_xlim)
        xlim(summary_xlim);
    end
    if ~isempty(panel1_torque_axis_ylims{i})
        ylim(panel1_torque_axis_ylims{i});
    elseif ~isempty(panel1_torque_ylim)
        ylim(panel1_torque_ylim);
    end
end

%% 9) Figure 4: motor thrust / pwm
f4 = figure('Name', 'Debug Motor', 'NumberTitle', 'off', 'Color', 'w');
tiledlayout(f4, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on;
for i = 1:4
    plot(time, motor_thrust(:,i), 'LineWidth', 1.2, 'Color', motor_colors(i,:));
end
grid on;
xlabel('time [s]');
ylabel('thrust [N]');
title('Per-motor uncapped thrust');
legend(motor_names, 'Location', 'best');

nexttile;
plot(time, sum_thrust, 'k', 'LineWidth', 1.4); hold on;
plot(time, thrust_error_to_mg, 'Color', [0.2 0.6 0.2], 'LineWidth', 1.0);
grid on;
xlabel('time [s]');
ylabel('[N]');
title('Total thrust and error to m g');
legend({'sum thrust', 'sum thrust - m g'}, 'Location', 'best');

nexttile;
hold on;
for i = 1:4
    plot(time, motor_pwm(:,i), 'LineWidth', 1.2, 'Color', motor_colors(i,:));
end
grid on;
xlabel('time [s]');
ylabel('ratio [0..65535]');
title('Final actuator command');
legend(motor_names, 'Location', 'best');

nexttile;
yyaxis left;
plot(time, zero_bias_count, 'LineWidth', 1.2);
ylabel('count');
yyaxis right;
plot(time, batt_pm, 'LineWidth', 1.2, 'Color', [0.85 0.33 0.10]); hold on;
plot(time, batt_status, '--', 'LineWidth', 1.0, 'Color', [0.49 0.18 0.56]);
grid on;
xlabel('time [s]');
ylabel('voltage [V]');
title('zero\_bias\_count and battery voltage');
legend({'zero\_bias\_count', 'pm.vbat', 'status.battery\_voltage'}, 'Location', 'best');

%% 10) Figure 4.5: position / MOB force / force command compare
panel45_xlim = [30 300];
panel45_pos_ylim = panel05_pos_ylim;
panel45_pos_x_ylim = [-0.5 0.5];
panel45_pos_y_ylim = panel05_pos_y_ylim;
panel45_pos_z_ylim = panel05_pos_z_ylim;
panel45_force_ylim = [-0.01 0.1];
panel45_force_x_ylim = panel45_xlim;
panel45_force_y_ylim = panel1_force_y_ylim;
panel45_force_z_ylim = panel1_force_z_ylim;
panel45_force_cmd_ylim = [-0.01 0.1];
panel45_pos_axis_ylims = {panel45_pos_x_ylim, panel45_pos_y_ylim, panel45_pos_z_ylim};
panel45_force_axis_ylims = {panel45_force_x_ylim, panel45_force_y_ylim, panel45_force_z_ylim};

f45 = figure('Name', 'Debug Position and Force Control', 'NumberTitle', 'off', 'Color', 'w');
tiledlayout(f45, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    nexttile(2 * i - 1);
    plot(time, pose_xyz(:,i), 'LineWidth', 1.2, 'Color', meas_color); hold on;
    plot(time, fw_cmd_xyz(:,i), '--', 'LineWidth', 1.2, 'Color', cmd_color);
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('%s [m]', axis_names{i}));
    title(sprintf('Desired vs measured position %s', axis_names{i}));
    legend({'measured', 'fw final position'}, 'Location', 'best');
    if ~isempty(panel45_xlim)
        xlim(panel45_xlim);
    elseif ~isempty(summary_xlim)
        xlim(summary_xlim);
    end
    if ~isempty(panel45_pos_axis_ylims{i})
        ylim(panel45_pos_axis_ylims{i});
    elseif ~isempty(panel45_pos_ylim)
        ylim(panel45_pos_ylim);
    end
end

nexttile(2, [2 1]);
plot(time, mob_force_final(:,1), 'LineWidth', 1.2, 'Color', cmd_color); hold on;
plot(time, mob_force_final(:,2), '--', 'LineWidth', 1.2, 'Color', meas_color);
plot(time, mob_force_final(:,3), ':', 'LineWidth', 1.4, 'Color', [0.2 0.6 0.2]);
grid on;
xlabel('time [s]');
ylabel('[N]');
title('Momentum observer force used by control (final fHatW)');
legend({'fHatW_x', 'fHatW_y', 'fHatW_z'}, 'Location', 'best');
if ~isempty(panel45_xlim)
    xlim(panel45_xlim);
elseif ~isempty(summary_xlim)
    xlim(summary_xlim);
end
if ~isempty(panel45_force_ylim)
    ylim(panel45_force_ylim);
end

nexttile(6);
plot(time, force_desired_plot, '--', 'LineWidth', 1.2, 'Color', cmd_color); hold on;
plot(time, force_measured_x_for_control, 'LineWidth', 1.2, 'Color', meas_color);
grid on;
xlabel('time [s]');
ylabel('[N]');
title('Force cmd vs measured X for control');
legend({'force desired', 'measured normal force = -F hat x'}, 'Location', 'best');
if ~isempty(panel45_xlim)
    xlim(panel45_xlim);
elseif ~isempty(summary_xlim)
    xlim(summary_xlim);
end
if ~isempty(panel45_force_cmd_ylim)
    ylim(panel45_force_cmd_ylim);
end

%% 11) Figure 5: velocity compare
f5 = figure('Name', 'Debug Velocity', 'NumberTitle', 'off', 'Color', 'w');
tiledlayout(f5, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    nexttile;
    plot(time, state_vel(:,i), '-', 'LineWidth', 1.2, 'Color', cmd_color); hold on;
    plot(time, pos_vel(:,i), '--', 'LineWidth', 1.2, 'Color', meas_color);
    plot(time, vel_des(:,i), ':', 'LineWidth', 1.2, 'Color', [0.2 0.6 0.2]);
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('%s [m/s]', axis_names{i}));
    title(sprintf('Velocity %s: state vs su\\_vel\\_from\\_pos', axis_names{i}));
    legend({'state.velocity', 'su\_vel\_from\_pos', 'desired'}, 'Location', 'best');
end
nexttile;
plot(time, state_vel_norm, 'LineWidth', 1.2, 'Color', cmd_color); hold on;
plot(time, pos_vel_norm, '--', 'LineWidth', 1.2, 'Color', meas_color);
plot(time, vel_des_norm, ':', 'LineWidth', 1.2, 'Color', [0.2 0.6 0.2]);
plot(time, vel_diff_norm, ':k', 'LineWidth', 1.2);
grid on;
xlabel('time [s]');
ylabel('[m/s]');
title('Velocity norm compare');
legend({'|state.velocity|', '|su\_vel\_from\_pos|', '|desired|', '|difference|'}, 'Location', 'best');

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

function y = local_lowpass_zero_phase(x, sample_hz, cutoff_hz)
    y = x;
    if isempty(cutoff_hz) || ~isfinite(cutoff_hz) || cutoff_hz <= 0 || sample_hz <= 0
        return;
    end

    y_forward = local_lowpass_first_order(x, sample_hz, cutoff_hz);
    y_reverse = flipud(local_lowpass_first_order(flipud(y_forward), sample_hz, cutoff_hz));
    y = y_reverse;
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

    hTraj = plot(ax, ee_valid(:,1), ee_valid(:,2), ...
        'LineWidth', 3.0, 'Color', [0.55 0.55 0.55]);
    hTip = scatter(ax, ee_valid(:,1), ee_valid(:,2), 16, color_valid, ...
        'filled', 'MarkerFaceAlpha', 0.9, 'MarkerEdgeAlpha', 0.0, ...
        'PickableParts', 'all', 'ButtonDownFcn', @local_handle_ee_xy_click);
    scatter(ax, ee_valid(1,1), ee_valid(1,2), 36, [0.30 0.30 0.30], 'filled', 'MarkerEdgeColor', 'k');
    scatter(ax, ee_valid(end,1), ee_valid(end,2), 36, [0.15 0.15 0.15], 'filled', 'MarkerEdgeColor', 'k');

    cmap = turbo(256);
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
    color_valid = time_valid(:);
    color_min = min(color_valid, [], 'omitnan');
    color_max = max(color_valid, [], 'omitnan');
    if ~isfinite(color_min) || ~isfinite(color_max) || abs(color_max - color_min) < 1.0e-9
        color_min = 0.0;
        color_max = 1.0;
        color_valid = linspace(0.0, 1.0, size(ee_valid,1)).';
    end

    plot3(ax, ee_valid(:,1), ee_valid(:,2), ee_valid(:,3), ...
        'LineWidth', 3.0, 'Color', [0.55 0.55 0.55]);
    hTip = scatter3(ax, ee_valid(:,1), ee_valid(:,2), ee_valid(:,3), 16, color_valid, ...
        'filled', 'MarkerFaceAlpha', 0.9, 'MarkerEdgeAlpha', 0.0, ...
        'PickableParts', 'all', 'ButtonDownFcn', @local_handle_ee_xy_click);
    scatter3(ax, ee_valid(1,1), ee_valid(1,2), ee_valid(1,3), 36, [0.30 0.30 0.30], 'filled', 'MarkerEdgeColor', 'k');
    scatter3(ax, ee_valid(end,1), ee_valid(end,2), ee_valid(end,3), 36, [0.15 0.15 0.15], 'filled', 'MarkerEdgeColor', 'k');

    cmap = turbo(256);
    colormap(ax, cmap);
    cb = colorbar(ax);
    cb.Label.String = 'time [s]';
    caxis(ax, [color_min color_max]);
    ax.UserData.ee_xy_time = time_valid;
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
    color_valid = time_valid(:);
    color_min = min(color_valid, [], 'omitnan');
    color_max = max(color_valid, [], 'omitnan');
    if ~isfinite(color_min) || ~isfinite(color_max) || abs(color_max - color_min) < 1.0e-9
        color_min = 0.0;
        color_max = 1.0;
        color_valid = linspace(0.0, 1.0, size(ee_valid,1)).';
    end

    plot3(ax, ee_valid(:,1), ee_valid(:,2), ee_valid(:,3), ...
        'LineWidth', 3.0, 'Color', [0.55 0.55 0.55]);
    hTip = scatter3(ax, ee_valid(:,1), ee_valid(:,2), ee_valid(:,3), 16, color_valid, ...
        'filled', 'MarkerFaceAlpha', 0.9, 'MarkerEdgeAlpha', 0.0, ...
        'PickableParts', 'all', 'ButtonDownFcn', @local_handle_ee_xy_click);
    scatter3(ax, ee_valid(1,1), ee_valid(1,2), ee_valid(1,3), 36, [0.30 0.30 0.30], 'filled', 'MarkerEdgeColor', 'k');
    scatter3(ax, ee_valid(end,1), ee_valid(end,2), ee_valid(end,3), 36, [0.15 0.15 0.15], 'filled', 'MarkerEdgeColor', 'k');

    cmap = turbo(256);
    colormap(ax, cmap);
    cb = colorbar(ax);
    cb.Label.String = 'time [s]';
    caxis(ax, [color_min color_max]);
    ax.UserData.ee_xy_time = time_valid;
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
        cmap = turbo(256);
        cdata = linspace(0.0, 1.0, size(ee_valid,1));
        surface(ax, ...
            [ee_valid(:,1), ee_valid(:,1)], ...
            [ee_valid(:,3), ee_valid(:,3)], ...
            zeros(size(ee_valid,1), 2), ...
            [cdata(:), cdata(:)], ...
            'FaceColor', 'none', 'EdgeColor', 'interp', 'LineWidth', 2.0);
        scatter(ax, ee_valid(1,1), ee_valid(1,3), 42, cmap(1,:), 'filled', 'MarkerEdgeColor', 'k');
        scatter(ax, ee_valid(end,1), ee_valid(end,3), 42, cmap(end,:), 'filled', 'MarkerEdgeColor', 'k');
        colormap(ax, cmap);
        cb = colorbar(ax);
        cb.Label.String = 'time [s]';
        cb.Ticks = linspace(0, 1, 5);
        cb.TickLabels = compose('%.1f', linspace(time_valid(1), time_valid(end), 5));
        caxis(ax, [0 1]);
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

    xlim(ax, center(1) + [-half_range, half_range]);
    ylim(ax, center(2) + [-half_range, half_range]);
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
    cdata = ax.UserData.ee_xy_cdata;
    cmap = ax.UserData.ee_xy_cmap;

    diff_xy = ee_pos(:,1:2) - click_xy;
    [~, idx] = min(sum(diff_xy.^2, 2, 'omitnan'));
    if isempty(idx) || ~isfinite(idx)
        return;
    end

    delete(findobj(ax, 'Tag', 'ee_xy_selected_point'));
    delete(findobj(ax, 'Tag', 'ee_xy_selected_text'));

    color_idx = max(1, min(size(cmap, 1), 1 + round(cdata(idx) * (size(cmap,1) - 1))));
    selected_color = cmap(color_idx, :);
    scatter3(ax, ee_pos(idx,1), ee_pos(idx,2), ee_pos(idx,3), 80, selected_color, ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.2, 'Tag', 'ee_xy_selected_point');
    text(ax, ee_pos(idx,1), ee_pos(idx,2), ee_pos(idx,3), sprintf('  t=%.2fs', ee_time(idx)), ...
        'Color', [0.1 0.1 0.1], 'FontWeight', 'bold', 'VerticalAlignment', 'bottom', ...
        'Tag', 'ee_xy_selected_text');

    if isfield(ax.UserData, 'ee_xy_title')
        title(ax, sprintf('%s | selected t = %.2f s', ax.UserData.ee_xy_title, ee_time(idx)));
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

function f_yaw_body = local_rotate_world_to_yaw_body(f_world, yaw)
    f_yaw_body = nan(size(f_world));
    if size(f_world, 2) ~= 3
        return;
    end
    valid = isfinite(yaw) & all(isfinite(f_world), 2);
    if ~any(valid)
        return;
    end

    cy = cos(yaw(valid));
    sy = sin(yaw(valid));
    f_yaw_body(valid,1) = cy .* f_world(valid,1) + sy .* f_world(valid,2);
    f_yaw_body(valid,2) = -sy .* f_world(valid,1) + cy .* f_world(valid,2);
    f_yaw_body(valid,3) = f_world(valid,3);
end
