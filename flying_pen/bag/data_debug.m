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
%   mobTorque_x,y,z, mobResidual_x,y,z                 [N*m], observer torque / consistency residual
%   accRawBody_x,y,z                                   [G], body-frame accel after manual bias correction, before gravity-trim/LPF
%   gyroBody_x,y,z                                     [deg/s], body-frame gyro used by Mahony/complementary
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
mass_kg = 0.048;      % Crazyflie 2.1 Brushless mass
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
batt_status = get1("status_battery_voltage");
batt_pm = get1("pm_vbat");
zero_bias_count = get1("zero_bias_count");
mob_force_none = [get1("mobForceNone_x"), get1("mobForceNone_y"), get1("mobForceNone_z")];
mob_force_residual = [get1("mobForceResidual_x"), get1("mobForceResidual_y"), get1("mobForceResidual_z")];
mob_torque = [get1("mobTorque_x"), get1("mobTorque_y"), get1("mobTorque_z")];
mob_residual = [get1("mobResidual_x"), get1("mobResidual_y"), get1("mobResidual_z")];
acc_raw_body = [get1("accRawBody_x"), get1("accRawBody_y"), get1("accRawBody_z")];
gyro_body = [get1("gyroBody_x"), get1("gyroBody_y"), get1("gyroBody_z")];

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
batt_status = batt_status(valid);
batt_pm = batt_pm(valid);
zero_bias_count = zero_bias_count(valid);
mob_force_none = mob_force_none(valid,:);
mob_force_residual = mob_force_residual(valid,:);
mob_torque = mob_torque(valid,:);
mob_residual = mob_residual(valid,:);
acc_raw_body = acc_raw_body(valid,:);
gyro_body = gyro_body(valid,:);

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
mob_torque_norm = vecnorm(mob_torque, 2, 2);
mob_residual_norm = vecnorm(mob_residual, 2, 2);

acc_from_raw_rpy = nan(size(acc_raw_body));
acc_from_raw_rpy(:,1) = atan2(acc_raw_body(:,2), acc_raw_body(:,3));
acc_from_raw_rpy(:,2) = atan2(-acc_raw_body(:,1), sqrt(acc_raw_body(:,2).^2 + acc_raw_body(:,3).^2));
acc_from_raw_rpy(:,3) = nan(size(time));

gyro_integrated_rpy = local_integrate_body_rates_to_rpy_deg(gyro_body, time, pose_rpy(1,:));
gyro_integrated_rpy(:,3) = unwrap(gyro_integrated_rpy(:,3));

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

%% 5.5) Figure -1: accel/gyro inputs and offline attitude reconstruction
panelm1_xlim = [25 52];                % e.g. [0 10]
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
    plot(time, acc_raw_body(:,i), 'LineWidth', 1.2, 'Color', cmd_color);
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('%s [G]', axis_names{i}));
    title(sprintf('Raw accel %s', axis_names{i}));
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
    plot(time, gyro_body(:,i), 'LineWidth', 1.2, 'Color', [0.1 0.6 0.1]);
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('%s [deg/s]', axis_names{i}));
    title(sprintf('Raw gyro %s', axis_names{i}));
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
panel05_xlim = [20 65];               % e.g. [0 10]
panel05_pos_ylim = [];           % fallback for all position subplots
panel05_vel_ylim = [];           % fallback for all velocity subplots
panel05_att_ylim = [];           % fallback for all attitude subplots
panel05_pos_x_ylim = [-0.0 1.0];         % e.g. [-1 1]
panel05_pos_y_ylim = [-0.3 0.5];         % e.g. [-1 1]
panel05_pos_z_ylim = [0.4 1.2];         % e.g. [0 1]
panel05_vel_x_ylim = [-0.3 0.3];         % e.g. [-1 1]
panel05_vel_y_ylim = [-0.3 0.3];         % e.g. [-1 1]
panel05_vel_z_ylim = [-0.3 0.3];         % e.g. [-1 1]
panel05_att_x_ylim = [-0.2 0.2];         % e.g. [-0.5 0.5]
panel05_att_y_ylim = [-0.2 0.2];         % e.g. [-0.5 0.5]
panel05_att_z_ylim = [-2 2];         % e.g. [-3.14 3.14]

att_des_plot = att_des;
att_des_plot(:,3) = unwrap(att_des_plot(:,3));

panel05_pos_axis_ylims = {panel05_pos_x_ylim, panel05_pos_y_ylim, panel05_pos_z_ylim};
panel05_vel_axis_ylims = {panel05_vel_x_ylim, panel05_vel_y_ylim, panel05_vel_z_ylim};
panel05_att_axis_ylims = {panel05_att_x_ylim, panel05_att_y_ylim, panel05_att_z_ylim};

f05 = figure('Name', 'Debug Pose / Command Position / Attitude Compare', 'NumberTitle', 'off', 'Color', 'w');
tiledlayout(f05, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    nexttile(3 * (i - 1) + 1);
    plot(time, pose_xyz(:,i), 'LineWidth', 1.2, 'Color', meas_color); hold on;
    plot(time, fw_cmd_xyz(:,i), '--', 'LineWidth', 1.2, 'Color', cmd_color);
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
    plot(time, state_vel(:,i), 'LineWidth', 1.2, 'Color', meas_color); hold on;
    plot(time, vel_des(:,i), '--', 'LineWidth', 1.2, 'Color', cmd_color);
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
    plot(time, pose_rpy(:,i), 'LineWidth', 1.2, 'Color', meas_color); hold on;
    plot(time, att_des_plot(:,i), '--', 'LineWidth', 1.2, 'Color', cmd_color);
    grid on;
    xlabel('time [s]');
    ylabel(sprintf('%s [rad]', axis_names{i}));
    title(sprintf('Attitude tracking %s', axis_names{i}));
    legend({'measured', 'command'}, 'Location', 'best');
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
panel1_xlim = [10 200];                % e.g. [0 10]
panel1_force_ylim = [];          % fallback for all MOB force subplots
panel1_torque_ylim = [];         % fallback for all MOB torque subplots
panel1_force_x_ylim = [-0.03 0.03];
panel1_force_y_ylim = [-0.03 0.03];
panel1_force_z_ylim = [];
panel1_torque_x_ylim = [];
panel1_torque_y_ylim = [];
panel1_torque_z_ylim = [];

panel1_force_axis_ylims = {panel1_force_x_ylim, panel1_force_y_ylim, panel1_force_z_ylim};
panel1_torque_axis_ylims = {panel1_torque_x_ylim, panel1_torque_y_ylim, panel1_torque_z_ylim};

f1 = figure('Name', 'Debug MOB Force / Torque Compare', 'NumberTitle', 'off', 'Color', 'w');
tiledlayout(f1, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    nexttile(2 * i - 1);
    plot(time, mob_force_none(:,i), 'LineWidth', 1.2, 'Color', meas_color); hold on;
    plot(time, mob_force_residual(:,i), '--', 'LineWidth', 1.2, 'Color', cmd_color);
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
    plot(time, mob_torque(:,i), 'LineWidth', 1.2, 'Color', [0.2 0.6 0.2]);
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

%% 10) Figure 5: velocity compare
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
