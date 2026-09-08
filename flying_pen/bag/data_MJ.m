%% data_MJ.m
% Compact viewer for drone pose, end-effector position, attitude,
% per-propeller thrust, and battery voltage.
%
% Supported inputs:
% - data_logging_debug CSV: *_debug.csv
% - data_logging CSV: *.csv, when matching columns are available
%
clear; close all; clc;
set(groot, 'defaultFigureRenderer', 'painters');

%% 0) User config
sample_hz = 50.0;  % Used only when the CSV has no t_sec column.

% End-effector offset in the drone body frame [m].
% Edit these values to match su_params.yaml / your hardware.
ee_offset_body = [0.08, 0.0, 0.04];  % [x, y, z]

defaultDir = fullfile(getenv("HOME"), "hitl_ws", "src", "flying_pen", "bag", "logging");
if ~isfolder(defaultDir)
    defaultDir = pwd;
end

%% 1) Pick CSV and read
[file, path] = uigetfile(fullfile(defaultDir, "*.csv"), "Select logging CSV");
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

if isempty(T) || height(T) < 2
    error("CSV has too few rows.");
end

vars = string(T.Properties.VariableNames);
fprintf("[INFO] Rows: %d, Columns: %d\n", height(T), numel(vars));

if any(vars == "t_sec")
    time = local_get1(T, vars, "t_sec");
    first_valid = find(isfinite(time), 1, 'first');
    if ~isempty(first_valid)
        time = time - time(first_valid);
    end
else
    time = (0:height(T)-1).' ./ sample_hz;
end

%% 2) Load requested signals
pose_xyz = [
    local_get1(T, vars, "pose_x"), ...
    local_get1(T, vars, "pose_y"), ...
    local_get1(T, vars, "pose_z")];

pose_rpy = [
    local_get1(T, vars, "pose_roll"), ...
    local_get1(T, vars, "pose_pitch"), ...
    unwrap(local_get1(T, vars, "pose_yaw"))];

ee_xyz = local_compute_ee_position_world(pose_xyz, pose_rpy, ee_offset_body);

motor_thrust = [
    local_get1_fallback(T, vars, "f1", "motor_f1"), ...
    local_get1_fallback(T, vars, "f2", "motor_f2"), ...
    local_get1_fallback(T, vars, "f3", "motor_f3"), ...
    local_get1_fallback(T, vars, "f4", "motor_f4")];

battery_voltage = local_get1_fallback(T, vars, "pm_vbat", "status_battery_voltage");
status_battery_voltage = local_get1(T, vars, "status_battery_voltage");
pm_vbat = local_get1(T, vars, "pm_vbat");

wall_xyz = [
    local_get1(T, vars, "wall_x"), ...
    local_get1(T, vars, "wall_y"), ...
    local_get1(T, vars, "wall_z")];
wall_quat_xyzw = [
    local_get1(T, vars, "wall_qx"), ...
    local_get1(T, vars, "wall_qy"), ...
    local_get1(T, vars, "wall_qz"), ...
    local_get1(T, vars, "wall_qw")];
wall_rpy = local_quat_xyzw_to_rpy(wall_quat_xyzw);

valid_time = isfinite(time);
time = time(valid_time);
pose_xyz = pose_xyz(valid_time, :);
pose_rpy = pose_rpy(valid_time, :);
ee_xyz = ee_xyz(valid_time, :);
motor_thrust = motor_thrust(valid_time, :);
battery_voltage = battery_voltage(valid_time);
status_battery_voltage = status_battery_voltage(valid_time);
pm_vbat = pm_vbat(valid_time);
wall_xyz = wall_xyz(valid_time, :);
wall_quat_xyzw = wall_quat_xyzw(valid_time, :);
wall_rpy = wall_rpy(valid_time, :);

fprintf("[INFO] EE offset body [m] = [%.4f %.4f %.4f]\n", ee_offset_body);
local_print_availability("drone position", pose_xyz);
local_print_availability("end-effector position", ee_xyz);
local_print_availability("drone attitude", pose_rpy);
local_print_availability("per-propeller thrust", motor_thrust);
local_print_availability("battery voltage", battery_voltage);
local_print_availability("tilted wall position", wall_xyz);
local_print_availability("tilted wall attitude", wall_rpy);
local_print_availability("tilted wall quaternion", wall_quat_xyzw);

%% 3) Plot: drone position
axis_names = {'x', 'y', 'z'};
att_names = {'roll', 'pitch', 'yaw'};

figure('Name', 'Drone Position', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    nexttile;
    plot(time, pose_xyz(:, i), 'LineWidth', 1.4);
    grid on;
    ylabel(sprintf('%s [m]', axis_names{i}));
    title(sprintf('Drone position %s', axis_names{i}));
end
xlabel('time [s]');

%% 4) Plot: end-effector position
figure('Name', 'End-Effector Position', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    nexttile;
    plot(time, ee_xyz(:, i), 'LineWidth', 1.4);
    grid on;
    ylabel(sprintf('%s [m]', axis_names{i}));
    title(sprintf('End-effector position %s', axis_names{i}));
end
xlabel('time [s]');

figure('Name', 'Drone and End-Effector XYZ', 'Color', 'w');
plot3(pose_xyz(:,1), pose_xyz(:,2), pose_xyz(:,3), 'LineWidth', 1.2);
hold on;
plot3(ee_xyz(:,1), ee_xyz(:,2), ee_xyz(:,3), 'LineWidth', 1.2);
if any(isfinite(wall_xyz(:)))
    plot3(wall_xyz(:,1), wall_xyz(:,2), wall_xyz(:,3), 'LineWidth', 1.2);
end
grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Drone, end-effector, and tilted wall position');
if any(isfinite(wall_xyz(:)))
    legend({'drone', 'end-effector', 'tilted wall'}, 'Location', 'best');
else
    legend({'drone', 'end-effector'}, 'Location', 'best');
end

%% 5) Plot: drone attitude
figure('Name', 'Drone Attitude', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    nexttile;
    plot(time, pose_rpy(:, i), 'LineWidth', 1.4);
    grid on;
    ylabel('[rad]');
    title(sprintf('Drone attitude %s', att_names{i}));
end
xlabel('time [s]');

%% 6) Plot: per-propeller thrust
figure('Name', 'Per-Propeller Thrust', 'Color', 'w');
plot(time, motor_thrust, 'LineWidth', 1.3);
grid on;
xlabel('time [s]');
ylabel('thrust [N]');
title('Per-propeller thrust');
legend({'f1', 'f2', 'f3', 'f4'}, 'Location', 'best');

figure('Name', 'Total Thrust', 'Color', 'w');
plot(time, sum(motor_thrust, 2, 'omitnan'), 'k', 'LineWidth', 1.4);
grid on;
xlabel('time [s]');
ylabel('total thrust [N]');
title('Total thrust');

%% 7) Plot: battery voltage
figure('Name', 'Battery Voltage', 'Color', 'w');
hold on;
if any(isfinite(pm_vbat))
    plot(time, pm_vbat, 'LineWidth', 1.4);
end
if any(isfinite(status_battery_voltage))
    plot(time, status_battery_voltage, 'LineWidth', 1.2);
end
if ~any(isfinite(pm_vbat)) && ~any(isfinite(status_battery_voltage))
    plot(time, battery_voltage, 'LineWidth', 1.4);
end
grid on;
xlabel('time [s]');
ylabel('voltage [V]');
title('Battery voltage');
legend_entries = {};
if any(isfinite(pm_vbat)), legend_entries{end+1} = 'pm.vbat'; end
if any(isfinite(status_battery_voltage)), legend_entries{end+1} = 'status.battery\_voltage'; end
if isempty(legend_entries), legend_entries = {'battery voltage'}; end
legend(legend_entries, 'Location', 'best');

%% 8) Plot: tilted wall pose
if any(isfinite(wall_xyz(:)))
    if ~exist('axis_names', 'var'), axis_names = {'x', 'y', 'z'}; end
    if ~exist('att_names', 'var'), att_names = {'roll', 'pitch', 'yaw'}; end

    figure('Name', 'Tilted Wall Position', 'Color', 'w');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        nexttile;
        plot(time, wall_xyz(:, i), 'LineWidth', 1.4);
        grid on;
        ylabel(sprintf('%s [m]', axis_names{i}));
        title(sprintf('Tilted wall position %s', axis_names{i}));
    end
    xlabel('time [s]');

    figure('Name', 'Tilted Wall Attitude', 'Color', 'w');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        nexttile;
        plot(time, wall_rpy(:, i), 'LineWidth', 1.4);
        grid on;
        ylabel('[rad]');
        title(sprintf('Tilted wall attitude %s', att_names{i}));
    end
    xlabel('time [s]');

    figure('Name', 'Tilted Wall RPY Overlay', 'Color', 'w');
    plot(time, wall_rpy, 'LineWidth', 1.4);
    grid on;
    xlabel('time [s]');
    ylabel('attitude [rad]');
    title('Tilted wall roll, pitch, yaw');
    legend({'roll', 'pitch', 'yaw'}, 'Location', 'best');

    figure('Name', 'Tilted Wall Quaternion', 'Color', 'w');
    plot(time, wall_quat_xyzw, 'LineWidth', 1.3);
    grid on;
    xlabel('time [s]');
    ylabel('quaternion [-]');
    title('Tilted wall orientation quaternion');
    legend({'qx', 'qy', 'qz', 'qw'}, 'Location', 'best');

    figure('Name', 'End-Effector to Wall X Distance', 'Color', 'w');
    plot(time, ee_xyz(:, 1) - wall_xyz(:, 1), 'LineWidth', 1.4);
    grid on;
    xlabel('time [s]');
    ylabel('x distance [m]');
    title('End-effector x distance from tilted wall');
end

%% 9) Plot: drone and tilted wall roll/pitch
if any(isfinite(wall_rpy(:)))
    figure('Name', 'Drone vs Tilted Wall Roll Pitch', 'Color', 'w');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(time, pose_rpy(:, 1), 'LineWidth', 1.5);
    hold on;
    plot(time, wall_rpy(:, 1), 'LineWidth', 1.5);
    grid on;
    ylabel('roll [rad]');
    title('Drone roll and tilted wall roll');
    legend({'drone roll', 'tilted wall roll'}, 'Location', 'best');

    nexttile;
    plot(time, pose_rpy(:, 2), 'LineWidth', 1.5);
    hold on;
    plot(time, wall_rpy(:, 2), 'LineWidth', 1.5);
    grid on;
    xlabel('time [s]');
    ylabel('pitch [rad]');
    title('Drone pitch and tilted wall pitch');
    legend({'drone pitch', 'tilted wall pitch'}, 'Location', 'best');
end

%% Local functions
function x = local_get1(T, vars, name)
    if any(vars == name)
        x = T{:, char(name)};
    else
        x = nan(height(T), 1);
    end
end

function x = local_get1_fallback(T, vars, primary, fallback)
    x = local_get1(T, vars, primary);
    missing = ~isfinite(x);
    if any(missing)
        y = local_get1(T, vars, fallback);
        x(missing) = y(missing);
    end
end

function ee_pos = local_compute_ee_position_world(pose_xyz, pose_rpy, ee_offset_body)
    n = size(pose_xyz, 1);
    ee_pos = nan(n, 3);
    for k = 1:n
        if any(~isfinite(pose_xyz(k, :))) || any(~isfinite(pose_rpy(k, :)))
            continue;
        end
        R = local_rpy_to_rotmat(pose_rpy(k, :));
        ee_pos(k, :) = pose_xyz(k, :) + (R * ee_offset_body(:)).';
    end
end

function R = local_rpy_to_rotmat(rpy)
    roll = rpy(1);
    pitch = rpy(2);
    yaw = rpy(3);

    cr = cos(roll);  sr = sin(roll);
    cp = cos(pitch); sp = sin(pitch);
    cy = cos(yaw);   sy = sin(yaw);

    R = [
        cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr;
        sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr;
        -sp,   cp*sr,            cp*cr];
end

function rpy = local_quat_xyzw_to_rpy(q_xyzw)
    n = size(q_xyzw, 1);
    rpy = nan(n, 3);
    for k = 1:n
        q = q_xyzw(k, :);
        if any(~isfinite(q))
            continue;
        end
        qn = norm(q);
        if qn < 1.0e-9
            continue;
        end
        q = q ./ qn;
        x = q(1); y = q(2); z = q(3); w = q(4);

        sinr_cosp = 2.0 * (w*x + y*z);
        cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
        roll = atan2(sinr_cosp, cosr_cosp);

        sinp = 2.0 * (w*y - z*x);
        if abs(sinp) >= 1.0
            pitch = sign(sinp) * pi / 2.0;
        else
            pitch = asin(sinp);
        end

        siny_cosp = 2.0 * (w*z + x*y);
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
        yaw = atan2(siny_cosp, cosy_cosp);

        rpy(k, :) = [roll, pitch, yaw];
    end
    rpy(:, 3) = unwrap(rpy(:, 3));
end

function local_print_availability(label, x)
    ratio = nnz(isfinite(x)) / numel(x);
    fprintf("[INFO] %-24s finite %.1f %%\n", label, 100.0 * ratio);
end
