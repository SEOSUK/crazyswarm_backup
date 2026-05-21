%% data_modifying.m
% Subplot-based viewer for raw logging data inspection before firmware edits.
% Keeps the same CSV loading / variable extraction flow as data_decryptor.m,
% but intentionally avoids extra scaling, offset compensation, or offline models.

clear; close all; clc;

%% 0) Pick CSV
defaultDir = fullfile(getenv("HOME"), "hitl_ws", "src", "flying_pen", "bag", "logging");
if ~isfolder(defaultDir), defaultDir = pwd; end

[file, path] = uigetfile(fullfile(defaultDir, "*.csv"), "Select logging CSV");
if isequal(file,0)
    disp("Canceled."); return;
end
csv_path = fullfile(path, file);
fprintf("[INFO] Reading: %s\n", csv_path);

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
t0 = time(find(isfinite(time),1,'first'));
time = time - t0;

mask = [];
if any(strcmp(vars, "validity_bitmask"))
    mask = uint64(T{:, "validity_bitmask"});
end

%% 2) Helpers
get1 = @(name) local_get1(T, vars, name);

%% 3) Load signals
pose_xyz = [get1("pose_x"), get1("pose_y"), get1("pose_z")];
pose_rpy = [get1("pose_roll"), get1("pose_pitch"), get1("pose_yaw")];

batt_v_status = get1("status_battery_voltage");
batt_v_raw  = get1("raw_battery_voltage");
batt_v_filt = get1("filt_battery_voltage");
if ~any(isfinite(batt_v_raw)), batt_v_raw = batt_v_status; end
if ~any(isfinite(batt_v_filt)), batt_v_filt = batt_v_raw; end

cmd_xyzyaw = [get1("cmd_x"), get1("cmd_y"), get1("cmd_z"), get1("cmd_yaw")];

est_v = [get1("est_vx"), get1("est_vy"), get1("est_vz")];
est_a = [get1("est_ax"), get1("est_ay"), get1("est_az")];

gyro_meas = [get1("gyro_x"), get1("gyro_y"), get1("gyro_z")];
ang_acc = [get1("angAcc_x"), get1("angAcc_y"), get1("angAcc_z")];

vel_des = [get1("velDes_vx"), get1("velDes_vy"), get1("velDes_vz")];
att_des = [get1("attDes_roll"), get1("attDes_pitch"), get1("attDes_yaw")];
rate_des = [get1("rateDes_roll"), get1("rateDes_pitch"), get1("rateDes_yaw")];

motor_force = [get1("motor_f1"), get1("motor_f2"), get1("motor_f3"), get1("motor_f4")];
motor_force_scaled = [get1("motor_f1_scaled"), get1("motor_f2_scaled"), get1("motor_f3_scaled"), get1("motor_f4_scaled")];

body_input_force = [get1("bodyInFx"), get1("bodyInFy"), get1("bodyInFz")];
drone_world_force = [get1("droneWorldFx"), get1("droneWorldFy"), get1("droneWorldFz")];
drone_world_force_scaled = [get1("droneWorldFx_scaled"), get1("droneWorldFy_scaled"), get1("droneWorldFz_scaled")];
zero_bias_count = get1("zero_bias_count");

%% 4) Cleanup rows (valid time only)
validTime = isfinite(time);
time = time(validTime);
pose_xyz = pose_xyz(validTime,:);
pose_rpy = pose_rpy(validTime,:);
batt_v_status = batt_v_status(validTime);
batt_v_raw = batt_v_raw(validTime);
batt_v_filt = batt_v_filt(validTime);
cmd_xyzyaw = cmd_xyzyaw(validTime,:);
est_v = est_v(validTime,:);
est_a = est_a(validTime,:);
gyro_meas = gyro_meas(validTime,:);
ang_acc = ang_acc(validTime,:);
vel_des = vel_des(validTime,:);
att_des = att_des(validTime,:);
rate_des = rate_des(validTime,:);
motor_force = motor_force(validTime,:);
motor_force_scaled = motor_force_scaled(validTime,:);
body_input_force = body_input_force(validTime,:);
drone_world_force = drone_world_force(validTime,:);
drone_world_force_scaled = drone_world_force_scaled(validTime,:);
zero_bias_count = zero_bias_count(validTime);
if ~isempty(mask), mask = mask(validTime); end

N = numel(time);
fprintf("[INFO] Using %d rows.\n", N);

%% 5) Plot style
cmd_color = [0.0000 0.4470 0.7410];
meas_color = [0.8500 0.3250 0.0980];
axis_names = {'X', 'Y', 'Z'};
att_names = {'Roll', 'Pitch', 'Yaw'};

%% Position
xlim_raw = [0, max(time)];
ylim_raw = [NaN NaN; NaN NaN; NaN NaN];
xlim_pre = [0, max(time)];
ylim_pre = [NaN NaN; NaN NaN; NaN NaN];

% 전처리
cmd_position_pre = cmd_xyzyaw(:,1:3);
meas_position_pre = pose_xyz;

plot_cmd_meas_3x2( ...
    'Position', time, ...
    cmd_xyzyaw(:,1:3), pose_xyz, ...
    cmd_position_pre, meas_position_pre, ...
    axis_names, '[m]', ...
    xlim_raw, ylim_raw, xlim_pre, ylim_pre, ...
    cmd_color, meas_color);

%% Velocity
xlim_raw = [0, max(time)];
ylim_raw = [NaN NaN; NaN NaN; NaN NaN];
xlim_pre = [0, max(time)];
ylim_pre = [NaN NaN; NaN NaN; NaN NaN];

% 전처리
cmd_velocity_pre = vel_des;
meas_velocity_pre = est_v;

plot_cmd_meas_3x2( ...
    'Velocity', time, ...
    vel_des, est_v, ...
    cmd_velocity_pre, meas_velocity_pre, ...
    axis_names, '[m/s]', ...
    xlim_raw, ylim_raw, xlim_pre, ylim_pre, ...
    cmd_color, meas_color);

%% Acceleration
xlim_raw = [0, max(time)];
ylim_raw = [NaN NaN; NaN NaN; NaN NaN];
xlim_pre = [0, max(time)];
ylim_pre = [NaN NaN; NaN NaN; NaN NaN];

% 전처리
cmd_acceleration_pre = nan(size(est_a));
meas_acceleration_pre = est_a;

plot_cmd_meas_3x2( ...
    'Acceleration', time, ...
    nan(size(est_a)), est_a, ...
    cmd_acceleration_pre, meas_acceleration_pre, ...
    axis_names, '[logged unit]', ...
    xlim_raw, ylim_raw, xlim_pre, ylim_pre, ...
    cmd_color, meas_color);

%% Attitude
xlim_raw = [0, max(time)];
ylim_raw = [NaN NaN; NaN NaN; NaN NaN];
xlim_pre = [0, max(time)];
ylim_pre = [NaN NaN; NaN NaN; NaN NaN];

% 전처리
cmd_attitude_pre = att_des;
meas_attitude_pre = pose_rpy;

plot_cmd_meas_3x2( ...
    'Attitude', time, ...
    att_des, pose_rpy, ...
    cmd_attitude_pre, meas_attitude_pre, ...
    att_names, '[logged unit]', ...
    xlim_raw, ylim_raw, xlim_pre, ylim_pre, ...
    cmd_color, meas_color);

%% Angular Velocity
xlim_raw = [0, max(time)];
ylim_raw = [NaN NaN; NaN NaN; NaN NaN];
xlim_pre = [0, max(time)];
ylim_pre = [NaN NaN; NaN NaN; NaN NaN];

% 전처리
cmd_angvel_pre = rate_des;
meas_angvel_pre = gyro_meas;

plot_cmd_meas_3x2( ...
    'Angular Velocity', time, ...
    rate_des, gyro_meas, ...
    cmd_angvel_pre, meas_angvel_pre, ...
    att_names, '[logged unit]', ...
    xlim_raw, ylim_raw, xlim_pre, ylim_pre, ...
    cmd_color, meas_color);

%% Angular Acceleration
xlim_raw = [0, max(time)];
ylim_raw = [NaN NaN; NaN NaN; NaN NaN];
xlim_pre = [0, max(time)];
ylim_pre = [NaN NaN; NaN NaN; NaN NaN];

% 전처리
cmd_angacc_pre = nan(size(ang_acc));
meas_angacc_pre = ang_acc;

plot_cmd_meas_3x2( ...
    'Angular Acceleration', time, ...
    nan(size(ang_acc)), ang_acc, ...
    cmd_angacc_pre, meas_angacc_pre, ...
    att_names, '[logged unit]', ...
    xlim_raw, ylim_raw, xlim_pre, ylim_pre, ...
    cmd_color, meas_color);

%% PWM
xlim_raw = [0, max(time)];
ylim_raw = [NaN NaN; NaN NaN; NaN NaN];
xlim_pre = [0, max(time)];
ylim_pre = [NaN NaN; NaN NaN; NaN NaN];

% 전처리
cmd_pwm_pre = nan(N,3);
meas_pwm_pre = motor_force(:,1:3);

plot_cmd_meas_3x2( ...
    'PWM Proxy (motor_f1~f3)', time, ...
    nan(N,3), motor_force(:,1:3), ...
    cmd_pwm_pre, meas_pwm_pre, ...
    {'M1', 'M2', 'M3'}, '[logged unit]', ...
    xlim_raw, ylim_raw, xlim_pre, ylim_pre, ...
    cmd_color, meas_color);

%% Force
xlim_raw = [0, max(time)];
ylim_raw = [NaN NaN; NaN NaN; NaN NaN];
xlim_pre = [0, max(time)];
ylim_pre = [NaN NaN; NaN NaN; NaN NaN];

% 전처리
cmd_force_pre = body_input_force;
meas_force_pre = drone_world_force;

plot_cmd_meas_3x2( ...
    'Force (body input vs world force)', time, ...
    body_input_force, drone_world_force, ...
    cmd_force_pre, meas_force_pre, ...
    axis_names, '[N or logged unit]', ...
    xlim_raw, ylim_raw, xlim_pre, ylim_pre, ...
    cmd_color, meas_color, 'reference', 'measured');

%% Torque
 xlim_raw = [0, max(time)];
ylim_raw = [NaN NaN; NaN NaN; NaN NaN];
xlim_pre = [0, max(time)];
ylim_pre = [NaN NaN; NaN NaN; NaN NaN];

% 전처리
cmd_torque_pre = nan(N,3);
meas_torque_pre = nan(N,3);

plot_cmd_meas_3x2( ...
    'Torque', time, ...
    nan(N,3), nan(N,3), ...
    cmd_torque_pre, meas_torque_pre, ...
    axis_names, '[Nm or logged unit]', ...
    xlim_raw, ylim_raw, xlim_pre, ylim_pre, ...
    cmd_color, meas_color);

%% ===================== local helper functions =====================
function v = local_get1(T, vars, name)
idx = find(strcmp(vars, name), 1);
if isempty(idx)
    v = nan(height(T),1);
else
    v = T{:, idx};
end
end

function plot_cmd_meas_3x2(fig_name, time, raw_cmd, raw_meas, pre_cmd, pre_meas, labels, unit_text, xlim_raw, ylim_raw, xlim_pre, ylim_pre, cmd_color, meas_color, cmd_name, meas_name)
if nargin < 15 || isempty(cmd_name)
    cmd_name = 'command';
end
if nargin < 16 || isempty(meas_name)
    meas_name = 'measured';
end

f = figure('Name', fig_name, 'NumberTitle', 'off', 'Color', 'w');
tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_raw = gobjects(3,1);
ax_pre = gobjects(3,1);

for i = 1:3
    ax = nexttile(tl, 2*(i-1)+1);
    ax_raw(i) = ax;
    draw_one_axis(ax, time, raw_cmd(:,i), raw_meas(:,i), labels{i}, unit_text, ylim_raw, i, cmd_color, meas_color, cmd_name, meas_name);
    if i == 1
        title(ax, sprintf('%s - Raw', fig_name));
    end
    if i < 3, ax.XTickLabel = []; end

    ax = nexttile(tl, 2*(i-1)+2);
    ax_pre(i) = ax;
    draw_one_axis(ax, time, pre_cmd(:,i), pre_meas(:,i), labels{i}, unit_text, ylim_pre, i, cmd_color, meas_color, cmd_name, meas_name);
    if i == 1
        title(ax, sprintf('%s - Preprocessed', fig_name));
    end
    if i < 3, ax.XTickLabel = []; end
end

apply_panel_xlim(ax_raw, xlim_raw);
apply_panel_xlim(ax_pre, xlim_pre);
xlabel(tl, 'time [s]');
set([ax_raw; ax_pre], 'FontSize', 10, 'Color', 'w');
end

function draw_one_axis(ax, time, cmd_col, meas_col, label_name, unit_text, ylim_cfg, idx, cmd_color, meas_color, cmd_name, meas_name)
hold(ax, 'on');

has_cmd = any(isfinite(cmd_col));
has_meas = any(isfinite(meas_col));

h = gobjects(0);
legend_names = {};
if has_cmd
    h(end+1) = plot(ax, time, cmd_col, '--', 'LineWidth', 1.25, 'Color', cmd_color);
    legend_names{end+1} = cmd_name; %#ok<AGROW>
end
if has_meas
    h(end+1) = plot(ax, time, meas_col, '-', 'LineWidth', 1.10, 'Color', meas_color);
    legend_names{end+1} = meas_name; %#ok<AGROW>
end

if ~has_cmd && ~has_meas
    plot(ax, time, nan(size(time)), '-', 'LineWidth', 1.0, 'Color', meas_color);
    text(ax, 0.5, 0.5, 'No logged data', 'Units', 'normalized', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
end

grid(ax, 'on');
ylabel(ax, sprintf('%s %s', label_name, unit_text));
apply_user_ylim(ax, ylim_cfg, idx, [cmd_col; meas_col]);
if ~isempty(h)
    legend(ax, h, legend_names, 'Location', 'best');
end
end

function apply_user_ylim(ax, ylim_cfg, idx, auto_data)
if size(ylim_cfg,1) >= idx
    yl = ylim_cfg(idx,:);
else
    yl = [NaN NaN];
end

if all(isfinite(yl)) && yl(2) > yl(1)
    ylim(ax, yl);
    return;
end

auto_data = auto_data(isfinite(auto_data));
if isempty(auto_data)
    ylim(ax, [-1 1]);
    return;
end
lo = min(auto_data);
hi = max(auto_data);
if hi <= lo
    pad = max(1e-6, 0.1 * max(1, abs(lo)));
    ylim(ax, [lo-pad, hi+pad]);
else
    pad = 0.05 * (hi - lo);
    ylim(ax, [lo-pad, hi+pad]);
end
end

function apply_panel_xlim(axlist, xlim_value)
if numel(xlim_value) ~= 2 || ~all(isfinite(xlim_value)) || xlim_value(2) <= xlim_value(1)
    return;
end
set(axlist, 'XLim', xlim_value);
linkaxes(axlist, 'x');
end
