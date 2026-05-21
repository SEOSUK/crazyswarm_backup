%% data_decryptor.m
% Robust reader + flight-debug dashboard for logging CSV
%
% Matches: data_logging.cpp (kDataLen=52)
% Columns:
%   t_sec,
%   pose_x,y,z, pose_roll,pitch,yaw,
%   status_battery_voltage,
%   raw_battery_voltage, filt_battery_voltage,
%   cmd_x,cmd_y,cmd_z,cmd_yaw,
%   est_vx,vy,vz,
%   est_ax,ay,az,
%   gyro_x,y,z,
%   angAcc_x,y,z,
%   velDes_vx,vy,vz,
%   attDes_roll,pitch,yaw,
%   motor_f1..4,
%   motor_f1_scaled..4_scaled,
%   bodyInFx,bodyInFy,bodyInFz,
%   droneWorldFx,droneWorldFy,droneWorldFz,
%   droneWorldFx_scaled,droneWorldFy_scaled,droneWorldFz_scaled,
%   zero_bias_count,
%   rateDes_roll,pitch,yaw,
%   age_* (16개),
%   validity_bitmask
%
% Adds:
%  - Figure 1: R*F_B vs m*acceleration
%  - Figure 2: [left] rebuilt MOB F_ext vs R*F_B, [right] cmd pos vs measured
%  - Figure 4: raw vs logged-scaled vs offline-scaled force comparison
%
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

% 1) Read table robustly
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

% 2) Helpers
get1 = @(name) local_get1(T, vars, name);

% 3) Load signals (missing cols -> NaN)
% --- Pose ---
pose_xyz = [get1("pose_x"), get1("pose_y"), get1("pose_z")];
pose_rpy = [get1("pose_roll"), get1("pose_pitch"), get1("pose_yaw")];

% --- Status / voltage logs ---
batt_v_status = get1("status_battery_voltage");
batt_v_raw  = get1("raw_battery_voltage");
batt_v_filt = get1("filt_battery_voltage");

if ~any(isfinite(batt_v_raw))
    batt_v_raw = batt_v_status;
end
if ~any(isfinite(batt_v_filt))
    batt_v_filt = batt_v_raw;
end

% --- Command publisher position command ---
cmd_xyzyaw = [get1("cmd_x"), get1("cmd_y"), get1("cmd_z"), get1("cmd_yaw")];

% --- stateEstimate ---
est_v = [get1("est_vx"), get1("est_vy"), get1("est_vz")];
est_a = [get1("est_ax"), get1("est_ay"), get1("est_az")];

% --- gyro / angular acceleration ---
gyro_meas = [get1("gyro_x"), -get1("gyro_y"), get1("gyro_z")];
ang_acc = [get1("angAcc_x"), get1("angAcc_y"), get1("angAcc_z")];

% --- vel_des / att_des ---
vel_des = [get1("velDes_vx"), get1("velDes_vy"), get1("velDes_vz")];
att_des = [get1("attDes_roll"), get1("attDes_pitch"), get1("attDes_yaw")];
rate_des = [get1("rateDes_roll"), get1("rateDes_pitch"), get1("rateDes_yaw")];

% --- motor force logs ---
motor_force = [get1("motor_f1"), get1("motor_f2"), get1("motor_f3"), get1("motor_f4")];
motor_force_scaled = [get1("motor_f1_scaled"), get1("motor_f2_scaled"), get1("motor_f3_scaled"), get1("motor_f4_scaled")];

% --- input / world force logs ---
body_input_force = [get1("bodyInFx"), get1("bodyInFy"), get1("bodyInFz")];
drone_world_force = [get1("droneWorldFx"), get1("droneWorldFy"), get1("droneWorldFz")];
drone_world_force_scaled = [get1("droneWorldFx_scaled"), get1("droneWorldFy_scaled"), get1("droneWorldFz_scaled")];
zero_bias_count = get1("zero_bias_count");

% --- Legacy compatibility placeholders for old plots ---
sp_xyzyaw = cmd_xyzyaw;
mob_force = [nan(height(T),1), nan(height(T),1), nan(height(T),1)];
body_v = [nan(height(T),1), nan(height(T),1)];
qComp  = [nan(height(T),1), nan(height(T),1), nan(height(T),1), nan(height(T),1)];
Dstate = [nan(height(T),1), nan(height(T),1), nan(height(T),1)];

% --- ages (optional; 존재하면 로드) ---
age_pose             = get1("age_pose");
age_status           = get1("age_status");
age_cf_voltage       = get1("age_cf_voltage");
age_cf_body_input_force = get1("age_cf_body_input_force");
age_cf_world_force   = get1("age_cf_world_force");
age_cf_F_input_scaled = get1("age_cf_F_input_scaled");
age_cmd_position     = get1("age_cmd_position");
age_stateEst_vel     = local_get1_fallback(T, vars, "age_stateEstimate_velocity", "age_stateEst_vel");
age_stateEst_acc     = local_get1_fallback(T, vars, "age_stateEstimate_acc", "age_stateEst_acc");
age_gyro_feedback    = get1("age_gyro_feedback");
age_vel_des          = get1("age_vel_des");
age_att_des          = get1("age_att_des");
age_rate_des         = get1("age_rate_des");
age_cf_motor_force   = get1("age_cf_motor_force");
age_cf_motor_force_scaled = get1("age_cf_motor_force_scaled");
age_cf_zero_bias_dbg = get1("age_cf_zero_bias_dbg");
age_state_body_vel   = nan(height(T),1);
age_kalman_att_qComp = nan(height(T),1);
age_kalman_att_err   = nan(height(T),1);
age_cf_Fext_MOB      = nan(height(T),1);
age_setpoint         = nan(height(T),1);

% 4) Cleanup rows (valid time only)
validTime = isfinite(time);
time = time(validTime);

pose_xyz   = pose_xyz(validTime,:);
pose_rpy   = pose_rpy(validTime,:);
batt_v_status = batt_v_status(validTime);
batt_v_raw    = batt_v_raw(validTime);
batt_v_filt   = batt_v_filt(validTime);
body_input_force = body_input_force(validTime,:);
drone_world_force = drone_world_force(validTime,:);
drone_world_force_scaled = drone_world_force_scaled(validTime,:);
mob_force  = mob_force(validTime,:);
cmd_xyzyaw = cmd_xyzyaw(validTime,:);

sp_xyzyaw  = sp_xyzyaw(validTime,:);
est_v      = est_v(validTime,:);
est_a      = est_a(validTime,:);
gyro_meas  = gyro_meas(validTime,:);
ang_acc    = ang_acc(validTime,:);
motor_force = motor_force(validTime,:);
motor_force_scaled = motor_force_scaled(validTime,:);
zero_bias_count = zero_bias_count(validTime);
body_v     = body_v(validTime,:);
vel_des    = vel_des(validTime,:);
att_des    = att_des(validTime,:) * pi/180;
rate_des   = rate_des(validTime,:);
qComp      = qComp(validTime,:);
Dstate     = Dstate(validTime,:);

age_pose             = age_pose(validTime);
age_status           = age_status(validTime);
age_cf_voltage       = age_cf_voltage(validTime);
age_cf_body_input_force = age_cf_body_input_force(validTime);
age_cf_world_force   = age_cf_world_force(validTime);
age_cf_F_input_scaled = age_cf_F_input_scaled(validTime);
age_cf_Fext_MOB      = age_cf_Fext_MOB(validTime);
age_cmd_position     = age_cmd_position(validTime);
age_setpoint         = age_setpoint(validTime);
age_stateEst_vel     = age_stateEst_vel(validTime);
age_stateEst_acc     = age_stateEst_acc(validTime);
age_gyro_feedback    = age_gyro_feedback(validTime);
age_state_body_vel   = age_state_body_vel(validTime);
age_vel_des          = age_vel_des(validTime);
age_att_des          = age_att_des(validTime);
age_rate_des         = age_rate_des(validTime);
age_cf_motor_force   = age_cf_motor_force(validTime);
age_cf_motor_force_scaled = age_cf_motor_force_scaled(validTime);
age_cf_zero_bias_dbg = age_cf_zero_bias_dbg(validTime);
age_kalman_att_qComp = age_kalman_att_qComp(validTime);
age_kalman_att_err   = age_kalman_att_err(validTime);

if ~isempty(mask), mask = mask(validTime); end

N = numel(time);
fprintf("[INFO] Using %d rows.\n", N);

% 5) Quaternion -> RPY (Comp only)
rpy_comp = quat_to_rpy_batch(qComp);
rpy_comp(:,3) = unwrap(rpy_comp(:,3));

% 6) Default window selection
twin = [35, 200];
if time(end) > 120
    twin = [35, 200];
elseif time(end) > 80
    twin = [35, 200];
end
twin(1) = max(time(1), twin(1));
twin(2) = min(time(end), twin(2));
if ~(isfinite(twin(1)) && isfinite(twin(2)) && twin(2) > twin(1))
    tmin = min(time(isfinite(time)));
    tmax = max(time(isfinite(time)));
    if isempty(tmin) || isempty(tmax)
        twin = [0, 1];
    elseif tmax > tmin
        twin = [tmin, tmax];
    else
        twin = [tmin - 0.5, tmax + 0.5];
    end
end

%% 7) Force comparison figures only
set(groot,'defaultFigureRenderer','painters');
axis_names = {'X','Y','Z'};

% Acceleration preprocessing:
% logged acceleration is in G unit, so multiply by g to convert to m/s^2.
gravity_ms2 = 9.81;
est_a_proc = est_a * gravity_ms2;

% User-configurable mass [kg] for (mass * acceleration)
mass_kg = 0.0393;
mass_acc = est_a_proc * mass_kg;
motor_arm_length_m = 0.046;
thrust_to_torque = 0.006;
cmd_color = [0.0000 0.4470 0.7410];
meas_color = [0.8500 0.3250 0.0980];
cmd_lw = 1.25;
meas_lw = 1.15;

% Offline voltage-scaling model fit from logging
% Fit on [20, 120] s such that:
%   Fz_scaled = (a*v + b) * Fz_raw ~= m*g
fit_twin = [40, 200];
fit_twin(1) = max(time(1), fit_twin(1));
fit_twin(2) = min(time(end), fit_twin(2));

mg_hover = mass_kg * gravity_ms2;
Fz_raw = drone_world_force(:,3);
fit_mask = time >= fit_twin(1) & time <= fit_twin(2) & ...
           isfinite(batt_v_filt) & isfinite(Fz_raw) & ...
           Fz_raw > 1e-6;

voltage_model_a = nan;
voltage_model_b = nan;
offline_force_scale = nan(N,1);
drone_world_force_scaled_offline = nan(size(drone_world_force));
fit_rmse_fz = nan;

if nnz(fit_mask) >= 2
    gain_target = mg_hover ./ Fz_raw(fit_mask);
    X = [batt_v_filt(fit_mask), ones(nnz(fit_mask),1)];
    theta = X \ gain_target;

    voltage_model_a = theta(1);
    voltage_model_b = theta(2);

    offline_force_scale = voltage_model_a * batt_v_filt + voltage_model_b;
    offline_force_scale = max(offline_force_scale, 0.0);
    drone_world_force_scaled_offline = drone_world_force .* offline_force_scale;

    fit_rmse_fz = sqrt(mean((drone_world_force_scaled_offline(fit_mask,3) - mg_hover).^2, 'omitnan'));
end

% Default force model for analysis:
% prefer logged scaled force, then offline scaled force, and use raw only as fallback.
force_world_default = drone_world_force_scaled;
if ~any(all(isfinite(force_world_default), 2))
    force_world_default = drone_world_force_scaled_offline;
end
if ~any(all(isfinite(force_world_default), 2))
    force_world_default = drone_world_force;
end

force_world_default_cmp = force_world_default;
force_world_default_cmp(:,3) = force_world_default_cmp(:,3) - mass_kg * gravity_ms2;
mob_z_offset = 0.012;
force_world_default_cmp(:,3) = force_world_default_cmp(:,3) - mob_z_offset;

% MOB parameters (mirroring provided C++ observer translational logic)
mob_dt_fixed_enable = false;   % offline re-integration: use log dt by default
mob_dt_fixed_value = 0.004;    % fallback when fixed mode is enabled
mob_alpha = 1.0;
mob_kf = 3.0;
mob_deadzone_f = 0.0;

% User-configurable y-limits per subplot [min max]
% Row order: X, Y, Z. Use [NaN NaN] for auto.
ylim_cfg_fig1 = [NaN NaN; NaN NaN; NaN NaN];      % Figure 1: R*F_B vs m*a
ylim_cfg_fig2_left  = [-0.2 0.3; -0.25 0.25; 0.0 0.3]; % Figure 2 left: F_ext vs R*F_B
ylim_cfg_fig2_right = [0.5 1.5; -0.2 0.2; 0.7 1.3]; % Figure 2 right: cmd pos vs measured
ylim_cfg_fig3_left = [-0.5 0.5; -0.5 0.5; -0.2 0.2];  % Figure 3 left: attitude (roll/pitch/yaw)
ylim_cfg_fig3_right = [0.5 1.5; -0.2 0.2; 0.7 1.3]; % Figure 3 right: position (x/y/z)

% Build world-frame force and body torque directly from scaled propeller thrust.
body_force_in_world = nan(size(body_input_force));
body_force_from_motors = nan(N,3);
body_torque_from_motors = nan(N,3);
for i = 1:N
    r = pose_rpy(i,1);
    p = pose_rpy(i,2);
    y = pose_rpy(i,3);
    if all(isfinite(motor_force_scaled(i,:)))
        f1 = motor_force_scaled(i,1);
        f2 = motor_force_scaled(i,2);
        f3 = motor_force_scaled(i,3);
        f4 = motor_force_scaled(i,4);
        arm = motor_arm_length_m * 0.7071067811865476;
        body_force_from_motors(i,:) = [0.0, 0.0, f1 + f2 + f3 + f4];
        body_torque_from_motors(i,:) = [ ...
            -arm * f1 - arm * f2 + arm * f3 + arm * f4, ...
            -arm * f1 + arm * f2 + arm * f3 - arm * f4, ...
            -thrust_to_torque * f1 + thrust_to_torque * f2 - thrust_to_torque * f3 + thrust_to_torque * f4];
    end

    if ~all(isfinite([r p y])) || ~all(isfinite(body_force_from_motors(i,:)))
        continue;
    end
    Rwb = rpy_to_rotm_zyx(r, p, y);
    body_force_in_world(i,:) = (Rwb * body_force_from_motors(i,:).').';
end

% Rebuild translational momentum observer external force (world frame)
% using the same structure as provided C++ code:
% p = m v, r = p - p_hat, w_raw = kf*r, p_hat_dot = u - grav + w_raw,
% F_ext_hat = LPF(-w_raw, mob_alpha)
fext_hat_world = nan(N,3);
dt_last_valid = mob_dt_fixed_value;
p_hat = zeros(1,3);
fext_hat_prev = zeros(1,3);

for i = 1:N
    if mob_dt_fixed_enable
        dt = mob_dt_fixed_value;
    else
        if i == 1
            dt = dt_last_valid;
        else
            dt = time(i) - time(i-1);
            if isfinite(dt) && dt > 0
                dt_last_valid = dt;
            else
                dt = dt_last_valid;
            end
        end
    end
    if ~(isfinite(dt) && dt > 0)
        if i > 1, fext_hat_world(i,:) = fext_hat_world(i-1,:); end
        continue;
    end

    v = est_v(i,:);
    u = force_world_default(i,:);
    if ~all(isfinite(v)) || ~all(isfinite(u))
        if i > 1, fext_hat_world(i,:) = fext_hat_world(i-1,:); end
        continue;
    end

    p = mass_kg * v;
    grav = [0.0, 0.0, mass_kg * gravity_ms2];
    r = p - p_hat;

    wext_raw = mob_kf * r;
    for k = 1:3
        wext_raw(k) = apply_deadzone(wext_raw(k), mob_deadzone_f);
    end

    p_hat_dot = u - grav + wext_raw;
    p_hat = p_hat + dt * p_hat_dot;

    for k = 1:3
        fext_hat_prev(k) = lpf1(fext_hat_prev(k), -wext_raw(k), mob_alpha);
    end
    fext_hat_world(i,:) = fext_hat_prev;
end

mob_force_cmp = mob_force;
mob_force_cmp(:,3) = mob_force_cmp(:,3) - mob_z_offset;

fext_hat_world_cmp = fext_hat_world;
fext_hat_world_cmp(:,3) = fext_hat_world_cmp(:,3) - mob_z_offset;

% Plot-only preprocessing shared by Figure 3 and Figure 5
att_pose = [pose_rpy(:,1), -pose_rpy(:,2), unwrap(pose_rpy(:,3))];
att_des_u = [att_des(:,1), att_des(:,2), unwrap(att_des(:,3))];
att_names = {'Roll', 'Pitch', 'Yaw'};
pos_names = {'X', 'Y', 'Z'};
batt_v_scaled = batt_v_filt;

% Prefer logged rate_des when available; otherwise approximate from att_des.
ang_vel_cmd = rate_des;
if ~any(all(isfinite(ang_vel_cmd), 2))
    ang_vel_cmd = [ ...
        local_time_derivative(att_des(:,1), time), ...
        local_time_derivative(att_des(:,2), time), ...
        local_time_derivative(unwrap(att_des(:,3)), time)];
end

acc_cmd = [ ...
    local_time_derivative(vel_des(:,1), time), ...
    local_time_derivative(vel_des(:,2), time), ...
    local_time_derivative(vel_des(:,3), time)];
acc_cmd = local_lpf_columns(acc_cmd, 0.2);

ang_acc_num = [ ...
    local_time_derivative(gyro_meas(:,1), time), ...
    local_time_derivative(gyro_meas(:,2), time), ...
    local_time_derivative(gyro_meas(:,3), time)];
ang_acc_lpf_alpha = 0.2;
ang_acc_num_filt = local_lpf_columns(ang_acc_num, ang_acc_lpf_alpha);

world_force_plot = body_force_in_world;
ang_acc_meas = local_lpf_columns(ang_acc, 0.2);

vel_actual = vel_des;
vel_measured = est_v;
vel_actual_norm = sqrt(sum(vel_actual.^2, 2));
vel_measured_norm = sqrt(sum(vel_measured.^2, 2));
vel_actual_unit = zeros(size(vel_actual));
actual_norm_valid = isfinite(vel_actual_norm) & (vel_actual_norm > 1e-9);
vel_actual_unit(actual_norm_valid, :) = ...
    vel_actual(actual_norm_valid, :) ./ vel_actual_norm(actual_norm_valid);
vel_measured_unit = zeros(size(vel_measured));
measured_norm_valid = isfinite(vel_measured_norm) & (vel_measured_norm > 1e-9);
vel_measured_unit(measured_norm_valid, :) = ...
    vel_measured(measured_norm_valid, :) ./ vel_measured_norm(measured_norm_valid);

% End-effector-frame velocity from rigid-body forward kinematics.
% v_ee^body = R_wb' * v_drone^world + omega_body x r_ee^body
ee_offset_body_m = [0.10, 0.00, 0.04];
vel_ee_desired = nan(size(vel_actual));
vel_ee_measured = nan(size(vel_measured));
for i = 1:N
    r = pose_rpy(i,1);
    p = pose_rpy(i,2);
    y = pose_rpy(i,3);
    v_des_world = vel_actual(i,:);
    v_world = vel_measured(i,:);
    omega_body = gyro_meas(i,:);
    if ~all(isfinite([r p y])) || ~all(isfinite(v_des_world)) || ...
            ~all(isfinite(v_world)) || ~all(isfinite(omega_body))
        continue;
    end

    Rwb = rpy_to_rotm_zyx(r, p, y);
    vel_ee_desired(i,:) = (Rwb.' * v_des_world.').';
    v_body = (Rwb.' * v_world.').';
    vel_ee_measured(i,:) = v_body + cross(omega_body, ee_offset_body_m);
end

% End-effector velocity low-pass filter: cutoff = 1.0 rad/s
vel_ee_lpf_omega_rad_s = 6.28;
vel_ee_desired_filt = local_lpf_columns_timeaware(vel_ee_desired, time, vel_ee_lpf_omega_rad_s);
vel_ee_measured_filt = local_lpf_columns_timeaware(vel_ee_measured, time, vel_ee_lpf_omega_rad_s);
vel_ee_desired_norm = sqrt(sum(vel_ee_desired_filt.^2, 2));
vel_ee_measured_norm = sqrt(sum(vel_ee_measured_filt.^2, 2));
vel_ee_desired_unit = zeros(size(vel_ee_desired_filt));
ee_desired_norm_valid = isfinite(vel_ee_desired_norm) & (vel_ee_desired_norm > 1e-9);
vel_ee_desired_unit(ee_desired_norm_valid, :) = ...
    vel_ee_desired_filt(ee_desired_norm_valid, :) ./ vel_ee_desired_norm(ee_desired_norm_valid);
vel_ee_measured_unit = zeros(size(vel_ee_measured_filt));
ee_measured_norm_valid = isfinite(vel_ee_measured_norm) & (vel_ee_measured_norm > 1e-9);
vel_ee_measured_unit(ee_measured_norm_valid, :) = ...
    vel_ee_measured_filt(ee_measured_norm_valid, :) ./ vel_ee_measured_norm(ee_measured_norm_valid);

all_axes = gobjects(0);

%% Figure 4.35) Velocity desired / measured dashboard
twin = [80 105];
xlim_cfg_fig435_left  = twin;
xlim_cfg_fig435_right = twin;
xlim_cfg_fig435_norm  = twin;

% Row order: X, Y, Z
% Use [NaN NaN] for auto y-limits.
ylim_cfg_fig435_left  = [-0.3 0.3; -0.3 0.3; -0.3 0.3];
ylim_cfg_fig435_right = [NaN NaN; NaN NaN; NaN NaN];
ylim_cfg_fig435_norm  = [0 0.3];

f435 = figure('Name','Velocity desired / measured dashboard','NumberTitle','off', ...
              'Color','w','Units','normalized','Position',[0.06 0.08 0.88 0.78]);
left = 0.05; right = 0.03; top = 0.06; bottom = 0.08;
hgap = 0.035; vgap = 0.055;
panel_w = (1 - left - right - 2*hgap) / 3;
panel_h = (1 - top - bottom - vgap) / 2;
panel_y_row1 = 1 - top - panel_h;
panel_y_row2 = bottom;

p435_left = uipanel('Parent', f435, ...
    'Position', [left, panel_y_row1, panel_w, panel_h], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
tl435_left = tiledlayout(p435_left, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig435_left = gobjects(0);
for i = 1:3
    ax = nexttile(tl435_left, i); ax_fig435_left(end+1) = ax;
    h_des = plot(time, vel_actual(:,i), '--', 'LineWidth', cmd_lw, 'Color', cmd_color); hold on;
    h_meas = plot(time, vel_measured(:,i), '-', 'LineWidth', meas_lw, 'Color', meas_color);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig435_left, i, [vel_actual(:,i); vel_measured(:,i)]);
    ylabel(sprintf('%s [m/s]', axis_names{i}));
    if i == 1
        title('Drone velocity desired vs measured');
        legend([h_des h_meas], 'desired', 'measured', 'Location', 'best');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig435_left, xlim_cfg_fig435_left, 'data_decryptor_specific_fig435_left_xlink');
xlabel(tl435_left, 'time [s]');

p435_right = uipanel('Parent', f435, ...
    'Position', [left + panel_w + hgap, panel_y_row1, panel_w, panel_h], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
tl435_right = tiledlayout(p435_right, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig435_right = gobjects(0);
for i = 1:3
    ax = nexttile(tl435_right, i); ax_fig435_right(end+1) = ax;
    h_des = plot(time, vel_actual_unit(:,i), '--', 'LineWidth', cmd_lw, 'Color', cmd_color); hold on;
    h_meas = plot(time, vel_measured_unit(:,i), '-', 'LineWidth', meas_lw, 'Color', meas_color);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig435_right, i, [vel_actual_unit(:,i); vel_measured_unit(:,i)]);
    ylabel(sprintf('%s', axis_names{i}));
    if i == 1
        title('Drone velocity / |velocity|');
        legend([h_des h_meas], 'desired', 'measured', 'Location', 'best');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig435_right, xlim_cfg_fig435_right, 'data_decryptor_specific_fig435_right_xlink');
xlabel(tl435_right, 'time [s]');

p435_norm = uipanel('Parent', f435, ...
    'Position', [left + 2*(panel_w + hgap), panel_y_row1, panel_w, panel_h], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
tl435_norm = tiledlayout(p435_norm, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig435_norm = gobjects(0);
norm_series = [vel_actual_norm, vel_measured_norm];
for i = 1:3
    ax = nexttile(tl435_norm, i); ax_fig435_norm(end+1) = ax;
    h_des_norm = plot(time, vel_actual_norm, '--', 'LineWidth', cmd_lw, 'Color', cmd_color); hold on;
    h_meas_norm = plot(time, vel_measured_norm, '-', 'LineWidth', meas_lw, 'Color', meas_color);
    grid on;
    if all(isfinite(ylim_cfg_fig435_norm)) && numel(ylim_cfg_fig435_norm) == 2 && ...
            ylim_cfg_fig435_norm(1) < ylim_cfg_fig435_norm(2)
        ylim(ax, ylim_cfg_fig435_norm);
    else
        local_set_ylim(ax, norm_series);
    end
    xlim(ax, xlim_cfg_fig435_norm);
    ylabel('|v| [m/s]');
    if i == 1
        title('|velocity|');
        legend([h_des_norm h_meas_norm], 'desired', 'measured', 'Location', 'best');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig435_norm, xlim_cfg_fig435_norm, 'data_decryptor_specific_fig435_norm_xlink');
xlabel(tl435_norm, 'time [s]');

p435_left_ee = uipanel('Parent', f435, ...
    'Position', [left, panel_y_row2, panel_w, panel_h], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
tl435_left_ee = tiledlayout(p435_left_ee, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig435_left_ee = gobjects(0);
for i = 1:3
    ax = nexttile(tl435_left_ee, i); ax_fig435_left_ee(end+1) = ax;
    h_des = plot(time, vel_ee_desired_filt(:,i), '--', 'LineWidth', cmd_lw, 'Color', cmd_color); hold on;
    h_meas = plot(time, vel_ee_measured_filt(:,i), '-', 'LineWidth', meas_lw, 'Color', meas_color);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig435_left, i, [vel_ee_desired_filt(:,i); vel_ee_measured_filt(:,i)]);
    ylabel(sprintf('%s [m/s]', axis_names{i}));
    if i == 1
        title('End-effector velocity desired vs measured (LPF 1.0 rad/s)');
        legend([h_des h_meas], 'desired', 'measured', 'Location', 'best');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig435_left_ee, xlim_cfg_fig435_left, 'data_decryptor_specific_fig435_left_ee_xlink');
xlabel(tl435_left_ee, 'time [s]');

p435_right_ee = uipanel('Parent', f435, ...
    'Position', [left + panel_w + hgap, panel_y_row2, panel_w, panel_h], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
tl435_right_ee = tiledlayout(p435_right_ee, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig435_right_ee = gobjects(0);
for i = 1:3
    ax = nexttile(tl435_right_ee, i); ax_fig435_right_ee(end+1) = ax;
    h_des = plot(time, vel_ee_desired_unit(:,i), '--', 'LineWidth', cmd_lw, 'Color', cmd_color); hold on;
    h_meas = plot(time, vel_ee_measured_unit(:,i), '-', 'LineWidth', meas_lw, 'Color', meas_color);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig435_right, i, [vel_ee_desired_unit(:,i); vel_ee_measured_unit(:,i)]);
    ylabel(sprintf('%s', axis_names{i}));
    if i == 1
        title('End-effector velocity / |velocity| (LPF 1.0 rad/s)');
        legend([h_des h_meas], 'desired', 'measured', 'Location', 'best');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig435_right_ee, xlim_cfg_fig435_right, 'data_decryptor_specific_fig435_right_ee_xlink');
xlabel(tl435_right_ee, 'time [s]');

p435_norm_ee = uipanel('Parent', f435, ...
    'Position', [left + 2*(panel_w + hgap), panel_y_row2, panel_w, panel_h], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
tl435_norm_ee = tiledlayout(p435_norm_ee, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig435_norm_ee = gobjects(0);
norm_series_ee = [vel_ee_desired_norm, vel_ee_measured_norm];
for i = 1:3
    ax = nexttile(tl435_norm_ee, i); ax_fig435_norm_ee(end+1) = ax;
    h_des_norm = plot(time, vel_ee_desired_norm, '--', 'LineWidth', cmd_lw, 'Color', cmd_color); hold on;
    h_meas_norm = plot(time, vel_ee_measured_norm, '-', 'LineWidth', meas_lw, 'Color', meas_color);
    grid on;
    if all(isfinite(ylim_cfg_fig435_norm)) && numel(ylim_cfg_fig435_norm) == 2 && ...
            ylim_cfg_fig435_norm(1) < ylim_cfg_fig435_norm(2)
        ylim(ax, ylim_cfg_fig435_norm);
    else
        local_set_ylim(ax, norm_series_ee);
    end
    xlim(ax, xlim_cfg_fig435_norm);
    ylabel('|v| [m/s]');
    if i == 1
        title('End-effector |velocity| (LPF 1.0 rad/s)');
        legend([h_des_norm h_meas_norm], 'desired', 'measured', 'Location', 'best');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig435_norm_ee, xlim_cfg_fig435_norm, 'data_decryptor_specific_fig435_norm_ee_xlink');
xlabel(tl435_norm_ee, 'time [s]');

set(findall(f435, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

%% Figure 4.4) Command / measurement dashboard
% Panel-wise x/y limits for the original Figure 4.4 layout
twin = [10 125];
xlim_cfg_fig44_pos  = twin;
xlim_cfg_fig44_att  = twin;
xlim_cfg_fig44_vel  = twin;
xlim_cfg_fig44_gyro = twin;
xlim_cfg_fig44_mot  = twin;
xlim_cfg_fig44_batt = twin;

% Row order: X, Y, Z or Roll, Pitch, Yaw
% Use [NaN NaN] for auto y-limits.
ylim_cfg_fig44_pos  = [-0.5 1.5; -0.5 0.5; 0.5 1.5];
ylim_cfg_fig44_att  = [-0.3 0.3; -0.3 0.0; -0.3 0.3];
ylim_cfg_fig44_vel  = [NaN NaN; NaN NaN; NaN NaN];
ylim_cfg_fig44_gyro = [NaN NaN; NaN NaN; NaN NaN];

f44 = figure('Name','Command / Measurement dashboard','NumberTitle','off', ...
             'Color','w','Units','normalized','Position',[0.03 0.05 0.94 0.88]);
left = 0.035; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.025; vgap = 0.045;
ncol = 3; nrow = 2;
w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;
getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];
% (1,1) position command vs measured
p111 = uipanel('Parent', f44, 'Position', getPos(1,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl111 = tiledlayout(p111, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig44_pos = gobjects(0);
for i = 1:3
    ax = nexttile(tl111, i); ax_fig44_pos(end+1) = ax;
    h_meas = plot(time, pose_xyz(:,i), '-', 'LineWidth', meas_lw, 'Color', meas_color); hold on;
    h_cmd = plot(time, cmd_xyzyaw(:,i), '--', 'LineWidth', cmd_lw, 'Color', cmd_color);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig44_pos, i, [cmd_xyzyaw(:,i); pose_xyz(:,i)]);
    ylabel(sprintf('%s [m]', pos_names{i}));
    if i == 1
        title('Position command vs measured');
        legend([h_cmd h_meas], 'command', 'measured', 'Location', 'best');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig44_pos, xlim_cfg_fig44_pos, 'data_decryptor_specific_fig44_pos_xlink');
xlabel(tl111, 'time [s]');

% (2,1) attitude command vs measured
p211 = uipanel('Parent', f44, 'Position', getPos(2,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl211 = tiledlayout(p211, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig44_att = gobjects(0);
for i = 1:3
    ax = nexttile(tl211, i); ax_fig44_att(end+1) = ax;
    h_meas = plot(time, att_pose(:,i), '-', 'LineWidth', meas_lw, 'Color', meas_color); hold on;
    h_cmd = plot(time, att_des_u(:,i), '--', 'LineWidth', cmd_lw, 'Color', cmd_color);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig44_att, i, [att_des_u(:,i); att_pose(:,i)]);
    ylabel(sprintf('%s [rad]', att_names{i}));
    if i == 1
        title('Attitude command vs measured');
        legend([h_cmd h_meas], 'command', 'measured', 'Location', 'best');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig44_att, xlim_cfg_fig44_att, 'data_decryptor_specific_fig44_att_xlink');
xlabel(tl211, 'time [s]');

% (1,2) velocity command vs measured
p121 = uipanel('Parent', f44, 'Position', getPos(1,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl121 = tiledlayout(p121, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig44_vel = gobjects(0);
for i = 1:3
    ax = nexttile(tl121, i); ax_fig44_vel(end+1) = ax;
    h_meas = plot(time, est_v(:,i), '-', 'LineWidth', meas_lw, 'Color', meas_color); hold on;
    h_cmd = plot(time, vel_des(:,i), '--', 'LineWidth', cmd_lw, 'Color', cmd_color);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig44_vel, i, [vel_des(:,i); est_v(:,i)]);
    ylabel(sprintf('%s [m/s]', axis_names{i}));
    if i == 1
        title('Velocity command vs measured');
        legend([h_cmd h_meas], 'command', 'measured', 'Location', 'best');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig44_vel, xlim_cfg_fig44_vel, 'data_decryptor_specific_fig44_vel_xlink');
xlabel(tl121, 'time [s]');

% (2,2) angular velocity command vs measured
p221 = uipanel('Parent', f44, 'Position', getPos(2,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl221 = tiledlayout(p221, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig44_gyro = gobjects(0);
for i = 1:3
    ax = nexttile(tl221, i); ax_fig44_gyro(end+1) = ax;
    h_meas = plot(time, gyro_meas(:,i), '-', 'LineWidth', meas_lw, 'Color', meas_color); hold on;
    h_cmd = plot(time, ang_vel_cmd(:,i), '--', 'LineWidth', cmd_lw, 'Color', cmd_color);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig44_gyro, i, [ang_vel_cmd(:,i); gyro_meas(:,i)]);
    ylabel(sprintf('%s rate', att_names{i}));
    if i == 1
        title('Angular velocity command vs measured');
        if any(all(isfinite(rate_des), 2))
            legend([h_cmd h_meas], 'command', 'measured', 'Location', 'best');
        else
            legend([h_cmd h_meas], 'command', 'measured', 'Location', 'best');
        end
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig44_gyro, xlim_cfg_fig44_gyro, 'data_decryptor_specific_fig44_gyro_xlink');
xlabel(tl221, 'time [s]');

% (1,3) motor force raw / scaled in two rows
p131 = uipanel('Parent', f44, 'Position', getPos(1,3), 'BackgroundColor', 'w', 'BorderType', 'none');
tl131 = tiledlayout(p131, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig44_mot = gobjects(0);

ax = nexttile(tl131, 1); ax_fig44_mot(end+1) = ax;
plot(time, motor_force(:,1), '-', 'LineWidth', 0.70, 'Color', [0.0000 0.4470 0.7410]); hold on;
plot(time, motor_force(:,2), '-', 'LineWidth', 0.70, 'Color', [0.8500 0.3250 0.0980]);
plot(time, motor_force(:,3), '-', 'LineWidth', 0.70, 'Color', [0.9290 0.6940 0.1250]);
plot(time, motor_force(:,4), '-', 'LineWidth', 0.70, 'Color', [0.4940 0.1840 0.5560]);
grid on;
ylim(ax, [0 0.25]);
xlim(ax, xlim_cfg_fig44_mot);
ylabel(ax, 'Raw');
title(ax, 'Motor force');
legend(ax, 'f1', 'f2', 'f3', 'f4', 'Location', 'best');
ax.XTickLabel = [];

ax = nexttile(tl131, 2); ax_fig44_mot(end+1) = ax;
plot(time, motor_force_scaled(:,1), '-', 'LineWidth', 0.70, 'Color', [0.0000 0.4470 0.7410]); hold on;
plot(time, motor_force_scaled(:,2), '-', 'LineWidth', 0.70, 'Color', [0.8500 0.3250 0.0980]);
plot(time, motor_force_scaled(:,3), '-', 'LineWidth', 0.70, 'Color', [0.9290 0.6940 0.1250]);
plot(time, motor_force_scaled(:,4), '-', 'LineWidth', 0.70, 'Color', [0.4940 0.1840 0.5560]);
grid on;
ylim(ax, [0 0.25]);
xlim(ax, xlim_cfg_fig44_mot);
xlabel(ax, 'time [s]');
ylabel(ax, 'Scaled');
legend(ax, 'f1 scaled', 'f2 scaled', 'f3 scaled', 'f4 scaled', 'Location', 'best');

% (2,3) battery voltage raw vs scaled
ax_fig44_batt = axes('Parent', f44, 'Position', getPos(2,3), 'Color', 'w');
plot(time, batt_v_raw, '-', 'LineWidth', 1.15); hold on;
plot(time, batt_v_scaled, '--', 'LineWidth', 1.2, 'Color', [0.8500 0.3250 0.0980]);
grid on;
local_set_ylim(ax_fig44_batt, [batt_v_raw; batt_v_scaled]);
xlim(ax_fig44_batt, xlim_cfg_fig44_batt);
xlabel(ax_fig44_batt, 'time [s]');
ylabel(ax_fig44_batt, 'Voltage [V]');
title(ax_fig44_batt, 'Battery voltage raw vs scaled');
legend(ax_fig44_batt, 'raw', 'scaled', 'Location', 'best');

set(findall(f44, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

%% Figure 4.4+) Derived dynamics dashboard
xlim_cfg_fig44b_pos   = twin;
xlim_cfg_fig44b_att   = twin;
xlim_cfg_fig44b_acc   = twin;
xlim_cfg_fig44b_wf    = twin;
xlim_cfg_fig44b_anga  = twin;
xlim_cfg_fig44b_tau   = twin;

f44b = figure('Name','Derived dynamics dashboard','NumberTitle','off', ...
              'Color','w','Units','normalized','Position',[0.03 0.05 0.94 0.88]);
left = 0.035; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.025; vgap = 0.045;
ncol = 3; nrow = 2;
w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;
getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

p111b = uipanel('Parent', f44b, 'Position', getPos(1,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl111b = tiledlayout(p111b, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig44b_pos = gobjects(0);
for i = 1:3
    ax = nexttile(tl111b, i); ax_fig44b_pos(end+1) = ax;
    h_meas = plot(time, pose_xyz(:,i), '-', 'LineWidth', meas_lw, 'Color', meas_color); hold on;
    h_cmd = plot(time, cmd_xyzyaw(:,i), '--', 'LineWidth', cmd_lw, 'Color', cmd_color);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig44_pos, i, [cmd_xyzyaw(:,i); pose_xyz(:,i)]);
    ylabel(sprintf('%s [m]', pos_names{i}));
    if i == 1
        title('Position command vs measured');
        legend([h_cmd h_meas], 'command', 'measured', 'Location', 'best');
    end
    if i < 3, ax.XTickLabel = []; end
end
apply_panel_xlim(ax_fig44b_pos, xlim_cfg_fig44b_pos, 'data_decryptor_specific_fig44b_pos_xlink');
xlabel(tl111b, 'time [s]');

p211b = uipanel('Parent', f44b, 'Position', getPos(2,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl211b = tiledlayout(p211b, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig44b_att = gobjects(0);
for i = 1:3
    ax = nexttile(tl211b, i); ax_fig44b_att(end+1) = ax;
    h_meas = plot(time, att_pose(:,i), '-', 'LineWidth', meas_lw, 'Color', meas_color); hold on;
    h_cmd = plot(time, att_des_u(:,i), '--', 'LineWidth', cmd_lw, 'Color', cmd_color);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig44_att, i, [att_des_u(:,i); att_pose(:,i)]);
    ylabel(sprintf('%s [rad]', att_names{i}));
    if i == 1
        title('Attitude command vs measured');
        legend([h_cmd h_meas], 'command', 'measured', 'Location', 'best');
    end
    if i < 3, ax.XTickLabel = []; end
end
apply_panel_xlim(ax_fig44b_att, xlim_cfg_fig44b_att, 'data_decryptor_specific_fig44b_att_xlink');
xlabel(tl211b, 'time [s]');

p121b = uipanel('Parent', f44b, 'Position', getPos(1,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl121b = tiledlayout(p121b, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig44b_acc = gobjects(0);
for i = 1:3
    ax = nexttile(tl121b, i); ax_fig44b_acc(end+1) = ax;
    h_meas = plot(time, est_a_proc(:,i), '-', 'LineWidth', meas_lw, 'Color', meas_color); hold on;
    h_cmd = plot(time, acc_cmd(:,i), '--', 'LineWidth', cmd_lw, 'Color', cmd_color);
    grid on;
    local_set_ylim(ax, [acc_cmd(:,i); est_a_proc(:,i)]);
    ylabel(sprintf('%s [m/s^2]', axis_names{i}));
    if i == 1
        title('Acceleration command vs measured');
        legend([h_cmd h_meas], 'command', 'measured', 'Location', 'best');
    end
    if i < 3, ax.XTickLabel = []; end
end
apply_panel_xlim(ax_fig44b_acc, xlim_cfg_fig44b_acc, 'data_decryptor_specific_fig44b_acc_xlink');
xlabel(tl121b, 'time [s]');

p131b = uipanel('Parent', f44b, 'Position', getPos(1,3), 'BackgroundColor', 'w', 'BorderType', 'none');
tl131b = tiledlayout(p131b, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig44b_wf = gobjects(0);
for i = 1:3
    ax = nexttile(tl131b, i); ax_fig44b_wf(end+1) = ax;
    plot(time, world_force_plot(:,i), '-', 'LineWidth', meas_lw, 'Color', meas_color);
    grid on;
    local_set_ylim(ax, world_force_plot(:,i));
    ylabel(sprintf('%s [N]', axis_names{i}));
    if i == 1
        title('World force from propeller thrust allocation');
    end
    if i < 3, ax.XTickLabel = []; end
end
apply_panel_xlim(ax_fig44b_wf, xlim_cfg_fig44b_wf, 'data_decryptor_specific_fig44b_wf_xlink');
xlabel(tl131b, 'time [s]');

p221b = uipanel('Parent', f44b, 'Position', getPos(2,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl221b = tiledlayout(p221b, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig44b_anga = gobjects(0);
for i = 1:3
    ax = nexttile(tl221b, i); ax_fig44b_anga(end+1) = ax;
    h_meas = plot(time, ang_acc_meas(:,i), '-', 'LineWidth', meas_lw, 'Color', meas_color); hold on;
    h_cmd = plot(time, ang_acc_num_filt(:,i), '--', 'LineWidth', cmd_lw, 'Color', cmd_color);
    grid on;
    local_set_ylim(ax, [ang_acc_meas(:,i); ang_acc_num_filt(:,i)]);
    ylabel(sprintf('%s [rad/s^2]', att_names{i}));
    if i == 1
        title('Angular acceleration');
        legend([h_cmd h_meas], 'command/derived', 'measured', 'Location', 'best');
    end
    if i < 3, ax.XTickLabel = []; end
end
apply_panel_xlim(ax_fig44b_anga, xlim_cfg_fig44b_anga, 'data_decryptor_specific_fig44b_anga_xlink');
xlabel(tl221b, 'time [s]');

p231b = uipanel('Parent', f44b, 'Position', getPos(2,3), 'BackgroundColor', 'w', 'BorderType', 'none');
tl231b = tiledlayout(p231b, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig44b_tau = gobjects(0);
for i = 1:3
    ax = nexttile(tl231b, i); ax_fig44b_tau(end+1) = ax;
    plot(time, body_torque_from_motors(:,i), '-', 'LineWidth', meas_lw, 'Color', meas_color);
    grid on;
    local_set_ylim(ax, body_torque_from_motors(:,i));
    ylabel(sprintf('%s [Nm]', axis_names{i}));
    if i == 1
        title('Body torque');
    end
    if i < 3, ax.XTickLabel = []; end
end
apply_panel_xlim(ax_fig44b_tau, xlim_cfg_fig44b_tau, 'data_decryptor_specific_fig44b_tau_xlink');
xlabel(tl231b, 'time [s]');

set(findall(f44b, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

%% Figure 4.5) 2x2 dashboard with 3x1 subplots per panel (offline MOB, last panel)
% Figure 4.5 panel-wise x/y limits
% Each x-limit applies to all 3 subplots inside that panel only.
twin = [35 180];
xlim_cfg_fig45_pos = twin;
xlim_cfg_fig45_ma  = twin;
xlim_cfg_fig45_att = twin;
xlim_cfg_fig45_mob = twin;

% Row order: X, Y, Z
% Use [NaN NaN] for auto y-limits.
ylim_cfg_fig45_pos = [-0.2 1; -0.1 1.3; 0.8 1.4];
ylim_cfg_fig45_ma  = [-0.1 0.1; -0.1 0.1; -0.1 0.1];
ylim_cfg_fig45_att = [-0.3 0.3; -0.3 0.3; -0.3 0.3];
ylim_cfg_fig45_mob = [-0.1 0.1; -0.1 0.1; -0.1 0.1];

f45 = figure('Name','Tracking / Force dashboard (offline MOB)','NumberTitle','off', ...
             'Color','w','Units','normalized','Position',[0.04 0.05 0.92 0.88]);
left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;
ncol = 2; nrow = 2;
w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;
getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% (1,1) Position XYZ (des vs actual)
p11 = uipanel('Parent', f45, 'Position', getPos(1,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl11 = tiledlayout(p11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig45_pos = gobjects(0);
for i = 1:3
    ax = nexttile(tl11, i); ax_fig45_pos(end+1) = ax;
    plot(time, pose_xyz(:,i), '-', 'LineWidth', 1.15); hold on;
    plot(time, cmd_xyzyaw(:,i), '--', 'LineWidth', 0.95, 'Color', [0.8500 0.3250 0.0980]);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig45_pos, i, [cmd_xyzyaw(:,i); pose_xyz(:,i)]);
    ylabel(sprintf('%s [m]', axis_names{i}));
    if i == 1
        title('Position');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig45_pos, xlim_cfg_fig45_pos, 'data_decryptor_specific_fig45_pos_xlink');
xlabel(tl11, 'time [s]');

% (1,2) scaled force vs mass * acceleration
p12 = uipanel('Parent', f45, 'Position', getPos(1,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl12 = tiledlayout(p12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig45_ma = gobjects(0);
for i = 1:3
    ax = nexttile(tl12, i); ax_fig45_ma(end+1) = ax;
    plot(time, mass_acc(:,i), '-', 'LineWidth', 1.35, 'Color', [0.25 0.55 0.95]); hold on;
    plot(time, force_world_default_cmp(:,i), 'LineWidth', 1.05);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig45_ma, i, [force_world_default_cmp(:,i); mass_acc(:,i)]);
    ylabel(sprintf('%s [N]', axis_names{i}));
    if i == 1
        title('F_{thrust} vs. mass * acceleration');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig45_ma, xlim_cfg_fig45_ma, 'data_decryptor_specific_fig45_ma_xlink');
xlabel(tl12, 'time [s]');

% (2,1) attitude des vs actual
p21 = uipanel('Parent', f45, 'Position', getPos(2,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl21 = tiledlayout(p21, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig45_att = gobjects(0);
for i = 1:3
    ax = nexttile(tl21, i); ax_fig45_att(end+1) = ax;
    plot(time, att_pose(:,i), '-', 'LineWidth', 1.15); hold on;
    plot(time, att_des_u(:,i), '--', 'LineWidth', 0.95, 'Color', [0.8500 0.3250 0.0980]);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig45_att, i, [att_des_u(:,i); att_pose(:,i)]);
    ylabel(sprintf('%s [rad]', att_names{i}));
    if i == 1
        title('Attitude');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig45_att, xlim_cfg_fig45_att, 'data_decryptor_specific_fig45_att_xlink');
xlabel(tl21, 'time [s]');

% (2,2) scaled force vs offline MOB force
p22 = uipanel('Parent', f45, 'Position', getPos(2,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl22 = tiledlayout(p22, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig45_mob = gobjects(0);
for i = 1:3
    ax = nexttile(tl22, i); ax_fig45_mob(end+1) = ax;
    plot(time, fext_hat_world_cmp(:,i), '-', 'LineWidth', 2.4, 'Color', [0.25 0.55 0.95]); hold on;
    plot(time, force_world_default_cmp(:,i), 'LineWidth', 1.05);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig45_mob, i, [force_world_default_cmp(:,i); fext_hat_world_cmp(:,i)]);
    ylabel(sprintf('%s [N]', axis_names{i}));
    if i == 1
        title('F_{thrust} vs. Momentum Observer');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig45_mob, xlim_cfg_fig45_mob, 'data_decryptor_specific_fig45_mob_xlink');
xlabel(tl22, 'time [s]');

set(findall(f45, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

%% Figure 5) 2x2 dashboard with 3x1 subplots per panel
% Figure 5 panel-wise x/y limits
% Each x-limit applies to all 3 subplots inside that panel only.
twin = [35 180];
xlim_cfg_fig5_pos = twin;
xlim_cfg_fig5_ma  = twin;
xlim_cfg_fig5_att = twin;
xlim_cfg_fig5_mob = twin;

% Row order: X, Y, Z
% Use [NaN NaN] for auto y-limits.
ylim_cfg_fig5_pos = [-0.2 1; -0.1 1.3; 0.8 1.4];
ylim_cfg_fig5_ma  = [-0.1 0.1; -0.1 0.1; -0.1 0.1];
ylim_cfg_fig5_att = [-0.3 0.3; -0.3 0.3; -0.3 0.3];
ylim_cfg_fig5_mob = [-0.1 0.1; -0.1 0.1; -0.1 0.1];

f5 = figure('Name','Tracking / Force dashboard','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.04 0.05 0.92 0.88]);
left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;
ncol = 2; nrow = 2;
w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;
getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% (1,1) Position XYZ (des vs actual)
p11 = uipanel('Parent', f5, 'Position', getPos(1,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl11 = tiledlayout(p11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_pos = gobjects(0);
for i = 1:3
    ax = nexttile(tl11, i); ax_fig5_pos(end+1) = ax;
    plot(time, pose_xyz(:,i), '-', 'LineWidth', 1.15); hold on;
    plot(time, cmd_xyzyaw(:,i), '--', 'LineWidth', 0.95, 'Color', [0.8500 0.3250 0.0980]);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_pos, i, [cmd_xyzyaw(:,i); pose_xyz(:,i)]);
    ylabel(sprintf('%s [m]', axis_names{i}));
    if i == 1
        title('Position');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_pos, xlim_cfg_fig5_pos, 'data_decryptor_specific_fig5_pos_xlink');
xlabel(tl11, 'time [s]');

% (1,2) scaled force vs mass * acceleration
p12 = uipanel('Parent', f5, 'Position', getPos(1,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl12 = tiledlayout(p12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_ma = gobjects(0);
for i = 1:3
    ax = nexttile(tl12, i); ax_fig5_ma(end+1) = ax;
    plot(time, mass_acc(:,i), '-', 'LineWidth', 1.35, 'Color', [0.25 0.55 0.95]); hold on;
    plot(time, force_world_default_cmp(:,i), 'LineWidth', 1.05);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_ma, i, [force_world_default_cmp(:,i); mass_acc(:,i)]);
    ylabel(sprintf('%s [N]', axis_names{i}));
    if i == 1
        title('F_{thrust} vs. mass * acceleration');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_ma, xlim_cfg_fig5_ma, 'data_decryptor_specific_fig5_ma_xlink');
xlabel(tl12, 'time [s]');

% (2,1) attitude des vs actual
p21 = uipanel('Parent', f5, 'Position', getPos(2,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl21 = tiledlayout(p21, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_att = gobjects(0);
for i = 1:3
    ax = nexttile(tl21, i); ax_fig5_att(end+1) = ax;
    plot(time, att_pose(:,i), '-', 'LineWidth', 1.15); hold on;
    plot(time, att_des_u(:,i), '--', 'LineWidth', 0.95, 'Color', [0.8500 0.3250 0.0980]);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_att, i, [att_des_u(:,i); att_pose(:,i)]);
    ylabel(sprintf('%s [rad]', att_names{i}));
    if i == 1
        title('Attitude');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_att, xlim_cfg_fig5_att, 'data_decryptor_specific_fig5_att_xlink');
xlabel(tl21, 'time [s]');

% (2,2) scaled force vs MOB force
p22 = uipanel('Parent', f5, 'Position', getPos(2,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl22 = tiledlayout(p22, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_mob = gobjects(0);
for i = 1:3
    ax = nexttile(tl22, i); ax_fig5_mob(end+1) = ax;
    plot(time, mob_force_cmp(:,i), '-', 'LineWidth', 2.4, 'Color', [0.25 0.55 0.95]); hold on;
    plot(time, force_world_default_cmp(:,i), 'LineWidth', 1.05);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_mob, i, [force_world_default_cmp(:,i); mob_force_cmp(:,i)]);
    ylabel(sprintf('%s [N]', axis_names{i}));
    if i == 1
        title('F_{thrust} vs. Momentum Observer');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_mob, xlim_cfg_fig5_mob, 'data_decryptor_specific_fig5_mob_xlink');
xlabel(tl22, 'time [s]');

set(findall(f5, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');


%% Figure 5+5) 2x2 dashboard with 3x1 subplots per panel
% Figure 5 panel-wise x/y limits
% Each x-limit applies to all 3 subplots inside that panel only.
twin = [25 80];
xlim_cfg_fig5_pos = twin;
xlim_cfg_fig5_ma  = twin;
xlim_cfg_fig5_att = twin;
xlim_cfg_fig5_mob = twin;

% Row order: X, Y, Z
% Use [NaN NaN] for auto y-limits.
ylim_cfg_fig5_pos = [-0.2 1; -0.1 1.3; 0.8 1.4];
ylim_cfg_fig5_ma  = [-0.04 0.04; -0.04 0.04; -0.04 0.04];
ylim_cfg_fig5_att = [-0.2 0.2; -0.2 0.2; -0.2 0.2];
ylim_cfg_fig5_mob = [-0.05 0.05; -0.05 0.05; -0.05 0.05];

f5 = figure('Name','Tracking / Force dashboard','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.04 0.05 0.92 0.88]);
left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;
ncol = 2; nrow = 2;
w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;
getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% (1,1) Position XYZ (des vs actual)
p11 = uipanel('Parent', f5, 'Position', getPos(1,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl11 = tiledlayout(p11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_pos = gobjects(0);
for i = 1:3
    ax = nexttile(tl11, i); ax_fig5_pos(end+1) = ax;
    plot(time, pose_xyz(:,i), '-', 'LineWidth', 1.15); hold on;
    plot(time, cmd_xyzyaw(:,i), '--', 'LineWidth', 0.95, 'Color', [0.8500 0.3250 0.0980]);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_pos, i, [cmd_xyzyaw(:,i); pose_xyz(:,i)]);
    ylabel(sprintf('%s [m]', axis_names{i}));
    if i == 1
        title('Position');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_pos, xlim_cfg_fig5_pos, 'data_decryptor_specific_fig5_pos_xlink');
xlabel(tl11, 'time [s]');

% (1,2) scaled force vs mass * acceleration
p12 = uipanel('Parent', f5, 'Position', getPos(1,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl12 = tiledlayout(p12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_ma = gobjects(0);
for i = 1:3
    ax = nexttile(tl12, i); ax_fig5_ma(end+1) = ax;
    plot(time, mass_acc(:,i), '-', 'LineWidth', 1.35, 'Color', [0.25 0.55 0.95]); hold on;
    plot(time, force_world_default_cmp(:,i), 'LineWidth', 1.05);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_ma, i, [force_world_default_cmp(:,i); mass_acc(:,i)]);
    ylabel(sprintf('%s [N]', axis_names{i}));
    if i == 1
        title('F_{thrust} vs. mass * acceleration');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_ma, xlim_cfg_fig5_ma, 'data_decryptor_specific_fig5_ma_xlink');
xlabel(tl12, 'time [s]');

% (2,1) attitude des vs actual
p21 = uipanel('Parent', f5, 'Position', getPos(2,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl21 = tiledlayout(p21, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_att = gobjects(0);
for i = 1:3
    ax = nexttile(tl21, i); ax_fig5_att(end+1) = ax;
    plot(time, att_pose(:,i), '-', 'LineWidth', 1.15); hold on;
    plot(time, att_des_u(:,i), '--', 'LineWidth', 0.95, 'Color', [0.8500 0.3250 0.0980]);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_att, i, [att_des_u(:,i); att_pose(:,i)]);
    ylabel(sprintf('%s [rad]', att_names{i}));
    if i == 1
        title('Attitude');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_att, xlim_cfg_fig5_att, 'data_decryptor_specific_fig5_att_xlink');
xlabel(tl21, 'time [s]');

% (2,2) scaled force vs MOB force
p22 = uipanel('Parent', f5, 'Position', getPos(2,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl22 = tiledlayout(p22, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_mob = gobjects(0);
for i = 1:3
    ax = nexttile(tl22, i); ax_fig5_mob(end+1) = ax;
    plot(time, mob_force_cmp(:,i), '-', 'LineWidth', 2.4, 'Color', [0.25 0.55 0.95]); hold on;
    plot(time, force_world_default_cmp(:,i), 'LineWidth', 1.05);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_mob, i, [force_world_default_cmp(:,i); mob_force_cmp(:,i)]);
    ylabel(sprintf('%s [N]', axis_names{i}));
    if i == 1
        title('F_{thrust} vs. Momentum Observer');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_mob, xlim_cfg_fig5_mob, 'data_decryptor_specific_fig5_mob_xlink');
xlabel(tl22, 'time [s]');

set(findall(f5, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');


%% Figure 5+5+5) 2x2 dashboard with 3x1 subplots per panel
% Figure 5 panel-wise x/y limits
% Each x-limit applies to all 3 subplots inside that panel only.
twin = [80 170];
xlim_cfg_fig5_pos = twin;
xlim_cfg_fig5_ma  = twin;
xlim_cfg_fig5_att = twin;
xlim_cfg_fig5_mob = twin;

% Row order: X, Y, Z
% Use [NaN NaN] for auto y-limits.
ylim_cfg_fig5_pos = [-0.2 1; -0.1 1.3; 0.8 1.4];
ylim_cfg_fig5_ma  = [-0.1 0.1; -0.1 0.1; -0.1 0.1];
ylim_cfg_fig5_att = [-0.3 0.3; -0.3 0.3; -0.3 0.3];
ylim_cfg_fig5_mob = [-0.1 0.1; -0.1 0.1; -0.1 0.1];

f5 = figure('Name','Tracking / Force dashboard','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.04 0.05 0.92 0.88]);
left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;
ncol = 2; nrow = 2;
w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;
getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% (1,1) Position XYZ (des vs actual)
p11 = uipanel('Parent', f5, 'Position', getPos(1,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl11 = tiledlayout(p11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_pos = gobjects(0);
for i = 1:3
    ax = nexttile(tl11, i); ax_fig5_pos(end+1) = ax;
    plot(time, pose_xyz(:,i), '-', 'LineWidth', 1.15); hold on;
    plot(time, cmd_xyzyaw(:,i), '--', 'LineWidth', 0.95, 'Color', [0.8500 0.3250 0.0980]);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_pos, i, [cmd_xyzyaw(:,i); pose_xyz(:,i)]);
    ylabel(sprintf('%s [m]', axis_names{i}));
    if i == 1
        title('Position');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_pos, xlim_cfg_fig5_pos, 'data_decryptor_specific_fig5_pos_xlink');
xlabel(tl11, 'time [s]');

% (1,2) scaled force vs mass * acceleration
p12 = uipanel('Parent', f5, 'Position', getPos(1,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl12 = tiledlayout(p12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_ma = gobjects(0);
for i = 1:3
    ax = nexttile(tl12, i); ax_fig5_ma(end+1) = ax;
    plot(time, mass_acc(:,i), '-', 'LineWidth', 1.35, 'Color', [0.25 0.55 0.95]); hold on;
    plot(time, force_world_default_cmp(:,i), 'LineWidth', 1.05);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_ma, i, [force_world_default_cmp(:,i); mass_acc(:,i)]);
    ylabel(sprintf('%s [N]', axis_names{i}));
    if i == 1
        title('F_{thrust} vs. mass * acceleration');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_ma, xlim_cfg_fig5_ma, 'data_decryptor_specific_fig5_ma_xlink');
xlabel(tl12, 'time [s]');

% (2,1) attitude des vs actual
p21 = uipanel('Parent', f5, 'Position', getPos(2,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl21 = tiledlayout(p21, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_att = gobjects(0);
for i = 1:3
    ax = nexttile(tl21, i); ax_fig5_att(end+1) = ax;
    plot(time, att_pose(:,i), '-', 'LineWidth', 1.15); hold on;
    plot(time, att_des_u(:,i), '--', 'LineWidth', 0.95, 'Color', [0.8500 0.3250 0.0980]);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_att, i, [att_des_u(:,i); att_pose(:,i)]);
    ylabel(sprintf('%s [rad]', att_names{i}));
    if i == 1
        title('Attitude');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_att, xlim_cfg_fig5_att, 'data_decryptor_specific_fig5_att_xlink');
xlabel(tl21, 'time [s]');

% (2,2) scaled force vs MOB force
p22 = uipanel('Parent', f5, 'Position', getPos(2,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl22 = tiledlayout(p22, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_mob = gobjects(0);
for i = 1:3
    ax = nexttile(tl22, i); ax_fig5_mob(end+1) = ax;
    plot(time, mob_force_cmp(:,i), '-', 'LineWidth', 2.4, 'Color', [0.25 0.55 0.95]); hold on;
    plot(time, force_world_default_cmp(:,i), 'LineWidth', 1.05);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_mob, i, [force_world_default_cmp(:,i); mob_force_cmp(:,i)]);
    ylabel(sprintf('%s [N]', axis_names{i}));
    if i == 1
        title('F_{thrust} vs. Momentum Observer');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_mob, xlim_cfg_fig5_mob, 'data_decryptor_specific_fig5_mob_xlink');
xlabel(tl22, 'time [s]');

set(findall(f5, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');


%% Figure 1) scaled/world force vs m*acceleration
f1 = figure('Name','Scaled force vs m*acceleration','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.06 0.08 0.88 0.82]);
tl1 = tiledlayout(f1, 3, 1, 'TileSpacing','compact', 'Padding','compact');
for i = 1:3
    ax = nexttile(tl1,i); all_axes(end+1) = ax;
    plot(time, force_world_default_cmp(:,i), 'LineWidth', 1.3); hold on;
    plot(time, mass_acc(:,i), '--', 'LineWidth', 1.2);
    grid on; xlim(twin);
    apply_user_ylim(ax, ylim_cfg_fig1, i, [force_world_default_cmp(:,i); mass_acc(:,i)]);
    ylabel(sprintf('%s axis', axis_names{i}));
    title(sprintf('scaled force vs m*a (%s)', axis_names{i}));
    if i == 1
        legend('scaled force', sprintf('m*a (m=%.3f)', mass_kg), 'Location', 'best');
    end
end
xlabel(tl1, 'time [s]');


%% Figure 2) [Left] F_ext(MOB) vs scaled force, [Right] position cmd vs measured
f2 = figure('Name','MOB F_ext / position tracking','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.08 0.10 0.88 0.82]);
tl2 = tiledlayout(f2, 3, 2, 'TileSpacing','compact', 'Padding','compact');
for i = 1:3
    % Left column: F_ext(MOB) vs scaled force
    ax = nexttile(tl2,2*(i-1)+1); all_axes(end+1) = ax;
    plot(time, fext_hat_world_cmp(:,i), 'LineWidth', 1.3); hold on;
    plot(time, force_world_default_cmp(:,i), '--', 'LineWidth', 1.2);
    grid on; xlim(twin);
    apply_user_ylim(ax, ylim_cfg_fig2_left, i, [fext_hat_world_cmp(:,i); force_world_default_cmp(:,i)]);
    ylabel(sprintf('%s axis', axis_names{i}));
    title(sprintf('F_{ext} vs scaled force (%s)', axis_names{i}));
    if i == 1
        legend('F_{ext} (rebuilt MOB)', 'scaled force', 'Location', 'best');
    end

    % Right column: position command vs measured
    ax = nexttile(tl2,2*(i-1)+2); all_axes(end+1) = ax;
    plot(time, cmd_xyzyaw(:,i), 'LineWidth', 1.2); hold on;
    plot(time, pose_xyz(:,i), '--', 'LineWidth', 1.2);
    grid on; xlim(twin);
    apply_user_ylim(ax, ylim_cfg_fig2_right, i, [cmd_xyzyaw(:,i); pose_xyz(:,i)]);
    ylabel(sprintf('%s axis', axis_names{i}));
    title(sprintf('cmd pos vs measured (%s)', axis_names{i}));
    if i == 1
        legend('cmd pos', 'measured pos', 'Location', 'best');
    end
end
xlabel(tl2, 'time [s]');

%% Figure 3) [Left] attitude, [Right] position (summary)
f3 = figure('Name','Attitude / Position summary','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.10 0.12 0.88 0.55]);
tl3 = tiledlayout(f3, 3, 2, 'TileSpacing','compact', 'Padding','compact');

for i = 1:3
    % Left column: attitude (pose vs des)
    ax = nexttile(tl3, 2*(i-1)+1); all_axes(end+1) = ax;
    plot(time, att_pose(:,i), 'LineWidth', 1.2); hold on;
    plot(time, att_des_u(:,i), '--', 'LineWidth', 1.0);
    grid on; xlim(twin);
    apply_user_ylim(ax, ylim_cfg_fig3_left, i, [att_pose(:,i); att_des_u(:,i)]);
    ylabel('rad');
    title(sprintf('%s attitude', att_names{i}));
    if i == 1
        legend('pose', 'des', 'Location', 'best');
    end

    % Right column: position (measured vs cmd)
    ax = nexttile(tl3, 2*(i-1)+2); all_axes(end+1) = ax;
    plot(time, pose_xyz(:,i), 'LineWidth', 1.2); hold on;
    plot(time, cmd_xyzyaw(:,i), '--', 'LineWidth', 1.0);
    grid on; xlim(twin);
    apply_user_ylim(ax, ylim_cfg_fig3_right, i, [pose_xyz(:,i); cmd_xyzyaw(:,i)]);
    ylabel('m');
    title(sprintf('%s position', pos_names{i}));
    if i == 1
        legend('measured', 'cmd', 'Location', 'best');
    end
end
xlabel(tl3, 'time [s]');

%% Figure 4) Raw vs logged/offline scaled force comparison
f4 = figure('Name','Raw vs scaled force','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.12 0.10 0.84 0.78]);
tl4 = tiledlayout(f4, 3, 1, 'TileSpacing','compact', 'Padding','compact');

ax = nexttile(tl4,1); all_axes(end+1) = ax;
plot(time, drone_world_force(:,1), 'LineWidth', 1.1); hold on;
plot(time, drone_world_force_scaled(:,1), '--', 'LineWidth', 1.4);
plot(time, drone_world_force_scaled_offline(:,1), ':', 'LineWidth', 1.4);
grid on; xlim(twin);
local_set_ylim(ax, [drone_world_force(:,1); drone_world_force_scaled(:,1); drone_world_force_scaled_offline(:,1)]);
ylabel('N');
title('World force X: raw vs logged/offline scaled');
legend('raw world force', 'logged scaled world force', 'offline scaled world force', 'Location', 'best');

ax = nexttile(tl4,2); all_axes(end+1) = ax;
plot(time, drone_world_force(:,2), 'LineWidth', 1.1); hold on;
plot(time, drone_world_force_scaled(:,2), '--', 'LineWidth', 1.4);
plot(time, drone_world_force_scaled_offline(:,2), ':', 'LineWidth', 1.4);
grid on; xlim(twin);
local_set_ylim(ax, [drone_world_force(:,2); drone_world_force_scaled(:,2); drone_world_force_scaled_offline(:,2)]);
ylabel('N');
title('World force Y: raw vs logged/offline scaled');
legend('raw world force', 'logged scaled world force', 'offline scaled world force', 'Location', 'best');

ax = nexttile(tl4,3); all_axes(end+1) = ax;
plot(time, drone_world_force(:,3), 'LineWidth', 1.1); hold on;
plot(time, drone_world_force_scaled(:,3), '--', 'LineWidth', 1.4);
plot(time, drone_world_force_scaled_offline(:,3), ':', 'LineWidth', 1.4);
grid on; xlim(twin);
local_set_ylim(ax, [drone_world_force(:,3); drone_world_force_scaled(:,3); drone_world_force_scaled_offline(:,3)]);
ylabel('N');
title('World force Z: raw vs logged/offline scaled');
legend('raw world force', 'logged scaled world force', 'offline scaled world force', 'Location', 'best');
xlabel(tl4, 'time [s]');

sgtitle(tl4, sprintf('Offline fit on [%.1f, %.1f] s: a=%.6f, b=%.6f', ...
    fit_twin(1), fit_twin(2), voltage_model_a, voltage_model_b));

% One x-limit control for all subplots in all figures
apply_global_xlim(all_axes, twin);

%% 8) Quick prints
fprintf("\n[CHECK]\n");
fprintf("  qComp finite rows: %d\n", any(all(isfinite(qComp),2)));
fprintf("  raw batt finite: %d\n", any(isfinite(batt_v_raw)));
fprintf("  filt batt finite: %d\n", any(isfinite(batt_v_filt)));
fprintf("  acc preprocessing: est_a_proc = est_a * g (g=%.2f)\n", gravity_ms2);
fprintf("  mass for m*a plot: %.3f [kg]\n", mass_kg);
fprintf("  offline fit window: [%.3f, %.3f] s\n", fit_twin(1), fit_twin(2));
fprintf("  offline fit samples: %d\n", nnz(fit_mask));
fprintf("  offline force scale: a=%.6f, b=%.6f\n", voltage_model_a, voltage_model_b);
fprintf("  offline fit RMSE |Fz_scaled - m*g|: %.6f N\n", fit_rmse_fz);
fprintf("  finite rows in body->world raw force: %d\n", sum(all(isfinite(body_force_in_world),2)));
fprintf("  finite rows in default scaled force: %d\n", sum(all(isfinite(force_world_default),2)));
fprintf("  finite rows in raw world force: %d\n", sum(all(isfinite(drone_world_force),2)));
fprintf("  finite rows in scaled world force: %d\n", sum(all(isfinite(drone_world_force_scaled),2)));
fprintf("  finite rows in offline scaled world force: %d\n", sum(all(isfinite(drone_world_force_scaled_offline),2)));
fprintf("  finite rows in rebuilt F_ext: %d\n", sum(all(isfinite(fext_hat_world),2)));

if ~isempty(mask)
    % optional: show "overall freshness" quickly
    fprintf("  validity_bitmask example: first=%llu, last=%llu\n", ...
        uint64(mask(1)), uint64(mask(end)));
end

%% ===================== local helper functions =====================
function v = local_get1(T, vars, name)
    name = string(name);
    if any(strcmp(vars, name))
        v = T{:, name};
    else
        v = nan(height(T),1);
    end
end

function v = local_get1_fallback(T, vars, primary_name, fallback_name)
    primary_name = string(primary_name);
    fallback_name = string(fallback_name);
    if any(strcmp(vars, primary_name))
        v = T{:, primary_name};
    elseif any(strcmp(vars, fallback_name))
        v = T{:, fallback_name};
    else
        v = nan(height(T),1);
    end
end

function local_set_ylim(ax, data)
    data = data(isfinite(data));
    if isempty(data)
        return;
    end
    dmin = min(data);
    dmax = max(data);
    if abs(dmax - dmin) < 1e-9
        pad = max(1.0, 0.1 * max(abs(dmax), 1.0));
    else
        pad = 0.1 * (dmax - dmin);
    end
    ylim(ax, [dmin - pad, dmax + pad]);
end

function apply_user_ylim(ax, ylim_cfg, idx, auto_data)
% If ylim_cfg(idx,:) is valid finite [min max], use it.
% Otherwise, use auto scaling from auto_data.
    use_manual = false;
    if size(ylim_cfg,1) >= idx
        yl = ylim_cfg(idx,:);
        if numel(yl) == 2 && all(isfinite(yl)) && yl(1) < yl(2)
            use_manual = true;
        end
    end

    if use_manual
        ylim(ax, yl);
    else
        local_set_ylim(ax, auto_data);
    end
end

function apply_global_xlim(axlist, xlim_value)
% Force one global x-range and keep all axes synchronized afterward.
    axlist = axlist(isgraphics(axlist, 'axes'));
    if isempty(axlist)
        return;
    end
    set(axlist, 'XLim', xlim_value);
    linkaxes(axlist, 'x');
    hlink = linkprop(axlist, {'XLim'});
    setappdata(groot, 'data_decryptor_specific_xlink', hlink);
end

function apply_panel_xlim(axlist, xlim_value, appdata_key)
% Force one x-range inside one panel only and keep only that panel synchronized.
    axlist = axlist(isgraphics(axlist, 'axes'));
    if isempty(axlist)
        return;
    end
    set(axlist, 'XLim', xlim_value);
    linkaxes(axlist, 'x');
    hlink = linkprop(axlist, {'XLim'});
    setappdata(groot, appdata_key, hlink);
end

function dx = local_time_derivative(x, t)
    dx = nan(size(x));
    finite_mask = isfinite(x) & isfinite(t);
    if nnz(finite_mask) < 2
        return;
    end

    xw = x(finite_mask);
    tw = t(finite_mask);
    dxw = gradient(xw, tw);
    dx(finite_mask) = dxw;
end

function y = local_lpf_columns(x, alpha)
    y = nan(size(x));
    for col = 1:size(x, 2)
        prev = nan;
        for row = 1:size(x, 1)
            value = x(row, col);
            if ~isfinite(value)
                continue;
            end
            if ~isfinite(prev)
                prev = value;
            else
                prev = lpf1(prev, value, alpha);
            end
            y(row, col) = prev;
        end
    end
end

function y = local_lpf_columns_timeaware(x, t, omega_c)
    y = nan(size(x));
    for col = 1:size(x, 2)
        prev = nan;
        prev_t = nan;
        for row = 1:size(x, 1)
            value = x(row, col);
            t_now = t(row);
            if ~isfinite(value) || ~isfinite(t_now)
                continue;
            end
            if ~isfinite(prev) || ~isfinite(prev_t)
                prev = value;
                prev_t = t_now;
            else
                dt = t_now - prev_t;
                if ~(isfinite(dt) && dt > 0.0)
                    dt = 0.0;
                end
                alpha = (dt * omega_c) / (1.0 + dt * omega_c);
                prev = lpf1(prev, value, alpha);
                prev_t = t_now;
            end
            y(row, col) = prev;
        end
    end
end

function y = lpf1(y_prev, x, alpha)
    y = y_prev + alpha * (x - y_prev);
end

function out = apply_deadzone(value, threshold)
    if abs(value) <= threshold
        out = 0.0;
    elseif value > 0.0
        out = value - threshold;
    else
        out = value + threshold;
    end
end

function rpy = quat_to_rpy_batch(q)
% q: Nx4 [w x y z]
    N = size(q,1);
    rpy = nan(N,3);
    for i = 1:N
        q0=q(i,1); q1=q(i,2); q2=q(i,3); q3=q(i,4);
        if ~all(isfinite([q0 q1 q2 q3])), continue; end
        nrm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if nrm < 1e-9, continue; end
        q0=q0/nrm; q1=q1/nrm; q2=q2/nrm; q3=q3/nrm;

        sinr = 2*(q0*q1 + q2*q3);
        cosr = 1 - 2*(q1*q1 + q2*q2);
        roll = atan2(sinr, cosr);

        sinp = 2*(q0*q2 - q3*q1);
        if abs(sinp) >= 1
            pitch = sign(sinp)*pi/2;
        else
            pitch = asin(sinp);
        end

        siny = 2*(q0*q3 + q1*q2);
        cosy = 1 - 2*(q2*q2 + q3*q3);
        yaw = atan2(siny, cosy);

        rpy(i,:) = [roll pitch yaw];
    end
end

function R = rpy_to_rotm_zyx(roll, pitch, yaw)
% Rotation matrix from body frame to world frame with ZYX order:
% R = Rz(yaw) * Ry(pitch) * Rx(roll)
    cr = cos(roll);  sr = sin(roll);
    cp = cos(pitch); sp = sin(pitch);
    cy = cos(yaw);   sy = sin(yaw);

    R = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr;
         sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr;
         -sp,   cp*sr,            cp*cr];
end
