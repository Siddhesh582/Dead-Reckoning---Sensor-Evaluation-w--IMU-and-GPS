load('C:\Users\siddh\Desktop\RSN\Lab 4\data_driving.mat');

% IMU sample rate and time vector
imu_sample_rate = 40;  
num_imu_points = length(imu_data);  
imu_time_rel = (0:num_imu_points-1) / imu_sample_rate;

% GPS sample rate and time vector
gps_sample_rate = 1;  
num_gps_points = length(gps_data);  
gps_time_rel = (0:num_gps_points-1) / gps_sample_rate;

% Initialize arrays
magnetic_field_data = [];  % Magnetic field data
imu_yaw_angles = [];  % Yaw angles from IMU
yaw_rates = [];  % Gyro yaw rate data
imu_acc_x = [];  % IMU acceleration in X direction
imu_acc_y = [];  % IMU acceleration in Y direction
imu_acc_z = [];  % IMU acceleration in Z direction

% Extracting data from GPS
for i = 1:num_gps_points
    gps_lat(i) = gps_data{i}.gps.latitude;  % Extract latitude
    gps_lon(i) = gps_data{i}.gps.longitude;  % Extract longitude
    gps_relative_time(i) = gps_data{i}.gps.altitude;  % You can adjust this field if needed
end

% Extracting data 
for i = 1:num_imu_points
    % Magnetic field structure
    magnetic_field = imu_data{i}.mag_field;
    
    % Magnetic field values (convert to microgauss)
    magnetic_field_data(i, :) = [magnetic_field.x * 1e7, magnetic_field.y * 1e7, magnetic_field.z * 1e7]; 

    % Only extract Z-axis gyro data
    gyro_data = imu_data{i}.imu.angular_velocity; 
    yaw_rates(i) = gyro_data.z;  % Keep the Z-axis component

    % Quaternion from IMU data
    quaternion = imu_data{i}.imu.orientation; 
    imu_yaw_angles(i) = atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), ...
                               1 - 2 * (quaternion.y^2 + quaternion.z^2)); % Quaternion to yaw
end


% Magnetometer data calibration
corrected_magnetic_field_data = apply_calibration(magnetic_field_data);

raw_magnetometer_yaw = atan2(magnetic_field_data(:, 2), magnetic_field_data(:, 1)); % Raw magnetometer yaw
corrected_magnetometer_yaw = atan2(corrected_magnetic_field_data(:, 2), corrected_magnetic_field_data(:, 1)); % Corrected magnetometer yaw

% Integrate gyro yaw rate 
integrated_yaw = cumtrapz(imu_time_rel, yaw_rates); 

% Alpha value for filters
alpha = 0.93;

% Initialize arrays for filters
yaw_gyro_highpass = zeros(size(corrected_magnetometer_yaw));
yaw_mag_lowpass = zeros(size(corrected_magnetometer_yaw));
yaw_combined = zeros(size(corrected_magnetometer_yaw));

% First values for the filters
yaw_mag_lowpass(1) = corrected_magnetometer_yaw(1);
yaw_gyro_highpass(1) = integrated_yaw(1);  % Initialize with integrated yaw value

% Apply filters
for i = 2:length(corrected_magnetometer_yaw)
    dt = imu_time_rel(i) - imu_time_rel(i-1); 
    
    % Low-pass filter for magnetometer yaw
    yaw_mag_lowpass(i) = (1-alpha) * corrected_magnetometer_yaw(i);

    % High-pass filter for gyroscope yaw (integrated)
    yaw_gyro_highpass(i) = (alpha * (yaw_combined(i-1)) + (integrated_yaw(i)*dt));

    % Combine both with the complementary filter
    yaw_combined(i) = yaw_gyro_highpass(i) + yaw_mag_lowpass(i);
end

% Unwrap yaw angles
yaw_combined = unwrap(yaw_combined);
yaw_gyro_highpass = unwrap(yaw_gyro_highpass);  % Unwrap gyroscope high-pass yaw
yaw_mag_lowpass = unwrap(yaw_mag_lowpass);      % Unwrap magnetometer low-pass yaw

% Plotting results
% Raw vs Corrected Magnetic Data
figure;
hold on;
plot(imu_time_rel, raw_magnetometer_yaw, 'r', 'DisplayName', 'Yaw from Raw Magnetic Data');
plot(imu_time_rel, corrected_magnetometer_yaw, 'b--', 'DisplayName', 'Yaw from Corrected Magnetic Data');
title('Yaw Angles from Magnetic Data Over Time');
xlabel('Time (s)');
ylabel('Yaw (rad)');
grid on;
legend;
hold off;

% Corrected Magnetometer Yaw vs Yaw Integrated from Gyro
figure;
plot(imu_time_rel, corrected_magnetometer_yaw, 'r', 'DisplayName', 'Corrected Magnetometer Yaw');
hold on;
plot(imu_time_rel, integrated_yaw, 'Color', [0, 0.5, 0], 'DisplayName', 'Yaw Integrated from Gyro'); % Dark green
xlabel('Time (s)');
ylabel('Yaw (rad)');
title('Corrected Magnetometer Yaw vs Yaw Integrated from Gyro');
legend;
grid on;

% LPF, HPF, and CF Comparison
figure;
plot(imu_time_rel, yaw_combined, 'Color', [0, 0.5, 0], 'DisplayName', 'Complementary Filter Yaw'); % Dark green
hold on;
plot(imu_time_rel, yaw_mag_lowpass, 'r', 'DisplayName', 'Low-Pass Filtered Magnetometer Yaw'); % Corrected Magnetometer Yaw (LPF)
plot(imu_time_rel, yaw_gyro_highpass, 'b', 'DisplayName', 'High-Pass Filtered Gyro Yaw'); % Yaw Integrated from Gyro (HPF)
xlabel('Time (s)');
ylabel('Yaw (rad)');
title('Yaw Comparison: LPF, HPF, and Complementary Filter');
legend;
grid on;

% Yaw from Complementary Filter vs Yaw from IMU
figure;
plot(imu_time_rel, yaw_combined, 'Color', [0, 0.5, 0], 'DisplayName', 'Yaw from Complementary Filter'); % Dark green
hold on;
plot(imu_time_rel, imu_yaw_angles, 'k--', 'DisplayName', 'IMU Yaw'); % IMU Yaw
xlabel('Time (s)');
ylabel('Yaw (rad)');
title('Sensor Fusion Yaw vs IMU Yaw');
legend;
grid on;
hold off;

function corrected_magnetic_field_data = apply_calibration(magnetic_field_data)
    % Hard iron offset using (max + min) / 2 for both axes
    hard_iron_offset = (max(magnetic_field_data(:, 1:2)) + min(magnetic_field_data(:, 1:2))) / 2; 

    % Display the hard iron offset
    disp('Hard Iron Offset:');
    disp(hard_iron_offset);

    % Hard iron correction
    corrected_magnetic_field_data = magnetic_field_data; 
    corrected_magnetic_field_data(:, 1:2) = corrected_magnetic_field_data(:, 1:2) - hard_iron_offset; 

    % Soft iron correction using covariance matrix
    covariance_matrix = cov(corrected_magnetic_field_data(:, 1:2)); 
    [eig_vectors, eig_values] = eig(covariance_matrix); 

    % Inverse of the square root of the eigenvalues
    sqrt_eig_values = diag(sqrt(diag(eig_values)));
    soft_iron_matrix = eig_vectors * inv(sqrt_eig_values) * eig_vectors'; 

    % Apply soft iron correction
    corrected_magnetic_field_data(:, 1:2) = (soft_iron_matrix * corrected_magnetic_field_data(:, 1:2)')'; 

    % Display the soft iron matrix
    disp('Soft Iron Matrix:');
    disp(soft_iron_matrix);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% VELOCITY ESTIMATE  


imu_readings = length(imu_data);
gps_readings = length(gps_data);

% Extract timestamps from IMU data
timestamp_imu = zeros(imu_readings, 1); % Preallocate array for IMU timestamps
for i = 1:imu_readings
    timestamp_imu(i) = imu_data{i}.header.timestamp; % Accessing the timestamp for each IMU reading
end

% Extract timestamps from GPS data
timestamp_gps = zeros(gps_readings, 1); % Preallocate array for GPS timestamps
for i = 1:gps_readings
    timestamp_gps(i) = gps_data{i}.header.timestamp; % Accessing the timestamp for each GPS reading
end

imu_readings = length(imu_data);
gps_readings = length(gps_data);

forward_acc_x = zeros(imu_readings, 1);

utm_easting = zeros(gps_readings, 1);
utm_northing = zeros(gps_readings, 1);
zone = zeros(gps_readings, 1);
letter = zeros(gps_readings, 1);

for i = 1:imu_readings
    forward_acc_x(i) = imu_data{i}.imu.linear_acceleration.x;
end

for i = 1:gps_readings
    utm_easting(i) = gps_data{i}.gps.utm_easting;
    utm_northing(i) = gps_data{i}.gps.utm_northing;
    zone(i) = gps_data{i}.gps.zone;
    letter(i) = gps_data{i}.gps.letter;
end

gps_velocity = zeros(gps_readings, 1); % Array to store velocity estimates
distance = zeros(gps_readings, 1); % Array to store distance between consecutive GPS points

% Loop through each GPS reading to calculate distance and velocity
for i = 2:gps_readings-1
    % Calculate time difference between consecutive GPS timestamps
    delta_t = timestamp_gps(i) - timestamp_gps(i - 1); % Time in seconds

    % Calculate distance between consecutive GPS coordinates
    distance(i) = sqrt((utm_easting(i+1) - utm_easting(i))^2 + (utm_northing(i+1) - utm_northing(i))^2);

    % Calculate velocity only if distance is greater than zero and delta_t is positive
    if distance(i) > 0 && delta_t > 0
        gps_velocity(i) = distance(i) / delta_t; % Velocity in meters per second
    end
end

% Forward Velocity from Acceleration of IMU

imu_velocity = cumtrapz(timestamp_imu, forward_acc_x);

%Acceleration Bias 
threshold = 0.016;

gps_velocity_interp = interp1(timestamp_gps, gps_velocity, timestamp_imu, 'linear', 'extrap');
zero_velocity_index = find(abs(gps_velocity_interp) < threshold);
bias_acceleration = mean(forward_acc_x(zero_velocity_index));
threshold = 0.2;
forward_acc_x_corrected = forward_acc_x + bias_acceleration - 0.135 - mean(forward_acc_x) ;
forward_acc_x_corrected(abs(forward_acc_x_corrected) <= threshold) = 0;
imu_velocity_corrected = cumtrapz(timestamp_imu, forward_acc_x_corrected);
velocity_offset = mean(gps_velocity_interp) - mean(imu_velocity_corrected);
driving_forward_velocity_corrected_shifted = imu_velocity_corrected + velocity_offset;
imu_velocity_corrected = driving_forward_velocity_corrected_shifted;

% Time axis for plots
timestamp_imu = timestamp_imu - timestamp_imu(1); 
timestamp_gps = timestamp_gps - timestamp_gps(1); 


% Velocity estimate from the GPS with Velocity estimate from accelerometer before adjustments
figure;
hold on;
plot(timestamp_imu, imu_velocity, 'b', 'DisplayName', 'Velocity estimate from Accelerometer');
plot(timestamp_imu, gps_velocity_interp, 'r', 'DisplayName', 'Velocity estimate from GPS');
xlabel('Time (s)', 'FontSize', 16); % Specify a value for FontSize
ylabel('Velocity (m/s)', 'FontSize', 16); % Specify a value for FontSize
legend;
title('Velocity estimate from the GPS with Velocity estimate from accelerometer before adjustment');
grid on;
hold off;

% Velocity estimate from the GPS with Velocity estimate from accelerometer after adjustments
figure;
hold on;
plot(timestamp_imu, imu_velocity_corrected, 'b', 'DisplayName', 'Velocity estimate from Accelerometer');
plot(timestamp_imu, gps_velocity_interp, 'r', 'DisplayName', 'Velocity estimate from GPS');
xlabel('Time (s)', 'FontSize', 16); % Specify a value for FontSize
ylabel('Velocity (m/s)', 'FontSize', 16); % Specify a value for FontSize
legend;
title('Velocity estimate from the GPS with Velocity estimate from accelerometer after adjustment');
grid on;
hold off;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  DEAD RECKONING


imu_readings = length(imu_data);
gps_readings = length(gps_data);

acceleration_x = zeros(imu_readings, 1);
acceleration_y = zeros(imu_readings, 1);
angular_velocity_z = zeros(imu_readings, 1);

utm_easting = zeros(gps_readings, 1);
utm_northing = zeros(gps_readings, 1);

for i = 1:imu_readings
    acceleration_x(i) = imu_data{i}.imu.linear_acceleration.x;
    acceleration_y(i) = imu_data{i}.imu.linear_acceleration.y;
    angular_velocity_z(i) = imu_data{i}.imu.angular_velocity.z;
end

for i = 1:gps_readings
    utm_easting(i) = gps_data{i}.gps.utm_easting;
    utm_northing(i) = gps_data{i}.gps.utm_northing;
end

% Integrating forward velocity to obtain displacement
% Integrate velocity to get displacement
imu_displacement = cumtrapz(timestamp_imu, imu_velocity_corrected);

% GPS displacement
gps_displacement = zeros(gps_readings, 1);
for i = 2:length(gps_data)
    dx = utm_easting(i) - utm_easting(i-1);
    dy = utm_northing(i) - utm_northing(i-1);
    gps_displacement(i) = gps_displacement(i-1) + sqrt( dx^2 + dy^2 );
end

figure;
plot(timestamp_imu, imu_displacement, 'b', 'LineWidth', 1.5, 'DisplayName', 'IMU Displacement');
hold on;
plot(timestamp_gps, gps_displacement, 'r', 'LineWidth', 1.5, 'DisplayName', 'GPS Displacement');
xlabel('Time (seconds)', 'FontSize',16);
ylabel('Displacement (meters)', 'FontSize',16);
title('IMU vs GPS Displacement Over Time');
legend;
grid on;
hold off;

% finding obs
x_obs = forward_acc_x_corrected;
y_obs = acceleration_y;

% Integrate this acceleration to obtain vel
%imu_velocity_corrected = cumtrapz(timestamp_imu, driving_acc_x_corrected);

% Calculate omega * X_dot
omega_X_dot = angular_velocity_z .* imu_velocity_corrected;

figure;
plot(timestamp_imu, y_obs, 'b', 'LineWidth', 1, 'DisplayName', 'y_{obs}');
hold on;
plot(timestamp_imu, omega_X_dot, 'r', 'LineWidth', 1, 'DisplayName', 'omega * X dot');
xlabel('Time (seconds)', 'FontSize',16);
ylabel('Displacement (meters)', 'FontSize', 16);
legend;
title('Comparision of y_{obs} and omega*X_{dot}');
grid on;
hold off;

% Parameters
fs = 40;                % Sampling frequency (Hz)
fc = 1.2;                 % Cutoff frequency (Hz) 

% Butterworth filter design
[b, a] = butter(2, fc/(fs/2), 'low');  % 2nd-order Butterworth filter

% Apply the filter
y_obs_filtered = filtfilt(b, a, y_obs);  % Zero-phase filtering to avoid phase shift

% Plot the results
figure;
plot(timestamp_imu, y_obs_filtered, 'b', 'DisplayName', 'y_{obs} (Filtered)');
hold on;
plot(timestamp_imu, omega_X_dot, 'r', 'DisplayName', 'omega * X dot');
xlabel('Time (seconds)', 'FontSize', 16);
ylabel('Displacement (meters)', 'FontSize', 16);
legend;
title('Low-Pass Filter Applied to y_{obs}');
grid on;

% Define GPS trajectory
gps_x = utm_easting;
gps_y = utm_northing;

% Ensure theta has correct dimensions
theta = integrated_yaw(:);  % Reshape to column vector if needed

% Apply a correction factor to IMU velocity
velocity_correction_factor = 1.01;
ve = imu_velocity_corrected .* cos(theta) * velocity_correction_factor;
vn = imu_velocity_corrected .* sin(theta) * velocity_correction_factor;

% Integrate IMU velocities to obtain estimated IMU trajectory
xe = cumtrapz(timestamp_imu, ve);
xn = cumtrapz(timestamp_imu, vn);

% Combine IMU trajectory
imu_trajectory = [xe'; xn'];   % Transpose to make it 2 x N

% Flip the IMU trajectory along the northing axis (if needed)
scaling_factor = 0.65; % Keep the original scaling factor
xn_flipped = -imu_trajectory(2, :) * scaling_factor;  % Flip and scale northing
xe_flipped = imu_trajectory(1, :) * scaling_factor;   % Scale easting as well

% Additional counterclockwise rotation
additional_rotation_angle = pi/1.78;  % Adjust this value to rotate more or less (e.g., pi/4, pi/3)
R_additional = [cos(additional_rotation_angle), -sin(additional_rotation_angle); 
                sin(additional_rotation_angle), cos(additional_rotation_angle)];

% Apply additional rotation
rotated_trajectory = R_additional * [xe_flipped; xn_flipped];
xe_rotated = rotated_trajectory(1, :);
xn_rotated = rotated_trajectory(2, :);

% Calculate the translation to align rotated IMU trajectory with GPS starting point
offset_x = gps_x(1) - xe_rotated(1);
offset_y = gps_y(1) - xn_rotated(1);

% Apply translation to align IMU trajectory with GPS starting point
xe_aligned = xe_rotated + offset_x;
xn_aligned = xn_rotated + offset_y;

% Plot the GPS and aligned IMU trajectories for comparison
figure;
plot(gps_x, gps_y, 'b', 'LineWidth', 1.5); hold on;
plot(xe_aligned, xn_aligned, 'r', 'LineWidth', 1.5);

% Labels and settings
xlabel('Easting (m)');
ylabel('Northing (m)');
title('GPS and IMU Trajectory Comparison (Rotated and Aligned)');
legend('GPS Trajectory', 'Aligned IMU Trajectory');
grid on;
axis equal;
hold off;

% Debugging Information
disp(['Additional Rotation Angle: ', num2str(additional_rotation_angle), ' radians']);
disp(['Offset X: ', num2str(offset_x), ', Offset Y: ', num2str(offset_y)]);