function magnetometer_correction()
    % Load magnetic data from the .mat file
    data = load('/home/sid/LAB4/src/nav_driver/data/data_going_in_circles.mat');
    
    % Extract imu_data from the loaded structure
    imu_data = data.imu_data; 

    % Initialize an array to store magnetic data
    mag_data = zeros(length(imu_data), 3); 

    % Extract magnetic data from imu_data
    for i = 1:length(imu_data)
        mag_data(i, 1) = imu_data{i}.mag_field.x; 
        mag_data(i, 2) = imu_data{i}.mag_field.y; 
        mag_data(i, 3) = imu_data{i}.mag_field.z; 
    end

    % Plot raw magnetic data
    plot_raw_data(mag_data);

    % Calculate hard iron and soft iron values before correction
    hard_iron_offset = get_hard_iron_offset(mag_data);
    fprintf('Hard Iron Offset (Before Correction): X = %.4f, Y = %.4f\n', hard_iron_offset(1), hard_iron_offset(2));

    soft_iron_matrix = get_soft_iron_matrix(mag_data);
    fprintf('Soft Iron Matrix (Before Correction):\n');
    disp(soft_iron_matrix);

    % Perform hard iron corrections
    hard_corrected_data = correct_hard_iron_data(mag_data);

    % Plot hard iron corrected data
    plot_corrected_data(hard_corrected_data);
    
    % Perform and plot soft iron corrected data
    soft_corrected_data = plot_soft_iron_data(hard_corrected_data);
    
    % Calculate yaw angles
    yaw_raw = atan2(mag_data(:, 2), mag_data(:, 1)); % Raw yaw
    yaw_corrected = atan2(soft_corrected_data(:, 2), soft_corrected_data(:, 1)); % Corrected yaw

    % Time array for plotting (assuming uniform sampling)
    time = (0:length(mag_data)-1)'; % Replace with actual time if available

    % Plot yaw angles
    plot_yaw_data(time, yaw_raw, yaw_corrected);
end

function plot_raw_data(mag_data)
    figure; 
    scatter(mag_data(:, 1), mag_data(:, 2), 'r', 'filled', 'DisplayName', 'Raw Magnetic Field');
    ax = gca;
    ax.XAxisLocation = 'origin';
    ax.YAxisLocation = 'origin';
    grid on;
    title('Raw Magnetic Field Data');
    xlabel('Magnetometer X (T)');
    ylabel('Magnetometer Y (T)');
    legend;
end

function corrected_data = correct_hard_iron_data(mag_data)
    % Calculate hard iron offset using (max + min) / 2 for both axes
    hard_iron_offset_x = (max(mag_data(:, 1)) + min(mag_data(:, 1))) / 2;
    hard_iron_offset_y = (max(mag_data(:, 2)) + min(mag_data(:, 2))) / 2;
    hard_iron_offset = [hard_iron_offset_x, hard_iron_offset_y]; 

    % Hard iron correction
    corrected_data = mag_data; 
    corrected_data(:, 1:2) = corrected_data(:, 1:2) - hard_iron_offset; 
end

function offset = get_hard_iron_offset(mag_data)
    % Calculate hard iron offset using (max + min) / 2 for both axes
    hard_iron_offset_x = (max(mag_data(:, 1)) + min(mag_data(:, 1))) / 2;
    hard_iron_offset_y = (max(mag_data(:, 2)) + min(mag_data(:, 2))) / 2;
    offset = [hard_iron_offset_x, hard_iron_offset_y]; 
end

function soft_iron_matrix = get_soft_iron_matrix(mag_data)
    % Calculate the covariance matrix for the soft iron correction
    covariance_matrix = cov(mag_data(:, 1:2)); 
    [eig_vec, eig_val] = eig(covariance_matrix); 

    % Normalizing eigenvalues to get the soft iron correction matrix
    soft_iron_matrix = eig_vec * inv(sqrt(diag(diag(eig_val)))) * eig_vec'; 
end

function plot_corrected_data(corrected_data)
    figure; 
    scatter(corrected_data(:, 1), corrected_data(:, 2), 'b', 'filled', 'DisplayName', 'Hard Iron Corrected');
    ax = gca;
    ax.XAxisLocation = 'origin';
    ax.YAxisLocation = 'origin';
    grid on;
    title('Hard Iron Corrected Magnetic Field Data');
    xlabel('Magnetometer X (T)');
    ylabel('Magnetometer Y (T)');
    legend;
end

function soft_corrected_data = plot_soft_iron_data(corrected_data)
    % Soft iron correction using covariance matrix
    covariance_matrix = cov(corrected_data(:, 1:2)); 
    [eig_vec, eig_val] = eig(covariance_matrix); 

    % Angle of rotation
    angle = atan2(eig_vec(2, 1), eig_vec(1, 1)); 
    rotation_matrix = [cos(-angle), -sin(-angle); sin(-angle), cos(-angle)];

    % Rotating the corrected data
    rotated_data = (rotation_matrix * corrected_data(:, 1:2)')';

    % Normalizing eigenvalues
    soft_iron_matrix = eig_vec * inv(sqrt(diag(diag(eig_val)))) * eig_vec'; 

    % Soft iron correction
    soft_corrected_data = (soft_iron_matrix * rotated_data')';

    % Soft iron plot
    figure; 
    scatter(soft_corrected_data(:, 1), soft_corrected_data(:, 2), 'g', 'filled', 'DisplayName', 'Soft Iron Corrected');
    ax = gca;
    ax.XAxisLocation = 'origin';
    ax.YAxisLocation = 'origin';
    grid on;
    title('Soft Iron Corrected Magnetic Field Data');
    xlabel('Magnetometer X (T)');
    ylabel('Magnetometer Y (T)');
    xlim([-3, 3]); % x-axis limits
    ylim([-3, 3]); % y-axis limits
end

function plot_yaw_data(time, yaw_raw, yaw_corrected)
    figure;
    plot(time, yaw_raw, 'b', 'DisplayName', 'Raw Yaw');
    hold on;
    plot(time, yaw_corrected, 'r', 'DisplayName', 'Corrected Yaw');
    xlabel('Time (s)');
    ylabel('Yaw (rad)');
    title('Yaw Comparison: Raw vs Corrected');
    legend;
    grid on;
    ylim([-6, 6]);
end
