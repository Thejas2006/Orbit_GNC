% Sample rate and time
Fs = 400;                 % Sampling rate in Hz
duration = 10;            % Duration in seconds
t = (0:1/Fs:duration)';   % Time vector

% Accelerometer Parameters (H3LIS331DL)
accelParams = accelparams(...
    'MeasurementRange', 100 * 9.81, ...                      % ±100g in m/s²
    'Resolution', 2 * 100 * 9.81 / (2^12), ...               % 12-bit ADC
    'NoiseDensity', 220e-6 * 9.81, ...                       % 220 µg/√Hz → m/s²/√Hz
    'ConstantBias', [1e-3, -1e-3, 0.5e-3] * 9.81, ...        % Constant bias
    'AxesMisalignment', [0.01; -0.01; 0.005] ...             % Misalignment
);

% Dummy Gyroscope Parameters (must be positive to avoid error)
gyroParams = gyroparams(...
    'MeasurementRange', 1, ...       % Minimal valid positive value
    'Resolution', 0, ...
    'NoiseDensity', 0, ...
    'ConstantBias', [0, 0, 0], ...
    'AxesMisalignment', [0; 0; 0] ...
);

% Create IMU sensor object with accel + dummy gyro
imu = imuSensor('accel-gyro', ...
    'SampleRate', Fs, ...
    'Accelerometer', accelParams, ...
    'Gyroscope', gyroParams ...
);

% Simulated stationary input
N = length(t);
accelTrue = zeros(N, 3);
gyroTrue = zeros(N, 3);

% Simulate IMU readings
[accelReadings, ~] = imu(accelTrue, gyroTrue);

% Plot accelerometer output
figure;
plot(t, accelReadings);
title('Simulated Accelerometer Readings (H3LIS331DL)');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('X', 'Y', 'Z');

% Create timeseries object
ts_h3lis_accel = timeseries(accelReadings, t);

% Save to .mat file
save('h3lis_accel_data.mat', 'ts_h3lis_accel', '-v7.3');
