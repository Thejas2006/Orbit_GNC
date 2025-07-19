% Sample rate and time
Fs = 400;                 % 400 Hz
duration = 10;            % 10 seconds
t = (0:1/Fs:duration)';   % time vector

% --- Accelerometer Parameters (LSM6DSL) ---
accelParams = accelparams( ...
    'MeasurementRange', 8*9.81, ...         % ±8g (LSM6DSL max range)
    'Resolution', 2 * 8 * 9.81 / (2^16), ... % LSM6DSL resolution
    'NoiseDensity', 90e-6 * 9.81, ...       % LSM6DSL noise density
    'ConstantBias', [0.002, -0.0015, 0.001]*9.81, ... % LSM6DSL typical bias
    'AxesMisalignment', [0.01; -0.01; 0.005] ...
);

% --- Gyroscope Parameters (LSM6DSL) ---
gyroParams = gyroparams( ...
    'MeasurementRange', 2000, ...            % ±2000dps (LSM6DSL max)
    'Resolution', 2 * 2000 / (2^16), ...     % LSM6DSL resolution
    'NoiseDensity', 0.01, ...                % LSM6DSL noise density
    'ConstantBias', [10, -10, 10] * (pi/180/3600), ... % Same unit conversion
    'AxesMisalignment', [0.01, -0.01, 0.005] ...
);

% --- IMU Sensor ---
imu = imuSensor('accel-gyro', ...
    'SampleRate', Fs, ...
    'Accelerometer', accelParams, ...
    'Gyroscope', gyroParams ...
);

N = length(t);
accelTrue = zeros(N, 3);  % true linear acceleration (no motion)
gyroTrue = zeros(N, 3);   % true angular velocity (no rotation)

% --- Simulate sensor readings ---
[accelReadings, gyroReadings] = imu(accelTrue, gyroTrue);

% --- Plotting ---
figure;
subplot(2,1,1);
plot(t, accelReadings);
title('Simulated Accelerometer Readings (LSM6DSL)');
ylabel('m/s^2');
legend('X','Y','Z');
ts_lsm_accel = timeseries(accelReadings, t);

subplot(2,1,2);
plot(t, gyroReadings);
title('Simulated Gyroscope Readings (LSM6DSL)');
ylabel('deg/s');
xlabel('Time (s)');
legend('X','Y','Z');
ts_lsm_gyro = timeseries(gyroReadings, t);  

save('lsm_accel_data.mat', 'ts_lsm_accel', '-v7.3');
save('lsm_gyro_data.mat', 'ts_lsm_gyro', '-v7.3');