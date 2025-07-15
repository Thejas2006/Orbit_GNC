% Sample rate and time
Fs = 400;                 % 400 Hz
duration = 10;            % 10 seconds
t = (0:1/Fs:duration)';   % time vector

% --- Accelerometer Parameters (BMI088) ---
accelParams = accelparams( ...
    'MeasurementRange', 24*9.81, ...
    'Resolution', 2 * 24 * 9.81 / (2^16), ...
    'NoiseDensity', 190e-6 * 9.81, ...       % μg/√Hz → m/s^2/√Hz
    'ConstantBias', [0.5e-3, -0.5e-3, 0.5e-3] * 9.81, ... % m/s²
 'AxesMisalignment', [0.01; -0.01; 0.005] ...
 );

% --- Gyroscope Parameters (BMI088) ---
gyroParams = gyroparams( ...
    'MeasurementRange', 2000, ...
    'Resolution', 2 * 2000 / (2^16), ...
    'NoiseDensity', 0.1, ...
    'ConstantBias', [10, -10, 10] * (pi/180/3600), ... % °/h → rad/s
    'AxesMisalignment', [0.01, -0.01, 0.005] ...
);

% --- Create IMU Sensor ---
imu = imuSensor('accel-gyro', ...
    'SampleRate', Fs, ...
    'Accelerometer', accelParams, ...
    'Gyroscope', gyroParams ...
);

% --- Generate some dummy motion (e.g., zero motion) ---
N = length(t);
accelTrue = zeros(N, 3);  % true linear acceleration (no motion)
gyroTrue = zeros(N, 3);   % true angular velocity (no rotation)

% --- Simulate sensor readings ---
[accelReadings, gyroReadings] = imu(accelTrue, gyroTrue);

% --- Plotting ---
figure;
subplot(2,1,1);
plot(t, accelReadings);
title('Simulated Accelerometer Readings (BMI088)');
ylabel('m/s^2');
legend('X','Y','Z');

subplot(2,1,2);
plot(t, gyroReadings);
title('Simulated Gyroscope Readings (BMI088)');
ylabel('deg/s');
xlabel('Time (s)');
legend('X','Y','Z');
