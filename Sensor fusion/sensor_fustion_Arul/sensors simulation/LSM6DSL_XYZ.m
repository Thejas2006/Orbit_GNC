% simulateLSM6DSL.m
% Simulates LSM6DSL IMU behavior (accelerometer + gyroscope)

% Step 1: Setup
duration = 10;           % Duration of simulation in seconds
fs = 104;                % Sample rate (ODR = 104 Hz from datasheet)
t = (0:1/fs:(duration - 1/fs))';  % Time vector - fixed to match data size
numSamples = length(t);

% Create IMU sensor object
imuModel = imuSensor('accel-gyro', ...
    'SampleRate', fs, ...
    'Temperature', 25);

% Step 2: Configure Accelerometer (LSM6DSL)
imuModel.Accelerometer = accelparams( ...
    'MeasurementRange', 4*9.81, ...
    'Resolution', 0.122e-3 * 9.81, ...
    'ConstantBias', 40e-3 * 9.81, ...
    'NoiseDensity', 80e-6 * 9.81, ...
    'BiasInstability', 0.1e-3 * 9.81, ...
    'AxesMisalignment', eye(3), ...
    'RandomWalk', 0.001 ...
);

% Step 3: Configure Gyroscope (LSM6DSL)
imuModel.Gyroscope = gyroparams( ...
    'MeasurementRange', 250 * pi/180, ...
    'Resolution', 8.75e-3 * pi/180, ...
    'ConstantBias', 3 * pi/180, ...
    'NoiseDensity', 4e-3 * pi/180, ...
    'BiasInstability', 0.015 * pi/180, ...
    'AxesMisalignment', eye(3), ...
    'RandomWalk', 0.001 ...
);

% Step 4: Define Ground Truth (stationary)
accelGT = repmat([0, 0, 9.81], numSamples, 1);
gyroGT = zeros(numSamples, 3);

% Step 5: Generate IMU Data
[accelMeas, gyroMeas] = imuModel(accelGT, gyroGT);

% Step 6: Plot All Axes
figure;

subplot(2,1,1);
plot(t, accelMeas(:,1), 'r'); hold on;
plot(t, accelMeas(:,2), 'g');
plot(t, accelMeas(:,3), 'b');
title('Accelerometer Measurement (X:Red, Y:Green, Z:Blue)');
ylabel('Acceleration (m/s²)');
grid on;

subplot(2,1,2);
plot(t, gyroMeas(:,1), 'r'); hold on;
plot(t, gyroMeas(:,2), 'g');
plot(t, gyroMeas(:,3), 'b');
title('Gyroscope Measurement (X:Red, Y:Green, Z:Blue)');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
grid on;
