% Sample rate and time
Fs = 400;                 % 400 Hz
duration = 10;            % 10 seconds
t = (0:1/Fs:duration)';   % time vector

% --- Magnetometer Parameters (LIS3MDL) ---
magParams = magparams( ...
    'MeasurementRange', 16e-4, ...           % ±16 gauss (LIS3MDL)
    'Resolution', 2*16e-4/(2^16), ...        % LIS3MDL resolution
    'NoiseDensity', 0.15e-6, ...             % LIS3MDL noise density
    'ConstantBias', [10, -5, 3]*1e-6, ...    % LIS3MDL typical bias
    'AxesMisalignment', 0.5 ...              % Degrees
);

% Create dummy accelerometer parameters
dummyAccelParams = accelparams(...
    'MeasurementRange', 2*9.81, ...          % Minimal range
    'NoiseDensity', 0.001, ...               % Minimal noise
    'ConstantBias', [0 0 0] ...              % No bias
);

% --- IMU Sensor ---
imu = imuSensor('accel-mag', ...
    'SampleRate', Fs, ...
    'Accelerometer', dummyAccelParams, ...   % Required but minimized
    'Magnetometer', magParams ...
);

N = length(t);
accelTrue = zeros(N, 3);  % No acceleration

% --- CORRECTED Rotating Field Implementation ---
baseField = [20, 0, 40]*1e-6; % Earth's field [X,Y,Z] in Tesla
yawRotation = 0.5*t;          % 0.5 rad/sec rotation about Z-axis

% Apply Z-axis rotation to magnetic field
magTrue = zeros(N,3);
magTrue(:,1) = baseField(1)*cos(yawRotation) - baseField(2)*sin(yawRotation);
magTrue(:,2) = baseField(1)*sin(yawRotation) + baseField(2)*cos(yawRotation);
magTrue(:,3) = baseField(3); % Z-component remains constant

% --- Simulate readings ---
[~, magReadings] = imu(accelTrue, magTrue); % Noise automatically included

% --- Plotting ---
figure;
plot(t, magReadings*1e6); % Convert to µT
title('LIS3MDL: Rotating Field + Noise');
ylabel('µT');
xlabel('Time (s)');
legend('X','Y','Z');
grid on;

% Save data
ts_lis_mag = timeseries(magReadings, t);
save('lis_mag_data.mat', 'ts_lis_mag', '-v7.3');

disp(std(magReadings)) 