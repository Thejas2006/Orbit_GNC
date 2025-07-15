clc; clear;

dt = 0.01;
t = 0:dt:10;

% Simulate true angle and gyro
true_angle = sin(t);                        % True angle
true_gyro = cos(t);                         % Angular velocity

% Simulated sensor measurements
gyro_meas = true_gyro + randn(size(t))*0.1;    % Gyro with noise
accel_meas = true_angle + randn(size(t))*0.2;  % Accel with noise

% Kalman Filter Initialization
x = [0; 0];        % [angle; angular_velocity]
P = eye(2);        % Initial uncertainty
A = [1 dt; 0 1];
B = [0; 1];        % For gyro input
H = [1 0];         % We measure angle only

Q = [0.001 0; 0 0.003];  % Process noise
R = 0.04;                % Measurement noise from accelerometer

x_est = zeros(2, length(t));

for k = 1:length(t)
    % Prediction
    u = gyro_meas(k);           % Angular velocity from gyro
    x = A * x + B * u;
    P = A * P * A' + Q;
    
    % Update using accelerometer
    z = accel_meas(k);
    K = P * H' / (H * P * H' + R);
    x = x + K * (z - H * x);
    P = (eye(2) - K * H) * P;
    
    x_est(:, k) = x;
end

% Plotting
figure;
plot(t, true_angle, 'g', 'LineWidth', 1.5); hold on;
plot(t, accel_meas, 'k:', 'LineWidth', 1);
plot(t, x_est(1,:), 'r--', 'LineWidth', 1.5);
title('Angle Estimation with Sensor Fusion');
legend('True Angle', 'Accel Measured', 'Kalman Estimated');
grid on;
