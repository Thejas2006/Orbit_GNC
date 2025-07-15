clc; clear;

% Time setup
dt = 0.1;
t = 0:dt:10;

% True values (simulate motion with constant acceleration)
true_acc = 2 * ones(size(t));
true_vel = cumtrapz(t, true_acc);
true_pos = cumtrapz(t, true_vel);

% Noisy GPS position measurement
pos_meas = true_pos + randn(size(t)) * 3;  % GPS is noisy!

% Kalman setup
A = [1 dt 0.5*dt^2;
     0 1 dt;
     0 0 1];

H = [1 0 0];     % We only measure position
Q = 0.1 * eye(3);  % Process noise
R = 9;            % Measurement noise (GPS)
P = eye(3);       % Initial uncertainty
x = [0; 0; 0];    % Initial state [pos; vel; acc]

x_est = zeros(3, length(t));

for k = 1:length(t)
    % Predict
    x = A * x;
    P = A * P * A' + Q;
    
    % Measurement
    z = pos_meas(k);
    
    % Kalman Gain
    K = P * H' / (H * P * H' + R);
    
    % Update
    x = x + K * (z - H * x);
    P = (eye(3) - K * H) * P;
    
    % Store result
    x_est(:, k) = x;
end

% Plotting
figure;
subplot(3,1,1);
plot(t, true_pos, 'g', 'LineWidth', 1.5); hold on;
plot(t, pos_meas, 'k:', 'LineWidth', 1);
plot(t, x_est(1,:), 'r--', 'LineWidth', 1.5);
title('Position'); legend('True', 'Measured', 'Estimated'); grid on;

subplot(3,1,2);
plot(t, true_vel, 'g', 'LineWidth', 1.5); hold on;
plot(t, x_est(2,:), 'r--', 'LineWidth', 1.5);
title('Velocity'); legend('True', 'Estimated'); grid on;

subplot(3,1,3);
plot(t, true_acc, 'g', 'LineWidth', 1.5); hold on;
plot(t, x_est(3,:), 'r--', 'LineWidth', 1.5);
title('Acceleration'); legend('True', 'Estimated'); grid on;
