clc;
clear;

% Time setup
dt = 1;                 % time step (1 second)
N = 50;                 % number of steps
time = 0:dt:N-1;

% True motion (constant velocity model)
true_velocity = 0.5;                     % constant velocity
true_position = 0.2 + true_velocity * time;
true_state = [true_position; true_velocity * ones(1, N)];

% Noisy measurements (only position is measured)
measurement_noise_std = 0.3;
z = true_position + measurement_noise_std * randn(1, N);  % measured positions

% Initialize Kalman filter
x_est = [0; 0];                  % initial guess [position; velocity]
P = eye(2);                      % initial uncertainty

A = [1 dt; 0 1];                 % state transition matrix
H = [1 0];                       % measurement matrix (only position)
Q = [1 0; 0 1] * 0.01;           % process noise
R = measurement_noise_std^2;    % measurement noise

% Store estimates
x_history = zeros(2, N);

% Kalman filter loop
for k = 1:N
    % Prediction
    x_pred = A * x_est;
    P_pred = A * P * A' + Q;

    % Kalman Gain
    K = P_pred * H' / (H * P_pred * H' + R);

    % Update
    x_est = x_pred + K * (z(k) - H * x_pred);
    P = (eye(2) - K * H) * P_pred;

    % Store result
    x_history(:, k) = x_est;
end

% Plot results
figure;
plot(time, true_position, 'g-', 'LineWidth', 2); hold on;
plot(time, z, 'r.', 'MarkerSize', 12);
plot(time, x_history(1, :), 'b-', 'LineWidth', 2);
legend('True Position', 'Measured Position', 'Estimated Position');
xlabel('Time (s)');
ylabel('Position');
title('2D Kalman Filter - Position Estimation');
grid on;

figure;
plot(time, x_history(2, :), 'm-', 'LineWidth', 2); hold on;
yline(true_velocity, 'g--', 'LineWidth', 1.5);
legend('Estimated Velocity', 'True Velocity');
xlabel('Time (s)');
ylabel('Velocity');
title('2D Kalman Filter - Velocity Estimation');
grid on;
