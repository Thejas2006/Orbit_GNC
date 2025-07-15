% Kalman Filter - 1D Position Estimation

% Initialization
num_steps = 50;            % number of time steps
true_position = 1.0;       % constant true value

Q = 0.01;                  % process noise
R = 0.5;                   % measurement noise

x_est = 0;                 % initial estimate
P = 1;                     % initial uncertainty

% Storage for plotting
z_vals = zeros(1, num_steps);       % sensor readings
x_est_vals = zeros(1, num_steps);   % Kalman estimates
true_vals = true_position * ones(1, num_steps); % true position

% Main loop
for k = 1:num_steps
    % Simulate noisy sensor reading
    z = true_position + randn * sqrt(R);
    z_vals(k) = z;

    % Prediction step
    x_pred = x_est;
    P_pred = P + Q;

    % Kalman Gain
    K = P_pred / (P_pred + R);

    % Update step
    x_est = x_pred + K * (z - x_pred);
    P = (1 - K) * P_pred;

    % Store estimate
    x_est_vals(k) = x_est;
end

% Plotting
figure;
plot(1:num_steps, z_vals, 'ro--', 'DisplayName', 'Sensor Readings');
hold on;
plot(1:num_steps, x_est_vals, 'b-', 'LineWidth', 2, 'DisplayName', 'Kalman Estimate');
plot(1:num_steps, true_vals, 'g--', 'LineWidth', 1.5, 'DisplayName', 'True Position');
xlabel('Time Step');
ylabel('Position');
legend;
title('1D Kalman Filter Simulation');
grid on;
  