dt = 0.01;              % time step (100 Hz)
t = 0:dt:10;            % 10 seconds

% True angles (simulate a small rotation over time)
roll_true = 5 * sind(2*pi*0.1*t);    % degrees
pitch_true = 3 * sind(2*pi*0.1*t);
yaw_true = 10 * sind(2*pi*0.1*t);

% Angular velocity (gyro): derivative of angles
gyro_x = [0 diff(roll_true)] / dt;
gyro_y = [0 diff(pitch_true)] / dt;
gyro_z = [0 diff(yaw_true)] / dt;

% Add some noise to gyro
gyro_x = gyro_x + randn(size(gyro_x))*0.5;
gyro_y = gyro_y + randn(size(gyro_y))*0.5;
gyro_z = gyro_z + randn(size(gyro_z))*0.5;

% Acc & Mag simulate angle measurements with noise
acc_roll = roll_true + randn(size(t))*1.0;
acc_pitch = pitch_true + randn(size(t))*1.0;
mag_yaw = yaw_true + randn(size(t))*2.0;

x = [0; 0; 0];    % initial estimate: [roll; pitch; yaw]
P = eye(3);       % initial uncertainty

Q = 0.1 * eye(3); % process noise
R = diag([1.5, 1.5, 3]);  % measurement noise (more noise in yaw)

X_est = zeros(3, length(t));  % store estimates

for k = 2:length(t)
    % --- Predict Step ---
    omega = [gyro_x(k); gyro_y(k); gyro_z(k)];  % angular velocity
    x_pred = x + omega * dt;
    P = P + Q;

    % --- Measurement Step ---
    z = [acc_roll(k); acc_pitch(k); mag_yaw(k)];

    % --- Kalman Gain ---
    K = P / (P + R);

    % --- Update Step ---
    x = x_pred + K * (z - x_pred);
    P = (eye(3) - K) * P;

    % Store result
    X_est(:,k) = x;
end

figure;
subplot(3,1,1);
plot(t, roll_true, 'g', t, X_est(1,:), 'b'); title('Roll');
legend('True','Estimated');

subplot(3,1,2);
plot(t, pitch_true, 'g', t, X_est(2,:), 'b'); title('Pitch');
legend('True','Estimated');

subplot(3,1,3);
plot(t, yaw_true, 'g', t, X_est(3,:), 'b'); title('Yaw');
legend('True','Estimated');
